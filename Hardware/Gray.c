#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Gray.h"

/*==================== 内部缓存 ====================*/
static uint16_t GrayRaw[GRAY_CHANNELS];		//8路ADC原始值（12bit，0~4095）
static uint16_t GrayWhite[GRAY_CHANNELS];		//每路白色基准（慢速滑动平均；压线时冻结；0=还没采到过首拍）
static uint8_t GrayState = 0;				//8路二值化状态，bit0~bit7=ch0(最左)~ch7(最右)，1=压线
static int8_t GrayError = 0;				//加权循迹误差

/*误差权重：越靠外侧权重越大，离中线越远纠偏越猛（ch0=最左，ch7=最右，中间4路对称）*/
static const int8_t GrayWeight[GRAY_CHANNELS] = {-4, -3, -2, -1, 1, 2, 3, 4};

/**
  * 函    数：8路灰度模块初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：OUT接PA0=ADC1通道0（模拟输入）；AD0/AD1/AD2接PA8/PA11/PA12（推挽输出）
  *           ADC时钟=PCLK2/6=12MHz（不超过14MHz），通道0采样时间55.5周期
  */
void Gray_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);		//开启ADC1的时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);							//ADC时钟=PCLK2÷6=12MHz

	/*GPIO初始化：PA0=模拟输入（接灰度模块OUT）*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;				//模拟输入模式（不能配成普通输入，会漏电影响精度）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIO初始化：PA8/PA11/PA12=推挽输出（接灰度模块AD0/AD1/AD2选通道）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12);	//初始选中ch0

	/*ADC1初始化：单通道、软件触发、右对齐*/
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);	//PA0=通道0
	ADC_Cmd(ADC1, ENABLE);

	/*复位校准并校准（上电必须做一次，保证读数准确）*/
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);

	/*每路白色基准清零：首拍读数直接作为基准，之后慢速跟随（0=未初始化标记）*/
	{
		uint8_t i;
		for (i = 0; i < GRAY_CHANNELS; i++)
		{
			GrayWhite[i] = 0;
		}
	}
}

/**
  * 函    数：选中模拟开关通道并读一次ADC
  * 参    数：Ch 通道号0~7
  * 返 回 值：ADC原始值，范围0~4095
  * 注意事项：切通道后延时500µs等模块输出稳定，再软件触发一次转换
  */
static uint16_t Gray_ReadCh(uint8_t Ch)
{
	if (Ch & 0x01) GPIO_SetBits(GPIOA, GPIO_Pin_8);			//AD0（PA8）
	else GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	if (Ch & 0x02) GPIO_SetBits(GPIOA, GPIO_Pin_11);		//AD1（PA11）
	else GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	if (Ch & 0x04) GPIO_SetBits(GPIOA, GPIO_Pin_12);		//AD2（PA12）
	else GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	Delay_us(500);											//等待通道切换和模块输出稳定（该模块输出建立慢，100µs会采到上一路过渡值，读数乱跳）

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);					//软件触发一次转换
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);	//等待转换完成
	return ADC_GetConversionValue(ADC1);					//读回结果（软件触发会自动清EOC）
}

/**
  * 函    数：8路全部读一遍并计算二值化缓存
  * 参    数：无
  * 返 回 值：无
  * 注意事项：每10ms控制周期调一次（8路约4.5ms），GetState/GetActiveCount/GetError
  *           全部读本次结果，不在函数内部再访问ADC；
  *           每路各用各的相对阈值：读数掉到本路白色基准的60%以下才判压线。
  *           各路白底因光照不均可以差很多（如600/1500都正常），绝对阈值会把
  *           白底低的探头误判成压线（全白读出11000000就是这种情况）；
  *           白色基准慢速滑动平均且压线时冻结：手影只掉约30%不误判，
  *           线压在探头下多久基准也不会被拖下去，环境光缓变自动适应
  */
void Gray_Read(void)
{
	uint16_t Th;						//二值化阈值
	uint8_t State = 0;
	uint8_t i;

	for (i = 0; i < GRAY_CHANNELS; i++)		//8路全部读一遍，每路按自己的白色基准算相对阈值并二值化
	{
		GrayRaw[i] = Gray_ReadCh(i);

		if (GrayWhite[i] == 0)				//首拍：读数直接作为该路白色基准，本拍不判压线
		{
			GrayWhite[i] = GrayRaw[i];
			continue;
		}

		if (GRAY_ACTIVE_LEVEL == 0)									//默认：压黑线时电压低
		{
			Th = (GrayWhite[i] * 6) / 10;							//相对阈值：掉到白基准60%以下=压线
			if (GrayRaw[i] < Th)
			{
				State |= (uint8_t)(0x01 << i);						//压线：本拍冻结白色基准，线压多久都不会把基准拖下去
				continue;
			}
		}
		else														//压线时电压高的模块
		{
			Th = (GrayWhite[i] * 14) / 10;							//相对阈值：升到白基准140%以上=压线
			if (GrayRaw[i] >= Th)
			{
				State |= (uint8_t)(0x01 << i);
				continue;
			}
		}

		/*没压线：白色基准慢速跟随读数（升快降慢，手影、灯光缓变都拖不垮基准）*/
		if (GrayRaw[i] > GrayWhite[i])
		{
			GrayWhite[i] += (uint16_t)((GrayRaw[i] - GrayWhite[i]) >> GRAY_WHITE_RISE_SHIFT);
		}
		else
		{
			GrayWhite[i] -= (uint16_t)((GrayWhite[i] - GrayRaw[i]) >> GRAY_WHITE_FALL_SHIFT);
		}
	}
	GrayState = State;

	/*计算加权误差：正=线偏右（车应向右纠），范围-10~+10*/
	GrayError = 0;
	for (i = 0; i < GRAY_CHANNELS; i++)
	{
		if (State & (0x01 << i)) GrayError += GrayWeight[i];
	}
	GrayError *= GRAY_DIR;					//方向开关：装反了把Gray.h里的GRAY_DIR改成-1
}

/**
  * 函    数：获取8路状态缓存
  * 参    数：无
  * 返 回 值：bit0~bit7对应ch0(最左)~ch7(最右)，1=压线
  */
uint8_t Gray_GetState(void)
{
	return GrayState;
}

/**
  * 函    数：统计压线路数
  * 参    数：无
  * 返 回 值：压线路数，范围0~8
  * 注意事项：用于丢线判断（连续几拍压线数为0=到达直角）
  */
uint8_t Gray_GetActiveCount(void)
{
	uint8_t Count = 0;
	uint8_t i;
	for (i = 0; i < GRAY_CHANNELS; i++)
	{
		if (GrayState & (0x01 << i))		//依次判断每一位
		{
			Count++;
		}
	}
	return Count;
}

/**
  * 函    数：获取加权循迹误差缓存
  * 参    数：无
  * 返 回 值：误差值，范围-10~+10，正=线偏右
  */
int8_t Gray_GetError(void)
{
	return GrayError;
}

/**
  * 函    数：获取某一路的ADC原始值
  * 参    数：Ch 通道号0~7
  * 返 回 值：ADC原始值，范围0~4095，越界返回0
  * 注意事项：调试用（诊断灰度读数乱跳/常亮时看原始值最直观）
  */
uint16_t Gray_GetRaw(uint8_t Ch)
{
	if (Ch >= GRAY_CHANNELS) return 0;
	return GrayRaw[Ch];
}
