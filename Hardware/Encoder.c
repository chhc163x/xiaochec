#include "stm32f10x.h"                  // Device header
#include "Encoder.h"

uint16_t LeftLastCount;			//左轮上一次的计数值
uint16_t RightLastCount;		//右轮上一次的计数值
int16_t LeftDelta;				//左轮本拍脉冲增量
int16_t RightDelta;				//右轮本拍脉冲增量

/**
  * 函    数：编码器初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：TIM3编码器模式接左轮编码器（PA6=A相、PA7=B相）
  *           TIM4编码器模式接右轮编码器（PB6=A相、PB7=B相）
  *           编码器模式为TI1+TI2四倍频：A/B两相的每个边沿都计数，
  *           一圈计数值 = 编码器线数 × 4 × 减速比（MG310约为13×4×20=1040）
  *           红线1：绝不调用GPIO_PinRemapConfig重映射（会踩到PB0/PB1等引脚）
  *           红线2：绝不配置TIM4的OC3/OC4输出（PB8/PB9是OLED引脚，配了OLED会花屏）
  */
void Encoder_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//开启TIM3的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//开启TIM4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟

	/*GPIO初始化，编码器输入用上拉输入（霍尔传感器多为开漏输出，需要上拉）*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;		//PA6=A相、PA7=B相（TIM3，左轮）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;		//PB6=A相、PB7=B相（TIM4，右轮）
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*时基单元初始化：不分频、满量程计数，计数方向由编码器模式自动控制*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;			//ARR，计数器最大计到65535后回绕
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;			//PSC，不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	/*编码器接口配置：TI1+TI2组合四倍频，两路输入均为上升沿有效*/
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	/*输入捕获初始化：给CH1、CH2各配置一次（带数字滤波，滤除霍尔信号的毛刺）*/
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	//输入直连TI1/TI2
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;					//输入滤波，若高速丢脉冲可降到0x0A以下
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	/*TIM使能*/
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：编码器计数更新
  * 参    数：无
  * 返 回 值：无
  * 注意事项：每个10ms控制周期调用一次。
  *           计数器是16位无符号数，会从65535回绕到0，
  *           用无符号减法再转int16_t即可正确得到带符号的增量（只要增量不超过32767）
  */
void Encoder_Update(void)
{
	uint16_t LeftCount = TIM_GetCounter(TIM3);			//读左轮编码器当前计数值
	uint16_t RightCount = TIM_GetCounter(TIM4);			//读右轮编码器当前计数值
	LeftDelta = (int16_t)(LeftCount - LeftLastCount);	//本拍增量 = 当前值 - 上次值
	RightDelta = (int16_t)(RightCount - RightLastCount);
	LeftLastCount = LeftCount;							//记录本次值，供下一拍使用
	RightLastCount = RightCount;
}

/**
  * 函    数：获取左轮本拍脉冲增量
  * 参    数：无
  * 返 回 值：本拍脉冲增量（计数/拍），轮子前进时为正
  */
int16_t Encoder_GetLeft(void)
{
	return LeftDelta * ENCODER_LEFT_DIR;
}

/**
  * 函    数：获取右轮本拍脉冲增量
  * 参    数：无
  * 返 回 值：本拍脉冲增量（计数/拍），轮子前进时为正
  */
int16_t Encoder_GetRight(void)
{
	return RightDelta * ENCODER_RIGHT_DIR;
}
