#include "stm32f10x.h"                  // Device header
#include "XunJi.h"

/**
  * 函    数：7路循迹初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：DO1(最左)=PA8、DO2=PA9、DO3=PA10、DO4(中间)=PA11、
  *           DO5=PA12、DO6=PB0、DO7(最右)=PA0
  *           （移植时DO7原为PB10，因PB10已被MPU6050的I2C2_SCL占用而改到PA0）
  *           全部配置为上拉输入，模块的DO输出直接高低电平，无需外部上拉电阻
  */
void XunJi_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//DO1~DO5接PA8~PA12，DO7接PA0

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//DO6接PB0
}

/**
  * 函    数：读取7路循迹状态
  * 参    数：无
  * 返 回 值：7路状态，bit0~bit6对应DO1(最左)~DO7(最右)，1=压线
  * 注意事项：不同模块压黑线时的输出电平可能不同，统一按XUNJI_ACTIVE_LEVEL归一化，
  *           如果发现显示的状态反了（压线显示0、白底显示1），
  *           把XunJi.h里的XUNJI_ACTIVE_LEVEL改成1即可
  */
uint8_t XunJi_GetState(void)
{
	uint8_t State = 0;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == XUNJI_ACTIVE_LEVEL) State |= 0x01;	//DO1 最左
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) == XUNJI_ACTIVE_LEVEL) State |= 0x02;	//DO2
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) == XUNJI_ACTIVE_LEVEL) State |= 0x04;	//DO3
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == XUNJI_ACTIVE_LEVEL) State |= 0x08;	//DO4 中间
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == XUNJI_ACTIVE_LEVEL) State |= 0x10;	//DO5
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == XUNJI_ACTIVE_LEVEL) State |= 0x20;	//DO6
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == XUNJI_ACTIVE_LEVEL) State |= 0x40;	//DO7 最右（原PB10改到PA0，PB10让给MPU6050 I2C2_SCL）
	return State;
}

/**
  * 函    数：统计压线探头个数
  * 参    数：无
  * 返 回 值：压线探头个数，范围0~7
  * 注意事项：用于丢线判断（任务3：连续几拍压线个数为0=到达直角）
  */
uint8_t XunJi_GetActiveCount(void)
{
	uint8_t State = XunJi_GetState();
	uint8_t Count = 0;
	uint8_t i;
	for (i = 0; i < 7; i++)
	{
		if (State & (0x01 << i))		//依次判断每一位是否为1
		{
			Count++;
		}
	}
	return Count;
}

/**
  * 函    数：计算循迹加权误差
  * 参    数：无
  * 返 回 值：误差值，范围-6~+6
  * 注意事项：误差 = -3×D1 -2×D2 -1×D3 +1×D5 +2×D6 +3×D7（D4居中不参与）
  *           误差为正说明黑线偏右，小车应向右转；为负说明黑线偏左；
  *           越靠外侧的探头权重越大，离中线越远纠偏越猛
  */
int8_t XunJi_GetError(void)
{
	uint8_t State = XunJi_GetState();
	int8_t Error = 0;
	if (State & 0x01) Error -= 3;	//DO1 最左
	if (State & 0x02) Error -= 2;	//DO2
	if (State & 0x04) Error -= 1;	//DO3
	if (State & 0x10) Error += 1;	//DO5
	if (State & 0x20) Error += 2;	//DO6
	if (State & 0x40) Error += 3;	//DO7 最右
	return (int8_t)(Error * XUNJI_DIR);	//乘方向开关：模块装反了把XUNJI_DIR改成-1即可
}
