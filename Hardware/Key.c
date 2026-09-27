#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：共4个按键：KEY1=PB1（暂停/继续），KEY2=PC13（暂停态切换任务），
  *           KEY3=PC14（调距键：任务2暂停态短按+1cm、长按-1cm），
  *           KEY4=PC15（选目标键：任务3暂停态按1~4下选角A~D；PC14/PC15空着没用）
  *           PC13/PC14/PC15不是5V容忍脚，按键只能采用一端接地、另一端接引脚（PB1同款接法）
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);		//开启GPIOC的时钟（KEY2/3/4在PC13/14/15）

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//PB1初始化上拉输入（暂停/继续键）

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOC, &GPIO_InitStructure);						//PC14初始化上拉输入（调距键KEY3）

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);						//PC15初始化上拉输入（选目标键KEY4）

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);						//PC13初始化上拉输入（切换任务键，原PB11被MPU6050的I2C2_SDA占用）
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;		//定义变量，默认键码值为0

	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
	{
		Delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 1;												//置键码为1
	}

	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0)			//读PC13输入寄存器的状态，如果为0，则代表按键2按下
	{
		Delay_ms(20);											//延时消抖
		while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0);	//等待按键松手
		Delay_ms(20);											//延时消抖
		KeyNum = 2;												//置键码为2
	}

	return KeyNum;			//返回键码值，如果没有按键按下，所有if都不成立，则键码为默认值0
}

/**
  * 函    数：按键3非阻塞读取（调距键）
  * 参    数：无
  * 返 回 值：1=PC14按下 0=没按
  * 注意事项：KEY3要支持短按/长按（+1cm/−1cm），需要连续轮询原始电平，
  *           由main自己做消抖和短/长按判定；PB1/PC13仍走阻塞式Key_GetNum
  */
uint8_t Key3_GetPress(void)
{
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 0) return 1;	//PC14按下（一端接地接法，按下=低电平）
	return 0;
}

/**
  * 函    数：按键4非阻塞读取（选目标键）
  * 参    数：无
  * 返 回 值：1=PC15按下 0=没按
  * 注意事项：KEY4要数"按了几下"，需要连续轮询原始电平，
  *           由Goal.c自己做消抖和按几下计数；PB1/PC13仍走阻塞式Key_GetNum
  */
uint8_t Key4_GetPress(void)
{
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 0) return 1;	//PC15按下（一端接地接法，按下=低电平）
	return 0;
}
