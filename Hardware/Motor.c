#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/**
  * 函    数：电机初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：驱动板为轮趣TB6612FNG稳压版（D153C）——电机侧6PIN排线直插MG513X
  *           （排线内含电机电源线+编码器A/B相，编码器由板载3.3V供电），STM32只接控制侧排针：
  *           PA4=AIN1、PA5=AIN2（A电机，左轮）；PB12=BIN1、PB13=BIN2（B电机，右轮）
  *           PA2=PWMA、PA3=PWMB（TIM2的PWM输出，10kHz）；STBY接3.3V常高
  *           编码器输出E1A/E1B（左轮）接PA6/PA7（TIM3），E2A/E2B（右轮）接PB6/PB7（TIM4）
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;		//PA4接TB6612的AIN1，PA5接AIN2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA4和PA5引脚初始化为推挽输出

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;		//PB12接TB6612的BIN1，PB13接BIN2
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB12和PB13引脚初始化为推挽输出

	/*方向脚先全部拉低，防止PWM初始化前电机误动作*/
	GPIO_ResetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);

	PWM_Init();			//PA2输出PWMA（通道3），PA3输出PWMB（通道4），PWM频率10kHz
}

/**
  * 函    数：A电机（左轮）速度控制
  * 参    数：Speed 速度，范围-100~100，正=前进、负=后退，绝对值即占空比
  * 返 回 值：无
  */
void Motor_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);		//AIN1=1
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);		//AIN2=0，正转
		PWM_SetCompare3(Speed);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);		//AIN1=0
		GPIO_SetBits(GPIOA, GPIO_Pin_5);		//AIN2=1，反转
		PWM_SetCompare3(-Speed);
	}
}

/**
  * 函    数：B电机（右轮）速度控制
  * 参    数：Speed 速度，范围-100~100，正=前进、负=后退，绝对值即占空比
  * 返 回 值：无
  */
void MotorB_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);		//BIN1=1
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);		//BIN2=0，正转
		PWM_SetCompare4(Speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);		//BIN1=0
		GPIO_SetBits(GPIOB, GPIO_Pin_13);		//BIN2=1，反转
		PWM_SetCompare4(-Speed);
	}
}

/**
  * 函    数：双电机停止
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Stop(void)
{
	Motor_SetSpeed(0);
	MotorB_SetSpeed(0);
}
