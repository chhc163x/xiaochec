#ifndef __RESERVED_H
#define __RESERVED_H

/*================================================================================
  预留引脚定义（本工程暂未使用这些模块，仅作接线预留与后续扩展参考）
  - 张大头步进电机驱动器：PUL=脉冲脚、DIR=方向脚、EN=使能脚，3.3V逻辑电平可直接驱动
    （电机供电需另接10~29V直流电源，不能接在3.3V上）
  - MOS开关模块：SIG高电平导通（负载串在模块的VCC与VOUT之间），3.3V可直接驱动
  - 轮趣TB6612FNG稳压版的ADC脚：电池电压检测输出（板上100k+10k分压，输出=电池电压/11，12V电池约1.09V），接PA1可测电池电量
  预留引脚：PA1=TB6612 ADC，PB0=MOS SIG，PB14=步进PUL，PB15=步进DIR，PB5=步进EN
  （PC14已让给KEY3调距键）
================================================================================*/

#define RESV_BAT_ADC	GPIO_Pin_1		//PA1：TB6612板ADC（电池电压/11检测输出，ADC1_IN1，未启用）
#define RESV_MOS_SIG	GPIO_Pin_0		//PB0：MOS开关模块信号脚（高电平导通）
#define RESV_STEP_PUL	GPIO_Pin_14		//PB14：张大头步进驱动器PUL脉冲脚
#define RESV_STEP_DIR	GPIO_Pin_15		//PB15：张大头步进驱动器DIR方向脚
#define RESV_STEP_EN	GPIO_Pin_5		//PB5：张大头步进驱动器EN使能脚

#endif
