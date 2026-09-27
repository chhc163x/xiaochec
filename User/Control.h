#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"                  // Device header

int16_t PI_Control(int16_t Target, int16_t Actual, int16_t *Integral, int16_t MinOut, int16_t MaxOut);	//PI速度闭环（输出=KP×误差+KI×积分÷16，防积分饱和）
int16_t TurnRate_PI(int32_t Target, int32_t Measured, int32_t *Integral);								//转向角速度PI闭环（输出=(误差+积分)÷32，限幅±TURN_DUTY_MAX）

#endif
