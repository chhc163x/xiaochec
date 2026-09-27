#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_LEFT_DIR	-1		//左轮编码器计数方向：轮子前进时读数应为正，若实测为负则改成-1（实测：左轮正转读数-73，已改-1）
#define ENCODER_RIGHT_DIR	1		//右轮编码器计数方向：同上（实测右轮反转读数-20，方向正确）

void Encoder_Init(void);			//初始化TIM3/TIM4编码器模式（左轮=PA6/PA7，右轮=PB6/PB7）
void Encoder_Update(void);			//每10ms调用一次，读取计数器并计算本拍脉冲增量
int16_t Encoder_GetLeft(void);		//获取左轮本拍脉冲增量（计数/拍，有符号）
int16_t Encoder_GetRight(void);		//获取右轮本拍脉冲增量（计数/拍，有符号）

#endif
