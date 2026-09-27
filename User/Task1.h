#ifndef __TASK1_H
#define __TASK1_H

#include "stm32f10x.h"                  // Device header
#include "Display.h"

void Task1_Init(uint8_t Corners, uint8_t AutoStop);	//任务1状态机初始化：清零积分/状态，设置目标直角数；AutoStop=1到目标角自动停，=0不停继续寻线转弯（任务2借道用）
uint8_t Task1_Run(DispInfo *D);			//每10ms调一次推进一步：返回1=任务完成（已停车，main统一收尾） 0=继续
uint8_t Task1_GetCorners(void);			//查询已过直角个数（任务2借道时用）
uint8_t Task1_GetState(void);			//查询循迹状态机：0=跟随 2=转向寻线（任务2借道时用）
void Task1_SetTurnDir(int8_t Dir);		//设置寻线转向方向：1=原地右转（默认）、-1=原地左转（任务3的A/B目标在B角拐弯用；须在Task1_Init之后调）

#endif
