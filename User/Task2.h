#ifndef __TASK2_H
#define __TASK2_H

#include "stm32f10x.h"                  // Device header
#include "Display.h"

void Task2_Init(uint8_t SubRun);		//任务2状态机初始化：清零积分/状态，记住子任务号（启动前调用；不重置中点距离）
uint8_t Task2_Run(DispInfo *D);			//每10ms调一次推进一步：返回1=任务完成（已停车，main统一收尾） 0=继续
void Task2_Tune(uint8_t SubRun, int8_t Delta);	//调距：指定子任务中点距离±Delta cm（钳位1~99，只改内存，确定后写死Config重烧）
int16_t Task2_GetMidDist(uint8_t SubRun);		//读指定子任务中点距离（cm，OLED调距显示/串口上报用；SubRun由main传入，不依赖启动时的内部Sub）

#endif
