#ifndef __TASK3_H
#define __TASK3_H

#include "stm32f10x.h"                  // Device header
#include "Display.h"

void Task3_Init(void);					//任务3状态机初始化：清零积分/状态（启动前调用）
uint8_t Task3_Run(DispInfo *D);			//每10ms调一次推进一步：返回1=任务完成（已停车，main统一收尾） 0=继续

#endif
