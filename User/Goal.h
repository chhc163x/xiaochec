#ifndef __GOAL_H
#define __GOAL_H

#include "stm32f10x.h"                  // Device header

/*目标选择模块（KEY4=PC15）：任务3暂停态按1~4下选目标角A~D，按过后OLED第2行常显
  Goal:A~D，详见Goal.c；cam串口消息就绪后优先于按键目标*/
void Goal_Poll(uint8_t Running, uint8_t TaskNum);	//主循环每拍调用：KEY4消抖+按几下计数
uint8_t Goal_Get(void);				//当前按键选的目标：1/2/3/4=角A/B/C/D，0=还没选过

#endif
