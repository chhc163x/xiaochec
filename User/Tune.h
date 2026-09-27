#ifndef __TUNE_H
#define __TUNE_H

#include "stm32f10x.h"                  // Device header

/*调距模块（KEY3=PC14）：任务2暂停态短按+1cm、长按-1cm微调"转弯后到中点"的距离，
  按过后OLED第2行显示Mid值3秒，详见Tune.c*/
void Tune_Poll(uint8_t Running, uint8_t TaskNum, uint8_t SubRun);	//主循环每拍调用：KEY3消抖+短/长按判定并调Task2中点距离
uint8_t Tune_ShowActive(void);			//1=按过KEY3后3秒内，OLED第2行应显示Mid值（main据此填Disp.TuneMode）

#endif
