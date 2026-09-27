#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);
uint8_t Key_GetNum(void);			//阻塞式获取键码：按住会卡到松手（PB1=暂停/继续，PC13=暂停态切子任务）
uint8_t Key3_GetPress(void);		//非阻塞读KEY3（PC14）按下状态：1=按下（不消抖不等松手，短/长按判定在Tune.c里）
uint8_t Key4_GetPress(void);		//非阻塞读KEY4（PC15）按下状态：1=按下（不消抖不等松手，按几下计数在Goal.c里）

#endif
