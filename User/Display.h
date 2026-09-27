#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "stm32f10x.h"                  // Device header

/*OLED显示数据：任务函数每拍把要显示的值写进结构体，main把系统状态填好后调Display_Refresh*/
typedef struct
{
	uint8_t Running;		//运行标志：0=暂停 1=运行中
	uint8_t TaskNum;		//当前任务号
	uint8_t SubRun;			//当前子任务号
	uint8_t CornerCount;	//已过直角个数
	uint8_t TargetCorner;	//目标直角个数
	int32_t GyroSumZ;		//陀螺Z轴积分角（LSB，1°=1640）
	int8_t PwmL;			//左电机PWM输出
	int8_t PwmR;			//右电机PWM输出
	int8_t LastError;		//最近一次循迹误差
	int16_t OdomCm;			//累计里程（cm，main换算好：推车标定每厘米脉冲数看它）
	uint8_t TuneMode;		//调距模式：1=第2行显示中点距离（Mid:+xxx cm，按过KEY3调距键后显示3秒，任务2专用）
	int16_t TuneCm;			//当前子任务中点距离（cm，调距模式时第2行显示）
	uint8_t GoalMode;		//选目标显示：1=第2行显示按键选的目标（Goal:A~D，任务3暂停态常显）
	uint8_t GoalNum;		//按键选的目标角号（1/2/3/4=A/B/C/D，GoalMode=1时第2行显示）
	uint8_t CamState;		//cam识别显示状态（任务3专用，跑的时候也一直显示）：
							//  0 = 不显示（第2行按老规则走 PWM/Mid/Goal/Ang）
							//  1 = 相机在线但没识别到卡 → 第2行显示 "Cam:--"
							//  2 = 识别到了            → 第2行显示 "Cam:5>C"
							//区分 1 和 2 是为了让"相机在看着但没卡"和"根本没连上"一眼能分出来
	uint8_t CamNum;			//MaixCam识别到的目标角号（1/2/3/4=A/B/C/D，0=还没识别到）
							//注意这是"角号"不是"卡号"！卡号是 2×角号−1（角1=卡1、角2=卡3、角3=卡5、角4=卡7），
							//Display.c 里用 CardDigit[] 把角号换成卡号显示，这样屏幕上是"3>B"而不是"2>B"
} DispInfo;

void Display_Refresh(DispInfo *D);	//每10ms调一次，每次只刷一小片（≤3字符），23片刷完整屏

#endif
