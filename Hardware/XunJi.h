#ifndef __XUNJI_H
#define __XUNJI_H

#define XUNJI_ACTIVE_LEVEL	1		//探头压到黑线时DO输出的电平：实测本模块压黑线输出高电平，故设为1（1=压线）
#define XUNJI_DIR			1		//循迹误差方向：1=DO1在车头最左（正常装法）；模块装反了（DO1在车头最右）就改成-1

void XunJi_Init(void);				//7路循迹初始化，上拉输入
uint8_t XunJi_GetState(void);		//读7路状态：bit0~bit6对应DO1(最左)~DO7(最右)，1=压线
uint8_t XunJi_GetActiveCount(void);	//返回压线探头个数0~7，用于丢线判断（任务3直角检测）
int8_t XunJi_GetError(void);		//加权误差-6~+6：正=线偏右，负=线偏左，0=居中

#endif
