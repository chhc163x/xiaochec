#ifndef __GRAY_H
#define __GRAY_H

/*感为无MCU八路灰度循迹模块驱动（模拟量输出版）：
  VCC接5V、GND共地、AD0/AD1/AD2三根线选通道、OUT输出模拟电压（黑线电压低、白底电压高）
  通道选择按模拟开关真值表：AD2 AD1 AD0 = 通道号二进制（AD0为最低位）
  EN=使能脚默认使能可悬空；ERR=过曝报错输出（强光直射读数不可信时拉高，可悬空，别在阳光直射下用）
  每10ms控制周期调一次Gray_Read()把8路全部读进缓存（约4.5ms），
  Gray_GetState/GetActiveCount/GetError直接读缓存，不再访问ADC
  每路各用各的相对阈值（读数掉到本路白色基准的60%以下=压线）：
  各路白底因光照不均可以差很多（如600/1500）都正常，绝对阈值会把白底低的探头误判成压线；
  手影只让读数小掉一点（约30%）不误判，压线时基准冻结不跟线走，环境光缓变自动适应
  ch0=最左路、ch7=最右路；若发现小车纠偏方向反了，把GRAY_DIR改成-1即可*/

#define GRAY_CHANNELS		8		//探头路数
#define GRAY_ACTIVE_LEVEL	0		//压线时的电平归一化开关：0=压黑线时电压低(默认)，改成1=压线电压高
#define GRAY_DIR			1		//误差方向开关：模块装反了(左右镜像)把GRAY_DIR改成-1
#define GRAY_WHITE_RISE_SHIFT	4	//白色基准上升速度：没压线且读数比基准亮时，每拍追近1/16（约0.2秒跟上）
#define GRAY_WHITE_FALL_SHIFT	6	//白色基准下降速度：没压线且读数比基准暗时，每拍追近1/64（约0.6秒，手影拖不垮基准）
#define GRAY_CENTER_MASK	0x18	//中间两路ch3/ch4的位掩码（备用：V16转向寻线改为任意路压线即恢复循迹，不再用中心掩码）

void Gray_Init(void);				//初始化：PA0=OUT(ADC1通道0)，PA8/PA11/PA12=AD0/AD1/AD2
void Gray_Read(void);				//8路全部读一遍并算好二值化缓存（每10ms调一次）
uint8_t Gray_GetState(void);		//8路状态，bit0~bit7对应ch0(最左)~ch7(最右)，1=压线
uint8_t Gray_GetActiveCount(void);	//压线路数，范围0~8
int8_t Gray_GetError(void);			//加权误差，范围-10~+10，正=线偏右
uint16_t Gray_GetRaw(uint8_t Ch);	//某一路ADC原始值，范围0~4095（调试用）

#endif
