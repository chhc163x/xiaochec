#include "stm32f10x.h"                  // Device header
#include "Key.h"
#include "Task2.h"
#include "Tune.h"

/*==================== 调距模块（KEY3=PC14） ====================*/
/*任务2暂停态微调"转弯后走到中点"的距离：KEY3短按+1cm、长按≥1秒-1cm（松开沿触发），
  按过后OLED第2行显示Mid值3秒自动切回Ang/O。状态全部static自持（模块化：main.c
  只每拍调一次Tune_Poll并读Tune_ShowActive，不用管消抖/长按/显示计时细节）*/

static uint8_t K3Old = 0;			//上一拍原始电平（消抖用）
static uint8_t K3Deb = 0;			//消抖计数：连续3拍(30ms)电平不变才采信
static uint8_t K3State = 0;			//消抖后的稳定电平：1=PC14按下
static uint8_t K3Prev = 0;			//上一拍稳定电平（松开沿判定用）
static uint16_t K3Hold = 0;			//按住持续拍数：≥100拍(1秒)判长按
static uint16_t TuneShowTick = 0;	//按过调距键后第2行显示中点距离Mid的剩余拍数（300拍=3秒，减到0自动切回Ang/O显示）

/**
  * 函    数：调距键轮询（主循环每拍调用一次）
  * 参    数：Running 运行标志、TaskNum 当前任务号、SubRun 当前子任务号（main每拍传入，
  *           避免模块反向依赖main的变量）
  * 返 回 值：无
  * 注意事项：KEY3=PC14非阻塞轮询：3拍(30ms)消抖，松开沿触发；任务2且暂停态时
  *           短按=中点距离+1cm、长按≥1秒=减1cm（其他状态按了没反应，防误触）；
  *           松开沿必须在更新K3Hold之前判定，否则长按永远判不出
  */
void Tune_Poll(uint8_t Running, uint8_t TaskNum, uint8_t SubRun)
{
	uint8_t K3Raw = Key3_GetPress();	//1=PC14按下（不消抖不等松手）
	uint8_t K3Up;
	if (K3Raw == K3Old) { if (K3Deb < 3) K3Deb++; }
	else { K3Deb = 0; K3Old = K3Raw; }
	if (K3Deb >= 3) K3State = K3Raw;	//电平稳定30ms才采信
	K3Up = K3Prev & ~K3State;			//松开沿（上一拍按着、这一拍松开）
	K3Prev = K3State;
	if (K3Up && !Running && TaskNum == 2)	//任务2且暂停态才有效（运行中/别的任务按了没反应，防误触）
	{
		if (K3Hold >= 100) Task2_Tune(SubRun, -1);	//按住≥1秒再松手=减1cm
		else Task2_Tune(SubRun, 1);					//短按=加1cm
		TuneShowTick = 300;					//第2行显示Mid值3秒（期间继续按会重新计时）
	}
	if (K3State) { if (K3Hold < 60000) K3Hold++; }
	else K3Hold = 0;
	if (TuneShowTick > 0) TuneShowTick--;
}

/**
  * 函    数：调距显示是否激活
  * 参    数：无
  * 返 回 值：1=按过KEY3后3秒内，OLED第2行应显示Mid值（main据此填Disp.TuneMode）
  * 注意事项：显示内容由Display.c的TuneMode分支负责（Mid:+xxx cm）
  */
uint8_t Tune_ShowActive(void)
{
	return (TuneShowTick > 0) ? 1 : 0;
}
