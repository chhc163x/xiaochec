#include "stm32f10x.h"                  // Device header
#include "Key.h"
#include "Goal.h"

/*==================== 目标选择模块（KEY4=PC15） ====================*/
/*任务3暂停态按KEY4选目标角：按1下=A、2下=B、3下=C、4下=D（再按回A循环），
  按的次数实时生效并常显在OLED第2行（Goal:A~D）；cam串口消息就绪后优先于按键目标。
  状态static自持（模块化：main只每拍调一次Goal_Poll并读Goal_Get），
  Goal跨启动保留（重跑同一目标不用重选，断电恢复0）。*/

static uint8_t K4Old = 0;			//上一拍原始电平（消抖用）
static uint8_t K4Deb = 0;			//消抖计数：连续3拍(30ms)电平不变才采信
static uint8_t K4State = 0;			//消抖后的稳定电平：1=PC15按下
static uint8_t K4Prev = 0;			//上一拍稳定电平（松开沿判定用）
static uint8_t Goal = 0;			//按键选的目标：1/2/3/4=角A/B/C/D，0=还没选过

/**
  * 函    数：目标选择键轮询（主循环每拍调用一次）
  * 参    数：Running 运行标志、TaskNum 当前任务号（main每拍传入，
  *           避免模块反向依赖main的变量）
  * 返 回 值：无
  * 注意事项：KEY4=PC15非阻塞轮询：3拍(30ms)消抖，松开沿触发；
  *           任务3且暂停态时每按一下Goal+1（>4回绕到1），按几下代表几（1~4=A~D）；
  *           其他任务/运行中按了没反应（防误触）
  */
void Goal_Poll(uint8_t Running, uint8_t TaskNum)
{
	uint8_t K4Raw = Key4_GetPress();	//1=PC15按下（不消抖不等松手）
	uint8_t K4Up;
	if (K4Raw == K4Old) { if (K4Deb < 3) K4Deb++; }
	else { K4Deb = 0; K4Old = K4Raw; }
	if (K4Deb >= 3) K4State = K4Raw;	//电平稳定30ms才采信
	K4Up = K4Prev & ~K4State;			//松开沿（上一拍按着、这一拍松开）
	K4Prev = K4State;
	if (K4Up && !Running && TaskNum == 3)	//任务3且暂停态才有效（运行中/别的任务按了没反应，防误触）
	{
		Goal++;
		if (Goal > 4) Goal = 1;			//按第5下回到A（1~4循环）
	}
}

/**
  * 函    数：获取按键选的目标
  * 参    数：无
  * 返 回 值：1/2/3/4=角A/B/C/D，0=还没选过
  * 注意事项：任务3阶段0在收不到cam消息时用它作为目标（cam优先）；
  *           Goal跨启动保留，重跑同一目标不用重选，断电恢复0
  */
uint8_t Goal_Get(void)
{
	return Goal;
}
