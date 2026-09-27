#include "stm32f10x.h"                  // Device header
#include "Config.h"
#include "Control.h"

/**
  * 函    数：PI速度闭环
  * 参    数：Target 目标速度（编码器计数/拍）
  *           Actual 实测速度（编码器计数/拍）
  *           Integral 该轮积分值的指针
  *           MinOut/MaxOut 输出占空比范围（跟随/直行传0~100防倒车，倒车传-100~0防前冲）
  * 返 回 值：电机占空比，范围MinOut~MaxOut
  * 注意事项：输出 = KP×误差 + KI×积分÷16（积分弱化，防止积分快速饱和）；
  *           输出贴住限幅且误差方向只会贴得更深时积分不累加（防积分饱和），
  *           这样"转速冲过目标"时输出只会落到下限停转，不会反向抖动（车一前一后的根源）
  */
int16_t PI_Control(int16_t Target, int16_t Actual, int16_t *Integral, int16_t MinOut, int16_t MaxOut)
{
	int16_t Error = Target - Actual;			//速度误差
	int32_t Out = (int32_t)SPEED_KP * Error + ((int32_t)SPEED_KI * (*Integral) >> 4);	//计算输出（积分弱化÷16）
	if (Out > MaxOut) Out = MaxOut;				//输出限幅
	if (Out < MinOut) Out = MinOut;
	if (!((Out >= MaxOut && Error > 0) || (Out <= MinOut && Error < 0)))	//输出没贴死在限幅上时才累加积分（防饱和）
	{
		*Integral += Error;						//积分累加误差
		if (*Integral > INTEGRAL_LIMIT) *Integral = INTEGRAL_LIMIT;		//积分限幅
		if (*Integral < -INTEGRAL_LIMIT) *Integral = -INTEGRAL_LIMIT;
	}
	return (int16_t)Out;
}

/**
  * 函    数：转向寻线角速度PI闭环
  * 参    数：Target 目标角速度（陀螺LSB/拍，16.4LSB≈1°/s）
  *           Measured 实测角速度（单位同上，已按GYRO_DIR和启动时自动标定的零漂GyroZero修正过的MPU_GZ）
  *           Integral 积分值指针
  * 返 回 值：占空比，范围-TURN_DUTY_MAX~+TURN_DUTY_MAX
  * 注意事项：输出=(误差+积分)÷32：误差1LSB≈0.03%占空比，积分每拍累加误差
  *           （约半秒跟上负载，转弯要的是柔和不用猛积分）；
  *           输出贴死限幅且误差只会贴得更深时积分不累加（防饱和，同PI_Control）
  */
int16_t TurnRate_PI(int32_t Target, int32_t Measured, int32_t *Integral)
{
	int32_t Error = Target - Measured;					//角速度误差
	int32_t Out = (Error + (*Integral)) >> 5;			//÷32：软一点的PI增益，转弯不冲
	if (Out > TURN_DUTY_MAX) Out = TURN_DUTY_MAX;		//输出限幅
	if (Out < -TURN_DUTY_MAX) Out = -TURN_DUTY_MAX;
	if (!((Out >= TURN_DUTY_MAX && Error > 0) || (Out <= -TURN_DUTY_MAX && Error < 0)))	//防饱和：贴限幅且误差同向时不累加
	{
		*Integral += Error;								//积分累加误差
		if (*Integral > TURN_DUTY_MAX * 32) *Integral = TURN_DUTY_MAX * 32;		//积分限幅（积分最多贡献±TURN_DUTY_MAX）
		if (*Integral < -TURN_DUTY_MAX * 32) *Integral = -TURN_DUTY_MAX * 32;
	}
	return (int16_t)Out;
}
