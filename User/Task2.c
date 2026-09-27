#include "stm32f10x.h"                  // Device header
#include "Config.h"
#include "Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "Display.h"
#include "Task1.h"
#include "Task2.h"

/*==================== 任务2总入口（薄壳分发：按子任务号转给各自独立的状态机） ====================*/
static uint8_t Sub = 1;				//当前子任务号（启动时由main传入：1=倒车入库 2=侧方停车）

extern int32_t GyroSumZ;			//陀螺Z轴积分角度（main每拍积分）
extern int32_t Odom;				//里程累计（main清零，本模块每拍累加）

static uint8_t Task2_1_Run(DispInfo *D);	//原型声明：Task2_Run分发在定义之前调用
static uint8_t Task2_2_Run(DispInfo *D);

/*==================== 2-1 倒车入库状态机（V2场地实测通过：方向/角度/中点/库深都对） ====================*/
static uint8_t T21State = 0;		//2-1状态机：0=循迹到B角+寻线转弯 1=沿BC走50cm到车库旁 2=原地左转90° 3=倒车入库
static uint16_t T21TurnTick = 0;	//原地转计时（拍）
static int16_t T21LeftIntegral = 0;	//左轮PI积分
static int16_t T21RightIntegral = 0;	//右轮PI积分

/*==================== 2-2 侧方停车状态机（参数待标定，随便改不影响上面已验证的2-1） ====================*/
static uint8_t T22State = 0;		//2-2状态机：0=借道循迹绕场3个直角到D角 1=沿DA走50cm到中点停 2=右后倒车（车尾向右甩进库） 3=左修摆正停车
static uint16_t T22TurnTick = 0;	//右后倒车/左修计时（拍）
static int16_t T22LeftIntegral = 0;	//左轮PI积分
static int16_t T22RightIntegral = 0;	//右轮PI积分

/*==================== 中点距离（运行时可调：KEY3调距键短按+1cm、长按≥1s−1cm，任务2暂停态有效） ====================*/
static int16_t MidDist1 = TASK2_BC_DIST_CM;	//2-1：转弯后走到车库旁的距离cm（默认Config值；调距只改内存，确定后写死Config重烧生效）
static int16_t MidDist2 = TASK2_DA_DIST_CM;	//2-2：转弯后走到中点的距离cm（两子任务独立，互不影响）

/**
  * 函    数：任务2状态机初始化
  * 参    数：SubRun 子任务号：1=倒车入库 2=侧方停车
  * 返 回 值：无
  * 注意事项：启动前必须清零积分和状态机（否则上次运行的残留会让车"蹿"出去）；
  *           两个子任务的状态机完全独立（各自static变量），调2-2不影响2-1；
  *           中点距离MidDist1/2不在此重置——调距模式调过的值跨启动保留，重烧才恢复Config默认
  */
void Task2_Init(uint8_t SubRun)
{
	Sub = SubRun;				//记住子任务号（Task2_Run按它分发）
	if (SubRun == 1)			//2-1：借道任务1循迹到B角（1个直角，到角不停车）
	{
		T21State = 0;
		T21TurnTick = 0;
		T21LeftIntegral = 0;
		T21RightIntegral = 0;
		Task1_Init(TASK2_SUB1_CORNERS, 0);
	}
	else						//2-2：借道任务1沿DA边（无直角）
	{
		T22State = 0;
		T22TurnTick = 0;
		T22LeftIntegral = 0;
		T22RightIntegral = 0;
		Task1_Init(TASK2_SUB2_CORNERS, 0);
	}
}

/**
  * 函    数：任务2每拍推进一步（薄壳：只按子任务号分发）
  * 参    数：D 显示数据指针（每拍写入PWM供OLED显示，阶段0/1转给Task1_Run）
  * 返 回 值：1=任务完成（已停车，main统一调Task_Stop收尾） 0=继续
  */
uint8_t Task2_Run(DispInfo *D)
{
	if (Sub == 1) return Task2_1_Run(D);	//2-1：倒车入库
	else return Task2_2_Run(D);				//2-2：侧方停车
}

/**
  * 函    数：子任务2-1——倒车入库（四阶段状态机，V2场地实测通过）
  * 参    数：D 显示数据指针（每拍写入PWM供OLED显示，阶段0/1转给Task1_Run）
  * 返 回 值：1=任务完成（已停车，main统一调Task_Stop收尾） 0=继续
  * 注意事项：T21State 0=借道任务1循迹到B角+寻线转弯（A点发车沿AB）；
  *           T21State 1=转弯完成瞬间起计里程，沿BC边循迹走50cm到车库旁
  *           （1m×1m场地B到中点=50cm——不记总距离，转弯后走多少算多少）；
  *           T21State 2=MPU6050陀螺积分原地左转90°（左-右+，车尾甩向右侧车库，
  *           提前TASK2_TURN_STOP_LEAD_DEG°判定，惯性滑完正好90°）；
  *           T21State 3=倒车入库（里程闭环，停稳后返回1由main收尾）；
  *           ⚠ 本函数已实测通过：调2-2时不要改这里
  */
static uint8_t Task2_1_Run(DispInfo *D)
{
	int16_t Target;
	int8_t LeftOut, RightOut;
	int32_t DistTarget;

	if (T21State == 0)					//阶段0：循迹到B角+寻线转弯（借道任务1）
	{
		if (Task1_Run(D)) return 1;		//任务1超时兜底→整体停车
		if (Task1_GetCorners() >= TASK2_SUB1_CORNERS && Task1_GetState() == 0)	//B角已判且转弯完成恢复跟随
		{
			Odom = 0;					//从转弯完成瞬间起计里程
			T21State = 1;				//沿BC走到车库旁
		}
		return 0;
	}

	if (T21State == 1)					//阶段1：沿BC边循迹走到车库旁（里程闭环判定）
	{
		if (Task1_Run(D)) return 1;		//任务1超时兜底→整体停车
		Odom += ((int32_t)Encoder_GetLeft() + Encoder_GetRight()) / 2;	//累计里程（两轮均值）
		DistTarget = (int32_t)MidDist1 * ENC_COUNTS_PER_CM;	//调距模式可改的距离（默认TASK2_BC_DIST_CM）
		if (Odom >= DistTarget)			//走到目标距离（到车库旁）
		{
			Motor_Stop();
			GyroSumZ = 0;				//原地转前清陀螺积分
			T21TurnTick = 0;
			T21State = 2;				//进入原地转阶段
		}
		return 0;
	}

	if (T21State == 2)					//阶段2：原地左转90°（左-右+，车尾甩向车库）
	{
		uint8_t TurnDone = 0;
		int32_t TurnAbs;				//已转角度绝对值（不依赖GYRO_DIR符号，转够角度必停）
		T21TurnTick++;
		Motor_SetSpeed(-TASK2_TURN_DUTY);	//左轮反转
		MotorB_SetSpeed(TASK2_TURN_DUTY);	//右轮正转→车头向左转，车尾甩向车库
		D->PwmL = -TASK2_TURN_DUTY;
		D->PwmR = TASK2_TURN_DUTY;
		TurnAbs = (GyroSumZ >= 0) ? GyroSumZ : -GyroSumZ;
		if (TurnAbs >= (int32_t)(TASK2_TURN_ANGLE_DEG - TASK2_TURN_STOP_LEAD_DEG) * 1640) TurnDone = 1;	//提前判定，惯性滑完正好90°（1°=1640 LSB）

		if (TurnDone)					//转到位：进入倒车入库阶段
		{
			Motor_Stop();
			GyroSumZ = 0;
			Odom = 0;
			T21TurnTick = 0;
			T21LeftIntegral = 0;
			T21RightIntegral = 0;
			T21State = 3;
			Encoder_Update();			//同步编码器防虚增量
		}
		else if (T21TurnTick >= TASK2_TURN_TIMEOUT_TICKS)	//15秒没转到位
		{
			return 1;					//兜底停车
		}
		return 0;
	}

	/*阶段3：倒车入库（里程闭环）*/
	Target = -TASK2_DRIVE_DUTY;			//倒车目标速度为负
	Odom += ((int32_t)Encoder_GetLeft() + Encoder_GetRight()) / 2;	//倒车时增量为负，里程减小
	LeftOut = (int8_t)PI_Control(Target + TRIM_LEFT, Encoder_GetLeft(), &T21LeftIntegral, -OUTPUT_LIMIT, 0);		//倒车不前冲
	RightOut = (int8_t)PI_Control(Target + TRIM_RIGHT, Encoder_GetRight(), &T21RightIntegral, -OUTPUT_LIMIT, 0);
	Motor_SetSpeed(LeftOut);
	MotorB_SetSpeed(RightOut);
	D->PwmL = LeftOut;
	D->PwmR = RightOut;

	DistTarget = (int32_t)TASK2_REVERSE_CM * ENC_COUNTS_PER_CM;
	if (Odom <= -DistTarget)			//倒够距离，入库完成
	{
		Motor_Stop();
		return 1;						//停车完成：main统一收尾，子任务保持不动，按PC13切下一个、PB1重跑当前
	}
	return 0;
}

/**
  * 函    数：子任务2-2——侧方停车（四阶段状态机，参数待标定）
  * 参    数：D 显示数据指针（每拍写入PWM供OLED显示，阶段0/1转给Task1_Run）
  * 返 回 值：1=任务完成（已停车，main统一调Task_Stop收尾） 0=继续
  * 注意事项：T22State 0=借道任务1绕场3个直角到D角（A→B→C→D，到角不停车）；
  *           T22State 1=D角转弯完成起计里程，沿DA边循迹走50cm到中点停下（和2-1同款）；
  *           T22State 2=右后倒车（左轮快倒右轮慢倒，车尾向右甩进库，转到
  *           TASK2_DA_BACK_TURN_DEG判定，提前TASK2_DA_TURN_STOP_LEAD_DEG°）；
  *           T22State 3=左修（继续倒车，左轮慢倒右轮快倒，车头向右摆回，
  *           陀螺角回到±TASK2_DA_BACK_STRAIGHT_DEG°内=车身正了）停车；
  *           ⚠ 本函数随便改，不影响已通过的2-1
  */
static uint8_t Task2_2_Run(DispInfo *D)
{
	int8_t LeftOut, RightOut;
	int32_t DistTarget;

	if (T22State == 0)					//阶段0：借道任务1循迹绕场3个直角到D角+寻线转弯（和2-1同款逻辑）
	{
		if (Task1_Run(D)) return 1;		//任务1超时兜底→整体停车
		if (Task1_GetCorners() >= TASK2_SUB2_CORNERS && Task1_GetState() == 0)	//3个直角已判且转弯完成恢复跟随
		{
			Odom = 0;					//从D角转弯完成瞬间起计里程
			T22State = 1;				//沿DA走到中点
		}
		return 0;
	}

	if (T22State == 1)					//阶段1：沿DA边循迹走到中点（里程闭环判定）
	{
		if (Task1_Run(D)) return 1;		//任务1超时兜底→整体停车
		Odom += ((int32_t)Encoder_GetLeft() + Encoder_GetRight()) / 2;	//累计里程（两轮均值）
		DistTarget = (int32_t)MidDist2 * ENC_COUNTS_PER_CM;	//调距模式可改的距离（默认TASK2_DA_DIST_CM）
		if (Odom >= DistTarget)			//走到目标距离（到中点）
		{
			Motor_Stop();
			GyroSumZ = 0;				//右后倒车前清陀螺积分
			T22TurnTick = 0;
			T22State = 2;				//进入右后倒车阶段
		}
		return 0;
	}

	if (T22State == 2)					//阶段2：右后倒车（左轮快倒右轮慢倒，车尾向右甩进库）
	{
		uint8_t TurnDone = 0;
		int32_t TurnAbs;				//已甩角度绝对值（不依赖GYRO_DIR符号，甩够角度必停）
		T22TurnTick++;
		Odom += ((int32_t)Encoder_GetLeft() + Encoder_GetRight()) / 2;	//倒车时增量为负，里程减小
		LeftOut = (int8_t)PI_Control(-TASK2_DA_BACK_FAST + TRIM_LEFT, Encoder_GetLeft(), &T22LeftIntegral, -OUTPUT_LIMIT, 0);	//左轮快倒
		RightOut = (int8_t)PI_Control(-TASK2_DA_BACK_SLOW + TRIM_RIGHT, Encoder_GetRight(), &T22RightIntegral, -OUTPUT_LIMIT, 0);	//右轮慢倒→车尾向右甩向车库
		Motor_SetSpeed(LeftOut);
		MotorB_SetSpeed(RightOut);
		D->PwmL = LeftOut;
		D->PwmR = RightOut;
		TurnAbs = (GyroSumZ >= 0) ? GyroSumZ : -GyroSumZ;
		if (TurnAbs >= (int32_t)(TASK2_DA_BACK_TURN_DEG - TASK2_DA_TURN_STOP_LEAD_DEG) * 1640) TurnDone = 1;	//提前判定，惯性滑到位

		if (TurnDone)					//车尾甩到位：进入左修阶段（GyroSumZ不清零，阶段3靠它回到0判定摆正）
		{
			Motor_Stop();
			T22TurnTick = 0;
			T22LeftIntegral = 0;
			T22RightIntegral = 0;
			T22State = 3;
			Encoder_Update();			//同步编码器防虚增量
		}
		else if (T22TurnTick >= TASK2_TURN_TIMEOUT_TICKS)	//15秒没甩到位
		{
			return 1;					//兜底停车
		}
		return 0;
	}

	if (T22State == 3)					//阶段3：左修（继续倒车，左轮慢倒右轮快倒，车头向右摆回车身摆正）
	{
		int32_t TurnAbs;				//当前偏角绝对值（从甩的角度一路回到0=正了）
		T22TurnTick++;
		Odom += ((int32_t)Encoder_GetLeft() + Encoder_GetRight()) / 2;	//倒车里程继续累计
		LeftOut = (int8_t)PI_Control(-TASK2_DA_BACK_SLOW + TRIM_LEFT, Encoder_GetLeft(), &T22LeftIntegral, -OUTPUT_LIMIT, 0);	//左轮慢倒
		RightOut = (int8_t)PI_Control(-TASK2_DA_BACK_FAST + TRIM_RIGHT, Encoder_GetRight(), &T22RightIntegral, -OUTPUT_LIMIT, 0);	//右轮快倒→车头向右摆回
		Motor_SetSpeed(LeftOut);
		MotorB_SetSpeed(RightOut);
		D->PwmL = LeftOut;
		D->PwmR = RightOut;
		TurnAbs = (GyroSumZ >= 0) ? GyroSumZ : -GyroSumZ;
		if (TurnAbs <= (int32_t)TASK2_DA_BACK_STRAIGHT_DEG * 1640)	//偏角回到0附近=车身正了
		{
			Motor_Stop();
			return 1;					//停车完成：main统一收尾
		}
		else if (T22TurnTick >= TASK2_TURN_TIMEOUT_TICKS)	//15秒没摆正
		{
			return 1;					//兜底停车
		}
		return 0;
	}

	return 1;							//状态机不在0~3（不可能发生）：兜底停车
}

/**
  * 函    数：调距加/减指定子任务的中点距离
  * 参    数：SubRun 子任务号（1=倒车入库 2=侧方停车），Delta 增量cm（+1或-1）
  * 返 回 值：无
  * 注意事项：只改内存、钳位1~99cm；调好的确定值由人写进Config.h
  *           （TASK2_BC_DIST_CM/TASK2_DA_DIST_CM），重新编译烧录生效；
  *           SubRun必须由main传入——本文件的内部Sub只在Task_Start时同步，
  *           暂停态用PC13切子任务后内部Sub还是旧的，用它判断会调错变量
  */
void Task2_Tune(uint8_t SubRun, int8_t Delta)
{
	if (SubRun == 1)
	{
		MidDist1 += Delta;
		if (MidDist1 < 1) MidDist1 = 1;
		if (MidDist1 > 99) MidDist1 = 99;
	}
	else
	{
		MidDist2 += Delta;
		if (MidDist2 < 1) MidDist2 = 1;
		if (MidDist2 > 99) MidDist2 = 99;
	}
}

/**
  * 函    数：读取指定子任务的中点距离
  * 参    数：SubRun 子任务号（1=倒车入库 2=侧方停车）
  * 返 回 值：中点距离（cm），OLED调距显示/串口上报用
  */
int16_t Task2_GetMidDist(uint8_t SubRun)
{
	return (SubRun == 1) ? MidDist1 : MidDist2;
}
