#include "stm32f10x.h"                  // Device header
#include "Config.h"
#include "Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "Gray.h"
#include "Display.h"
#include "Task1.h"

/*==================== 模块内状态（main每拍调Task1_Run推进一步；任务3最后阶段复用） ====================*/
static uint8_t FollowState = 0;		//循迹状态机：0=循迹跟随 2=转向寻线（原地慢转找线）
static uint8_t NoLineTick = 0;		//跟随中连续全丢线的拍数（判定经过直角）
static uint16_t TurnTick = 0;		//转向寻线计时（拍）
static int32_t TurnIntegral = 0;	//转向寻线角速度PI积分（原地慢转转速闭环用）
static uint8_t FollowTick = 0;		//恢复跟随后的冷却拍数：期间丢线重新转向寻线，不误判新直角（防连判）
static uint8_t StartGraceTick = 0;	//起步宽限期剩余拍数（起点压角A，防假触发）
static uint8_t CornerCount = 0;		//已经过的直角个数
static uint8_t TargetCorner = 0;	//经过该直角个数后自动停车
static int8_t LastError = 0;		//最近一次循迹误差，丢线瞬间沿用
static int16_t LeftIntegral = 0;	//左轮PI积分
static int16_t RightIntegral = 0;	//右轮PI积分
static uint8_t AutoStop = 1;		//到达目标直角后是否自动停车：任务2借道循迹时=0（转弯后继续走里程）
static int8_t TurnDir = 1;			//转向寻线方向：1=原地右转（默认，任务1/2、任务3的C/D目标）、-1=原地左转（任务3的A/B目标在B角拐弯用）

extern uint8_t MPU_OnLine;			//MPU在线标志（main开机自检判定）
extern int16_t MPU_GZ;				//MPU6050角速度Z轴原始值（main每拍读取）
extern int32_t GyroZero;			//启动时自动标定的陀螺零漂（main在Task_Start采样）

/**
  * 函    数：任务1状态机初始化
  * 参    数：Corners 目标直角个数（经过该个数后自动停车）
  *           AutoStop 1=到目标角自动停车返回完成；0=不停车继续寻线转弯（任务2借道）
  * 返 回 值：无
  * 注意事项：启动前必须清零积分和各类状态机（否则上次运行的残留会让车"蹿"出去），
  *           并装填起步宽限期（起点压角A，防止起步瞬间判成经过直角）
  */
void Task1_Init(uint8_t Corners, uint8_t AutoStopFlag)
{
	FollowState = 0;			//循迹从跟随阶段开始
	NoLineTick = 0;				//丢线判定清零
	TurnTick = 0;				//转向寻线计时清零
	TurnIntegral = 0;			//转向寻线角速度PI积分清零
	FollowTick = 0;				//转弯冷却清零
	StartGraceTick = START_DEBOUNCE_TICKS;	//装填起步宽限期（起点压角A）
	CornerCount = 0;			//直角计数清零
	LastError = 0;				//循迹误差清零
	LeftIntegral = 0;			//积分清零
	RightIntegral = 0;
	TargetCorner = Corners;		//设置目标直角数
	AutoStop = AutoStopFlag;	//保存自动停车开关
	TurnDir = 1;				//转向寻线方向重置为默认右转（任务3的A/B目标在Init后再调SetTurnDir改左转）
}

/**
  * 函    数：设置寻线转向方向
  * 参    数：Dir 1=原地右转（默认）、-1=原地左转
  * 返 回 值：无
  * 注意事项：必须在Task1_Init之后调用（Init会把方向重置为默认右转）；
  *           任务3的A/B目标在B角要左转拐上AB边，右转寻线永远扫不到左侧的线
  */
void Task1_SetTurnDir(int8_t Dir)
{
	TurnDir = (Dir < 0) ? -1 : 1;	//只认符号，防乱传
}

/**
  * 函    数：查询已过直角个数（任务2借道循迹时用）
  * 参    数：无
  * 返 回 值：已过直角个数
  */
uint8_t Task1_GetCorners(void)
{
	return CornerCount;
}

/**
  * 函    数：查询循迹状态机（任务2借道循迹时用）
  * 参    数：无
  * 返 回 值：0=跟随 2=转向寻线
  */
uint8_t Task1_GetState(void)
{
	return FollowState;
}

/**
  * 函    数：任务1——巡线（循迹+直角计数状态机，任务3最后阶段也复用）
  * 参    数：D 显示数据指针（每拍写入角计数/循迹误差/PWM供OLED显示）
  * 返 回 值：1=任务完成（已停车，main统一调Task_Stop收尾） 0=继续
  * 注意事项：FollowState=0"跟随"：8路灰度循迹走线，连续CORNER_DEBOUNCE拍
  *           全丢线=经过一个直角，CornerCount++；到达TargetCorner个直角后
  *           返回1由main停车，子任务保持不动；否则进入转向寻线阶段；转弯后
  *           冷却期内丢线会重新转向寻线，不算新直角（防连判）；
  *           FollowState=2"转向寻线"：左轮正转右轮反转原地慢慢右转（陀螺角速度
  *           PI闭环，约10°/s，转多慢都没事；陀螺离线退化为开环TURN_DUTY），
  *           任意一路扫到线立即恢复循迹——线全程不离探头，顺着线把弯转过去，
  *           不存在转早转晚；超TURN_TIMEOUT_TICKS还没压到线返回1强制停车兜底
  */
uint8_t Task1_Run(DispInfo *D)
{
	D->CornerCount = CornerCount;	//每拍更新显示
	D->TargetCorner = TargetCorner;
	D->LastError = LastError;

	if (FollowState == 0)					//阶段0：循迹跟随
	{
		uint8_t Active = Gray_GetActiveCount();
		int8_t Error;
		if (Active >= 1)					//正常压线
		{
			Error = Gray_GetError();
			NoLineTick = 0;
			if (FollowTick > 0) FollowTick--;	//压线才递减冷却（丢线会重新转向寻线，冷却重置）
		}
		else								//完全丢线
		{
			if (FollowTick > 0)				//刚转完弯的冷却期内：重新转向寻线（线还在车头外侧，继续慢转压上去）
			{
				FollowState = 2;
				TurnTick = 0;
				TurnIntegral = 0;
				Motor_Stop();
				return 0;
			}
			else if (StartGraceTick > 0)	//起步宽限期内（起点压角A）：不算丢线
			{
				StartGraceTick--;
				Error = 0;
				NoLineTick = 0;
			}
			else
			{
				NoLineTick++;
				if (NoLineTick >= CORNER_DEBOUNCE)	//连续几拍全丢线=经过一个直角
				{
					CornerCount++;
					if (CornerCount >= TargetCorner && AutoStop)	//已到达目标直角且允许自动停
					{
						Motor_Stop();			//在丢线瞬间停车（车头压角点）
						return 1;				//停在目标角：main统一收尾，子任务保持不动，按PC13切下一个、PB1重跑当前
					}
					FollowState = 2;			//进入转向寻线：原地慢慢右转，压到线即恢复循迹
					TurnTick = 0;
					TurnIntegral = 0;
					Motor_Stop();				//先停稳，下一拍开始慢转找线
					return 0;
				}
				Error = LastError;				//丢线几拍内先按原误差微调
			}
		}
		LastError = Error;

		int16_t LeftTarget = LINE_BASE + LINE_KP * Error + TRIM_LEFT;	//左轮目标速度
		int16_t RightTarget = LINE_BASE - LINE_KP * Error + TRIM_RIGHT;	//右轮目标速度
		if (LeftTarget > TARGET_LIMIT) LeftTarget = TARGET_LIMIT;			//目标限幅
		if (LeftTarget < FOLLOW_MIN_TARGET) LeftTarget = FOLLOW_MIN_TARGET;	//下限钳位：跟随中不倒车
		if (RightTarget > TARGET_LIMIT) RightTarget = TARGET_LIMIT;
		if (RightTarget < FOLLOW_MIN_TARGET) RightTarget = FOLLOW_MIN_TARGET;

		int8_t LeftOut = (int8_t)PI_Control(LeftTarget, Encoder_GetLeft(), &LeftIntegral, 0, OUTPUT_LIMIT);		//左轮PI（下限0：冲过目标只停转不倒车）
		int8_t RightOut = (int8_t)PI_Control(RightTarget, Encoder_GetRight(), &RightIntegral, 0, OUTPUT_LIMIT);	//右轮PI
		Motor_SetSpeed(LeftOut);
		MotorB_SetSpeed(RightOut);
		D->PwmL = LeftOut;					//记录PWM输出，OLED显示用
		D->PwmR = RightOut;
	}
	else									//阶段2：转向寻线（原地慢慢右转，任意一路压到线即恢复循迹，线全程不离探头）
	{
		uint8_t State;
		int32_t TargetRate;
		TurnTick++;
		State = Gray_GetState();
		if (TurnTick < TURN_RAMP_TICKS)					//起步软启动：角速度从0线性升到寻线速度（方向由TurnDir定）
		{
			TargetRate = TURN_RATE_CRUISE * TurnDir * TurnTick / TURN_RAMP_TICKS;
		}
		else
		{
			TargetRate = TURN_RATE_CRUISE * TurnDir;
		}

		if (MPU_OnLine)									//陀螺在线：角速度PI闭环，稳稳的慢转
		{
			int8_t Out = (int8_t)TurnRate_PI(TargetRate, (int32_t)(MPU_GZ - GyroZero) * GYRO_DIR, &TurnIntegral);
			Motor_SetSpeed(Out);
			MotorB_SetSpeed(-Out);
			D->PwmL = Out;								//OLED显示
			D->PwmR = -Out;
		}
		else											//陀螺离线兜底：开环固定占空比（TurnDir=1原地右转、-1原地左转）
		{
			Motor_SetSpeed((int8_t)(TURN_DUTY * TurnDir));
			MotorB_SetSpeed((int8_t)(-TURN_DUTY * TurnDir));
			D->PwmL = (int8_t)(TURN_DUTY * TurnDir);	//OLED显示
			D->PwmR = (int8_t)(-TURN_DUTY * TurnDir);
		}

		if (TurnTick > TURN_BLIND_TICKS && State != 0)	//任意一路压到线：立即恢复循迹，顺着线把弯转过去
		{
			FollowState = 0;				//回到跟随阶段
			FollowTick = TURN_COOLDOWN_TICKS;	//装填冷却：期间丢线重新转向寻线，不误判新直角
			NoLineTick = 0;
			LastError = 0;
			LeftIntegral = 0;
			RightIntegral = 0;
			Motor_Stop();
			Encoder_Update();				//同步编码器，防止下一拍出现虚增量
			return 0;
		}

		if (TurnTick >= TURN_TIMEOUT_TICKS)	//转了15秒还没压到线
		{
			return 1;						//强制停车兜底
		}
	}
	return 0;
}
