#include "stm32f10x.h"                  // Device header
#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Gray.h"
#include "MaixCam.h"
#include "Display.h"
#include "Task1.h"
#include "Control.h"
#include "Goal.h"
#include "Task3.h"

/*==================== 模块内状态（main每拍调Task3_Run推进一步） ====================*/
static uint8_t T3State = 0;			//任务3状态机：0=等识别结果 1=启动找线 2=弧线慢转对准 3=巡线到目标角
static uint8_t MaixTarget = 0;		//任务3识别目标：1/2/3/4=角A/B/C/D
static uint16_t TurnTick = 0;		//找线/弧线慢转计时（拍）
static uint8_t T3LostLine = 0;		//弧线慢转期间中途丢过线标志（先转出去，再扫回线车头方向才算拐对）
static int16_t T3LeftIntegral = 0;	//弧线慢转左轮速度环积分（编码器闭环，进阶段2前清零）
static int16_t T3RightIntegral = 0;	//弧线慢转右轮速度环积分（编码器闭环，进阶段2前清零）

extern int32_t Odom;				//里程累计（main清零，本模块每拍累加）

/**
  * 函    数：任务3状态机初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：启动前必须清零状态机（否则上次运行的残留会让车"蹿"出去）；
  *           还要清一次 MaixCam 串口缓存——MaixCAM 的串口是系统日志口，开机日志里
  *           的"115200"会被解析模块读成"C角"之类的假目标，而 MaixNew 要等
  *           MaixCam_GetTarget() 才清、任务三之前没人调它，不清掉的话一按 PB1
  *           车会直接朝错误的角跑（详见 MaixCam.c 里 MaixCam_Flush 的说明）；
  *           清掉之后还能保证"cam 消息优先、没有才用 KEY4 按键目标"这条规则不被假消息破坏
  */
void Task3_Init(void)
{
	MaixCam_Flush();			//丢掉启动前积压的历史字节和已解析出的假目标
	T3State = 0;				//从等待识别结果开始
	MaixTarget = 0;				//识别目标清零
	TurnTick = 0;				//找线/弧线慢转计时清零
	T3LostLine = 0;				//丢线标志清零
	T3LeftIntegral = 0;			//弧线慢转速度环积分清零
	T3RightIntegral = 0;
}

/**
  * 函    数：弧线对准完成切巡线——按目标初始化任务1
  * 参    数：Target 目标角：1/2/3/4=角A/B/C/D
  * 返 回 值：无
  * 注意事项：拐弯计数从切巡线这一刻才开始算（弧线慢转不算拐弯）；A/B目标寻线左转
  *           （B角左转拐上AB）、C/D默认右转（C角右转拐CD）
  */
static void T3StartFollow(uint8_t Target)
{
	if (Target == 1) { Task1_Init(TASK3_CORNERS_A, 1); Task1_SetTurnDir(-1); }	//A角：B角左转拐上AB继续走到A角停
	else if (Target == 2) { Task1_Init(TASK3_CORNERS_B, 1); Task1_SetTurnDir(-1); }	//B角：到B角停
	else if (Target == 3) Task1_Init(TASK3_CORNERS_C, 1);	//C角：到C角停
	else Task1_Init(TASK3_CORNERS_D, 1);						//D角：C角右转拐上CD继续走到D角停
}

/**
  * 函    数：任务3——视觉识别巡线（仿任务1/2写法：启动找线→弧线慢转对准→任务1循迹）
  * 参    数：D 显示数据指针（每拍写入PWM供OLED显示，阶段3转给Task1_Run）
  * 返 回 值：1=任务完成（已停车，main统一调Task_Stop收尾） 0=继续
  * 注意事项：T3State 0=车在BC边中点车库等待目标（cam串口消息优先，没有就取KEY4按键
  *           选的目标）；T3State 1=启动找线：停下来一拍就判断灰度——00000000（离线）
  *           两轮同速慢慢直走前进找到线（一定要慢，防冲过头），压线立即进弧线慢转；
  *           T3State 2=弧线慢转对准（编码器双闭环、不用陀螺：两轮都正转、内轮慢外轮快，
 *           内轮闭环保证真的在慢慢转——开环占空比太小电机带不动）：
  *           车头骑在BC线上（全压线）但方向垂直，按目标方向沿弧线慢慢转（A/B左前朝B、
  *           C/D右前朝C），先转出去丢了线、再扫到线那一刻车头已对准沿线方向，立即切
  *           任务1循迹——不一定要转90度，扫到线就是巡线第一位；这个转不算拐弯次数
  *           （拐弯计数从切巡线才开始算）；
  *           T3State 3=复用任务1的循迹+角计数状态机巡线到目标角停（压线沿线走、到
  *           直角丢线按目标方向慢转找线把弯拐过去、数到目标拐点数停）；
  *           任务3全程不用陀螺积分判定，不受零漂影响；目标映射：1/I→A角、3/III→B角、
  *           5/V→C角、7/VII→D角
  */
uint8_t Task3_Run(DispInfo *D)
{
	if (T3State == 0)					//阶段0：等待目标（cam串口消息优先，收不到就用KEY4按键选的目标）
	{
		uint8_t Target = MaixCam_GetTarget();
		if (Target == 0) Target = Goal_Get();	//cam没消息：用按键选的目标（调试入口，比赛时cam优先）
		if (Target != 0)				//拿到目标：1/2/3/4=角A/B/C/D
		{
			MaixTarget = Target;
			Odom = 0;
			TurnTick = 0;
			T3State = 1;				//进入启动找线阶段
		}
		return 0;
	}

	if (T3State == 1)					//阶段1：启动找线（停下来一拍就判断00000000离线还是11111111压线）
	{
		uint8_t OnLine = Gray_GetState();	//8路灰度压线状态（0=全离线）
		if (OnLine != 0)				//压着线：直接进弧线慢转
		{
			TurnTick = 0;
			T3LostLine = 0;
			T3LeftIntegral = 0;			//速度环积分清零（防上次运行残留）
			T3RightIntegral = 0;
			T3State = 2;				//进入弧线慢转对准阶段
		}
		else							//离线差1~2cm：两轮同速慢慢直走前进找线（一定要慢，1~2cm冲过头就乱拐）
		{
			TurnTick++;
			Motor_SetSpeed(TASK3_FIND_DUTY);
			MotorB_SetSpeed(TASK3_FIND_DUTY);
			D->PwmL = TASK3_FIND_DUTY;
			D->PwmR = TASK3_FIND_DUTY;
			if (TurnTick >= TASK3_FIND_TIMEOUT_TICKS)	//找线超过10秒还没压到线：强制停车兜底
			{
				return 1;
			}
		}
		return 0;
	}

	if (T3State == 2)					//阶段2：弧线慢转对准（编码器双闭环：两轮都正转、内轮慢外轮快；不转满90度，扫到线立即切巡线；这个转不算拐弯）
	{
		uint8_t OnLine;
		int8_t LeftOut;
		int8_t RightOut;
		int16_t LeftTarget;
		int16_t RightTarget;
		TurnTick++;
		if (MaixTarget == 1 || MaixTarget == 2)	//A/B目标：左前弧线（左轮=内轮慢、右轮=外轮快，车头边前进边朝B转）
		{
			LeftTarget = TASK3_TURN_IN_SPEED + TRIM_LEFT;
			RightTarget = TASK3_TURN_OUT_SPEED + TRIM_RIGHT;
		}
		else										//C/D目标：右前弧线（左轮=外轮快、右轮=内轮慢，车头边前进边朝C转）
		{
			LeftTarget = TASK3_TURN_OUT_SPEED + TRIM_LEFT;
			RightTarget = TASK3_TURN_IN_SPEED + TRIM_RIGHT;
		}
		LeftOut = (int8_t)PI_Control(LeftTarget, Encoder_GetLeft(), &T3LeftIntegral, 0, OUTPUT_LIMIT);
		RightOut = (int8_t)PI_Control(RightTarget, Encoder_GetRight(), &T3RightIntegral, 0, OUTPUT_LIMIT);
		Motor_SetSpeed(LeftOut);
		MotorB_SetSpeed(RightOut);
		D->PwmL = LeftOut;
		D->PwmR = RightOut;
		OnLine = Gray_GetState();
		if (OnLine == 0) T3LostLine = 1;	//先转出去丢了线，再扫回线车头才算对准沿线方向
		if (T3LostLine && OnLine != 0)		//扫到线那一刻：车头已对准，立即切巡线（不一定要转90度，巡线是第一位）
		{
			Motor_Stop();
			Odom = 0;
			T3StartFollow(MaixTarget);	//按目标初始化任务1巡线（拐弯计数从这开始，弧线慢转不算）
			T3State = 3;				//进入巡线阶段
			Encoder_Update();
		}
		else if (TurnTick >= TASK3_TURN_TIMEOUT_TICKS)	//弧线慢转超过20秒还没扫回线：强制停车兜底
		{
			return 1;
		}
		return 0;
	}

	return Task1_Run(D);				//阶段3：复用任务1的循迹+角计数状态机巡线到目标角停
}
