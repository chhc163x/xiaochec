


#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "Motor.h"
#include "Encoder.h"
#include "Gray.h"
#include "MaixCam.h"
#include "Key.h"
#include "Config.h"
#include "Display.h"
#include "Task1.h"
#include "Task2.h"
#include "Task3.h"
#include "Tune.h"
#include "Goal.h"

/*==================== 可调参数 ====================*/
#define TICK_MS			10		//控制周期10ms，主循环每次执行一小步（其余所有调参都在Config.h里）

/*DWT周期计数器寄存器（本工程core_cm3.h精简版没有DWT定义，直接按地址访问）：
  计数器以72MHz自增，前后两次读数的差÷72=经过的微秒数，主循环用它补齐节拍*/
#define DWT_CTRL		(*(volatile uint32_t *)0xE0001000)	//DWT控制寄存器
#define DWT_CYCCNT		(*(volatile uint32_t *)0xE0001004)	//DWT周期计数寄存器

/*==================== 全局变量（任务模块按需extern引用） ====================*/
uint8_t TaskNum = 1;			//当前任务号：1=巡线 2=停车入库 3=视觉识别巡线
uint8_t SubRun = 1;				//当前子任务号：任务1/2各有1/2两个子任务，跑完后保持不动，按PC13才切下一个（方便调试）
uint8_t Running = 0;			//运行标志：0=暂停 1=运行中
int32_t GyroSumZ = 0;			//陀螺Z轴积分角度（LSB累计，1°=1640@±2000dps×10ms）
int32_t Odom = 0;				//里程累计（两轮编码器增量均值，编码器计数）
uint8_t ID = 0;					//MPU6050的WHO_AM_I（0x68），开机自检画面显示用
uint8_t MPU_OnLine = 0;			//MPU在线标志：开机读ID=0x68/0x74才算在线；离线时跳过每拍读取，防止I2C等超时拖慢节拍
int16_t MPU_AZ = 0;				//MPU6050加速度Z轴原始值
int16_t MPU_GZ = 0;				//MPU6050角速度Z轴原始值
int32_t GyroZero = 0;			//陀螺Z轴零漂（启动时自动标定：按PB1前车静止在起点，采样0.5秒取平均）
DispInfo Disp = {0};			//OLED显示数据：main每拍填系统状态，任务函数每拍填任务状态

void MPU_ReadTick(void);		//原型声明：任务启动里零漂采样要在定义之前调用

/**
  * 函    数：陀螺零漂自动标定
  * 参    数：无
  * 返 回 值：无
  * 注意事项：按PB1启动前车本来就停在起点，采样0.5秒(50拍)取平均。不标零漂的话Ang
  *           会自己慢慢涨，任务2原地转90°的判定会被带偏（转早或转不到）。按下按键
  *           的瞬间车会被按得晃动：必须等车静止再采样——连续50拍每拍与上拍差
  *           <20LSB(≈1.2°/s)才算静止，车在动就清零重来，最多等2秒兜底
  */
static void MPU_CalibrateZero(void)
{
	if (MPU_OnLine)
	{
		int32_t Sum = 0;
		uint8_t Stable = 0;
		uint16_t Wait = 0;
		int16_t Prev;
		MPU_ReadTick();
		Prev = MPU_GZ;
		while (Stable < 50 && Wait < 200)
		{
			Delay_ms(10);
			MPU_ReadTick();
			Wait++;
			{
				int16_t Diff = MPU_GZ - Prev;
				if (Diff < 0) Diff = -Diff;
				if (Diff < 20)			//本拍读数稳定：计入均值
				{
					Sum += MPU_GZ;
					Stable++;
				}
				else					//车在动（按键晃动/还没放稳）：清零重来
				{
					Sum = 0;
					Stable = 0;
				}
				Prev = MPU_GZ;
			}
		}
		GyroZero = (Stable > 0) ? Sum / Stable : 0;	//50拍均值；一直没静下来就用已采到的稳定段
	}
	else
	{
		GyroZero = 0;			//MPU离线：兜底0（任务2转判定不准，任务1寻线转弯开环兜底）
	}
}

/**
  * 函    数：任务启动
  * 参    数：无
  * 返 回 值：无
  * 注意事项：启动前必须清零积分和各类状态机（否则上次运行的残留会让车"蹿"出去），
  *           并先更新一次编码器读数，让"上一次计数值"与当前同步（防止首拍出现虚增量）
  */
void Task_Start(void)
{
	MPU_CalibrateZero();		//陀螺零漂自动标定（详见函数定义）
	GyroSumZ = 0;				//陀螺积分清零
	Odom = 0;					//里程清零
	if (TaskNum == 1)			//任务1按子任务设目标直角数（到角自动停）
	{
		Task1_Init((SubRun == 1) ? TASK1_SUB1_CORNERS : TASK1_SUB2_CORNERS, 1);
	}
	else if (TaskNum == 2)		//任务2：借道任务1循迹（Task2_Init里自设角数、到角不停）
	{
		Task2_Init(SubRun);
	}
	else						//任务3：直角计数在收到识别结果后由Task3_Run设置
	{
		Task1_Init(0, 1);
		Task3_Init();
	}
	Disp.CornerCount = 0;		//显示清零
	Disp.TargetCorner = 0;
	Disp.LastError = 0;
	Encoder_Update();			//先同步一次编码器读数
	Running = 1;				//置运行标志
}

/**
  * 函    数：任务停止
  * 参    数：无
  * 返 回 值：无
  */
void Task_Stop(void)
{
	Motor_Stop();				//双电机停止
	Running = 0;				//清运行标志
	Disp.PwmL = 0;				//显示PWM清零
	Disp.PwmR = 0;
}

/**
  * 函    数：MPU6050单轴读数（每拍调用一次）
  * 参    数：无
  * 返 回 值：无
  * 注意事项：只读AccZ+GyroZ两轴共4个字节（约3ms@50kHz），
  *           绝不能用MPU6050_GetData——它内部12次单字节读约8~10ms，
  *           会把10ms控制节拍拉爆（小车一卡一卡的根源）
  */
void MPU_ReadTick(void)
{
	uint8_t DH, DL;
	DH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	MPU_AZ = (int16_t)((DH << 8) | DL);
	DH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	MPU_GZ = (int16_t)((DH << 8) | DL);
}

/**
  * 函    数：开机自检画面
  * 参    数：无
  * 返 回 值：无
  * 注意事项：读MPU6050的ID并验证陀螺数据（防寄存器不兼容读乱数/每拍拖慢节拍），显示
  *           ID和版本号停2秒确认硬件正常；再写第1行静态标签（第2~4行标签由刷新函数
  *           每屏周期自写）；开机连刷23片立即画完整屏（每片约1~2ms，共约40ms）
  */
static void BootScreen(void)
{
	ID = MPU6050_GetID();	//读一次ID（正品=0x68；兼容替代片常见0x70/0x72/0x74/0x98，寄存器兼容就照常用）
	if (ID != 0x68) ID = MPU6050_GetID();	//没读到再试两次，排除总线偶发抖动/首拍没就绪
	if (ID != 0x68) ID = MPU6050_GetID();
	MPU_OnLine = (ID == 0x68 || ID == 0x74);	//0x74=本车实测的兼容替代芯片ID；其他值先判离线，把ID值报告后加入白名单
	if (MPU_OnLine)								//ID对上了再验证陀螺数据正常（防寄存器不兼容读乱数/每拍拖慢节拍）
	{
		int16_t G1 = (int16_t)((MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H) << 8) | MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L));	//静止读一次Z轴陀螺
		int16_t G2 = (int16_t)((MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H) << 8) | MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L));	//再读一次
		if (G1 > 600 || G1 < -600 || G2 > 600 || G2 < -600) MPU_OnLine = 0;	//静止时|读数|应<600LSB(≈36°/s)，超了=乱数
		if ((G1 > G2 ? G1 - G2 : G2 - G1) > 100) MPU_OnLine = 0;				//两次读数不稳定=乱数，也判离线
	}
	OLED_Clear();
	OLED_ShowString(1, 1, "MPU6050 ID:");
	OLED_ShowHexNum(1, 13, ID, 2);	//直接显示实际读到的ID：68=正品、74=本车替代片（都可用）；69=AD0没接地；00/FF=没供电或SCL/SDA断线
	OLED_ShowString(2, 1, "Gray8+MaixCam V1");	//版本标记：看到V1才说明新程序真的烧进去了（任务三完成后版本号重置，准备后续任务）
	OLED_ShowString(3, 1, "PB1: Start/Stop");
	OLED_ShowString(4, 1, "K3:Mid K4:Goal");
	Delay_ms(2000);

	OLED_Clear();			//上电清屏一次，之后循环内不再清屏
	/*第1行静态标签只写一次（第2~4行标签由刷新函数每屏周期自写，防模式切换残留）*/
	OLED_ShowChar(1, 1, 'T');
	OLED_ShowChar(1, 3, '-');
	OLED_ShowChar(1, 5, ' ');
	OLED_ShowChar(1, 9, ' ');
	OLED_ShowChar(1, 10, 'C');
	OLED_ShowChar(1, 12, '/');
	OLED_ShowChar(1, 14, 'V');
	OLED_ShowChar(1, 15, '1');
	OLED_ShowChar(1, 16, ' ');
	/*开机连续刷23片，立即画完整屏（每片约1~2ms，共约40ms，只在上电时做一次）*/
	{
		uint8_t i;
		for (i = 0; i < 23; i++) Display_Refresh(&Disp);
	}
}

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化（PB8=SCL，PB9=SDA）
	MPU6050_Init();		//MPU6050初始化（硬件I2C2：PB10=SCL，PB11=SDA）
	Motor_Init();		//电机初始化（PA2/PA3=PWM，PA4/PA5=AIN1/AIN2，PB12/PB13=BIN1/BIN2）
	Encoder_Init();		//编码器初始化（左轮=PA6/PA7 TIM3，右轮=PB6/PB7 TIM4）
	Gray_Init();		//8路灰度初始化（PA0=OUT模拟输入，PA8/PA11/PA12=AD0/AD1/AD2）
	MaixCam_Init();		//MaixCam串口初始化（USART1：PA9=TX，PA10=RX，115200）
	Key_Init();			//按键初始化（PB1=暂停/继续，PC13=暂停态手动切下一个，PC14=调距键KEY3，PC15=选目标键KEY4）
	/*注意：不要调用LED_Init()，LED模块的PA2与左轮PWM复用冲突*/

	BootScreen();			//开机自检画面+第1行静态标签+首刷整屏（详见函数定义）

	/*打开DWT周期计数器：主循环用它把每拍延时精确补齐到10ms（见循环末尾）*/
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	//开启DWT调试组件
	DWT_CTRL |= 0x00000001;		//使能周期计数器（CYCCNTENA位）
	DWT_CYCCNT = 0;				//计数器清零

	while (1)
	{
		uint32_t T0 = DWT_CYCCNT;		//记录本拍开始时的周期计数（整拍计时，含MPU读取）
		uint8_t KeyNum = Key_GetNum();	//阻塞式按键：按住会卡到松手（原逻辑，只用来暂停/启动/切任务）

		/*按键处理：PB1=暂停/继续（运行中按=暂停，暂停中按=从当前任务+子任务启动，
		  子任务保持不动，反复调试同一段）；PC13=暂停态手动推进一个子任务
		  （1-1→1-2→2-1→2-2→3-1→1-1循环，不按就一直停在当前子任务）*/
		if (KeyNum == 1)
		{
			if (Running) Task_Stop();
			else Task_Start();
		}
		else if (KeyNum == 2 && !Running)
		{
			if (TaskNum == 1)
			{
				if (SubRun < TASK1_SUBRUN_COUNT) SubRun++;
				else { TaskNum = 2; SubRun = 1; }
			}
			else if (TaskNum == 2)
			{
				if (SubRun < TASK2_SUBRUN_COUNT) SubRun++;
				else { TaskNum = 3; SubRun = 1; }
			}
			else
			{
				TaskNum = 1;
				SubRun = 1;
			}
		}

		Tune_Poll(Running, TaskNum, SubRun);	//KEY3调距键轮询（模块Tune.c：任务2暂停态短按+1cm、长按-1cm，按过后第2行显示Mid 3秒）
		Goal_Poll(Running, TaskNum);		//KEY4选目标键轮询（模块Goal.c：任务3暂停态按1~4下选目标角A~D）

		Gray_Read();		//8路灰度读一遍并算好缓存（约4.5ms，每路切通道后等500µs稳定）
		MaixCam_Poll();		//解析MaixCam串口消息（中断只收字节，这里才解析）
		Encoder_Update();	//每拍更新编码器读数
		if (MPU_OnLine)		//MPU在线才读：离线时I2C等超时每拍多花约30ms，节拍拖长会让速度环误判（一前一后抖的根源之一）
		{
			MPU_ReadTick();	//每拍读一次AccZ+GyroZ（4次单字节读，约3ms）
			GyroSumZ += (int32_t)(MPU_GZ - GyroZero) * GYRO_DIR;	//陀螺积分（零漂已按启动时标定值扣除，任务2转弯、任务1转向寻线用；任务3不用陀螺）
		}

		/*执行当前任务（每个循环执行一小步，每步约10ms）；任务返回1=已完成，统一停车收尾*/
		if (Running)
		{
			uint8_t Done = 0;
			if (TaskNum == 1) Done = Task1_Run(&Disp);
			else if (TaskNum == 2) Done = Task2_Run(&Disp);
			else Done = Task3_Run(&Disp);
			if (Done) Task_Stop();
		}

		/*每拍刷一小片OLED（≤2个字符，约1~2ms），再用DWT周期计数器把本拍补齐到10ms：
		   写屏时间摊到每一拍、节拍严格10ms，PI不会因节拍抖动误纠（一卡一卡的根源）；
		   MPU掉线时I2C超时会使本拍超过10ms，此时不再延时（加了ElapsedUs<10000防下溢）*/
		Disp.Running = Running;
		Disp.TaskNum = TaskNum;
		Disp.SubRun = SubRun;
		Disp.GyroSumZ = GyroSumZ;
		Disp.OdomCm = (int16_t)(Odom / ENC_COUNTS_PER_CM);	//里程换算成cm显示（推车标定每厘米脉冲数看它）
		Disp.TuneMode = Tune_ShowActive();		//调距显示：按过KEY3后第2行显示Mid 3秒（显示计时在Tune.c里）
		Disp.TuneCm = Task2_GetMidDist(SubRun);			//当前子任务中点距离（cm，调距显示用）
		Disp.GoalMode = (!Running && TaskNum == 3 && Goal_Get() != 0) ? 1 : 0;	//选目标显示：任务3暂停态常显Goal:A~D
		Disp.GoalNum = Goal_Get();				//按键选的目标角号（1~4=A~D）
		/*cam识别显示（任务3 专用，暂停和运行都一直显示）：
		  用 PeekTarget 而不是 GetTarget —— GetTarget 会把消息从任务三手里抢走，
		  用它来显示的话任务三永远收到 0、车永远不动；PeekTarget 只看不取。
		  三种状态，一眼分开，排查时不用瞎猜：
		    2 = 识别到了        → 第2行 "Cam:5>C"（左边=卡片号码，右边=要去的角）
		    1 = 相机在线但没看到卡 → 第2行 "Cam:--"（串口通着、相机在看着，只是没认到卡）
		    0 = 从来没收到过消息   → 第2行按老规则走（Ang/Goal...），说明相机没跑、线没通、
		                            或者刚按完PB1还没收到新帧
		  【和按键的关系】cam 优先级高于按键目标，和 Task3_Run 的规则一致——
		  屏幕显示的永远等于车实际会用的目标，不会出现"显示B角、车却跑去C角"这种骗人的情况。*/
		Disp.CamNum = MaixCam_PeekTarget();		//MaixCam最近识别到的目标角号（1~4=A~D，0=还没识别到）
		Disp.CamState = 0;
		if (TaskNum == 3)
		{
			if (Disp.CamNum != 0)		Disp.CamState = 2;		//识别到了
			else if (MaixCam_IsAlive())	Disp.CamState = 1;		//相机在线但没识别到卡
		}
		Display_Refresh(&Disp);
		{
			uint32_t ElapsedUs = (DWT_CYCCNT - T0) / 72;	//本拍实际耗时（72MHz，72个周期=1µs）
			if (ElapsedUs < TICK_MS * 1000) Delay_us(TICK_MS * 1000 - ElapsedUs);	//补齐到10ms
		}
	}
}
