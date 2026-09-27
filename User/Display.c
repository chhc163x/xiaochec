#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Encoder.h"
#include "Gray.h"
#include "Display.h"

/*==================== 内部缓存 ====================*/
static uint8_t OledChunk = 0;			//OLED分片刷新的片号（0~22），每拍刷一小片
static int16_t SpeedL = 0;				//左轮实测速度（计数/拍），本屏周期内左半片和右半片共用
static int16_t SpeedR = 0;				//右轮实测速度

/**
  * 函    数：带符号数格式化
  * 参    数：Buf 输出缓冲区（长度=Len+1），Num 数值，Len 数字位数
  * 返 回 值：无
  * 注意事项：Buf[0]放符号（+/-），Buf[1..Len]放数字字符，供分片显示逐字符上屏
  */
static void FormatSigned(char *Buf, int16_t Num, uint8_t Len)
{
	uint8_t i;
	uint16_t N;
	uint16_t Div = 1;
	if (Num < 0) { Buf[0] = '-'; N = (uint16_t)(-Num); }
	else { Buf[0] = '+'; N = (uint16_t)Num; }
	for (i = 0; i < Len - 1; i++) Div *= 10;
	for (i = 0; i < Len; i++)
	{
		Buf[i + 1] = N / Div % 10 + '0';
		Div /= 10;
	}
}

/**
  * 函    数：OLED刷新（分片版，4行竞赛布局，仿6-3工程显示风格）
  * 参    数：D 显示数据指针（main每拍填充，任务函数每拍写入任务相关字段）
  * 返 回 值：无
  * 注意事项：每次调用只刷一小片（≤3个字符，约1~3ms），调用23次刷完整屏：
  *           第1行=任务-子任务+运行态+已过/目标直角数+V1版本标记；
  *           第2行=运行中：左右轮PWM占空比（PWM:L +xxx R +xxx，查哪只轮没输出）；
  *           第3行=8路灰度条（G:L 00000000 R，1=压线）；
  *           第4行=运行中：左右轮实测速度+循迹误差（L:+xx R:+xx E:+xx）；
  *           停车时第2行=陀螺角度Ang+累计里程O（cm，推车标定每厘米脉冲数看它）、
  *           第4行=实测速度L/R（推车看编码器方向）；
  *           调距模式（按过KEY3调距键后显示3秒，任务2专用）第2行=中点距离Mid:+xxx cm
  *           （微调"转弯后走多远"，KEY3短按+1cm、长按−1cm）；
  *           选目标模式（任务3暂停态按键选目标后常显）第2行=Goal:A~D
  *           （KEY4按1~4下选角A~D）；
  *           cam识别模式（任务3暂停态、已收到MaixCam识别结果时显示，优先级高于按键目标）
  *           第2行=Cam:5>B，左边是MaixCam实际识别到的数字卡、右边是映射到的角
  *           （1=A、2=B、3=C、4=D）——把原数字也显示出来，是为了能分清
  *           "卡认错了"和"压根没收到"，排查串口时特别有用。
  *           主循环每10ms调用一次：写屏时间摊到每一拍，且主循环用DWT周期
  *           计数器把每拍精确补齐到10ms——整屏集中刷新会把这一拍拉长到
  *           20ms以上，下一拍编码器读数虚高、PI误判超速猛纠，小车就"一卡一卡"。
  *           第1行静态标签在main里开机只写一次；第2~4行标签由刷新函数每
  *           屏周期自写，第2行"运行/停车"两种模式切换时不残留旧字符。
  */
void Display_Refresh(DispInfo *D)
{
	static char BufPwm[4];				//PWM显示缓冲区：符号+3位数字（左右轮共用）
	static char BufAng[5];				//陀螺角度显示缓冲区：符号+4位数字(0.1°)
	static char BufTune[4];				//调距中点距离显示缓冲区：符号+3位数字(cm)
	static char BufSpeed[3];			//速度显示缓冲区：符号+2位数字（左右轮共用）
	static char BufErr[3];				//循迹误差显示缓冲区：符号+2位数字
	static char BufOdom[4];				//里程显示缓冲区：符号+3位数字(cm，停车时第2行显示)
	static const char CardDigit[5] = { '-', '1', '3', '5', '7' };
										//角号→卡片号码的对照表：角1=卡1、角2=卡3、角3=卡5、角4=卡7
										//（题目规定 I→A、III→B、V→C、VII→D，所以卡号 = 2×角号−1）
										//屏幕上要显示"卡号"而不是"角号"，否则卡片3会显示成"2>B"让人看不懂
	static uint8_t BarState = 0;		//本屏周期灰度条快照
	uint8_t i;
	int32_t AngleQ;

	switch (OledChunk)
	{
		case 0:		/*第1行：任务号*/
			OLED_ShowNum(1, 2, D->TaskNum, 1);
			break;
		case 1:		/*第1行：子任务号*/
			OLED_ShowNum(1, 4, D->SubRun, 1);
			break;
		case 2:		/*第1行：Run/Stp前2个字符*/
			if (D->Running) { OLED_ShowChar(1, 6, 'R'); OLED_ShowChar(1, 7, 'u'); }
			else { OLED_ShowChar(1, 6, 'S'); OLED_ShowChar(1, 7, 't'); }
			break;
		case 3:		/*第1行：Run/Stp第3个字符*/
			if (D->Running) OLED_ShowChar(1, 8, 'n');
			else OLED_ShowChar(1, 8, 'p');
			break;
		case 4:		/*第1行：已过直角个数*/
			OLED_ShowNum(1, 11, D->CornerCount, 1);
			break;
		case 5:		/*第1行：目标直角个数*/
			OLED_ShowNum(1, 13, D->TargetCorner, 1);
			break;
		case 6:		/*第2行：标签（cam识别=Cam，运行=PWM，调距=Mid，选目标=Goal，停车=Ang）
					  cam 排在最前面：任务3 一旦收到过识别结果，跑起来也一直显示 Cam，
					  直到 cam 识别到别的数字（或退出任务3）才变——这是用户要求的效果*/
			if (D->CamState) OLED_ShowString(2, 1, "Cam");
			else if (D->Running) OLED_ShowString(2, 1, "PWM");
			else if (D->TuneMode) OLED_ShowString(2, 1, "Mid");
			else if (D->GoalMode) OLED_ShowString(2, 1, "Goal");
			else OLED_ShowString(2, 1, "Ang");
			break;
		case 7:		/*第2行：cam识别=:卡号>角 或 :--，运行=:L+空格，调距=:符号+首位，选目标=:角字母，停车=角度符号+首位*/
			if (D->CamState == 2)		//识别到了：Cam:5>C
			{
				/*左边必须显示**卡片本身的号码**（1/3/5/7），不能显示角号！
				  角号2对应的卡片是"3"，显示成 2>B 会让人以为"卡是3怎么显示2"。
				  右边是这张卡要去的角（角1=A、角2=B、角3=C、角4=D）。*/
				OLED_ShowChar(2, 4, ':');
				OLED_ShowChar(2, 5, CardDigit[D->CamNum]);			//卡片号码：1/3/5/7
				OLED_ShowChar(2, 6, '>');
				OLED_ShowChar(2, 7, (char)('A' + D->CamNum - 1));	//要去的角：A~D
			}
			else if (D->CamState == 1)	//相机在线但没看到卡：Cam:--
			{
				/*单独给一个提示，和"根本没连上相机"（那时显示 Ang/Goal）区分开：
				  看到 Cam:-- 就知道串口是通的、相机在看着，只是没认到卡（卡没摆正/太远/光太暗）。*/
				OLED_ShowChar(2, 4, ':');
				OLED_ShowChar(2, 5, '-');
				OLED_ShowChar(2, 6, '-');
				OLED_ShowChar(2, 7, ' ');
			}
			else if (D->Running)
			{
				OLED_ShowString(2, 4, ":L");
				OLED_ShowChar(2, 6, ' ');
			}
			else if (D->TuneMode)
			{
				FormatSigned(BufTune, D->TuneCm, 3);	//中点距离cm：符号+3位
				OLED_ShowChar(2, 4, ':');
				OLED_ShowChar(2, 5, BufTune[0]);
				OLED_ShowChar(2, 6, BufTune[1]);
			}
			else if (D->GoalMode)
			{
				/*注意"Goal"是4个字符（占第1~4列），所以冒号必须写在第5列！
				  老代码把冒号写在第4列、把'l'覆盖掉了，实际显示成"Goa:B"（已修）*/
				OLED_ShowChar(2, 5, ':');
				OLED_ShowChar(2, 6, (char)('A' + D->GoalNum - 1));	//角号转字母：1~4=A~D
				OLED_ShowChar(2, 7, ' ');
			}
			else
			{
				AngleQ = D->GyroSumZ / 164;			//换算成0.1°（1°=1640 LSB，÷164=×10/1640）
				if (AngleQ > 9999) AngleQ = 9999;	//限幅防溢出
				if (AngleQ < -9999) AngleQ = -9999;
				FormatSigned(BufAng, (int16_t)AngleQ, 4);
				OLED_ShowChar(2, 4, ':');
				OLED_ShowChar(2, 5, BufAng[0]);
				OLED_ShowChar(2, 6, BufAng[1]);
			}
			break;
		case 8:		/*第2行：cam/选目标=清格，运行=左PWM符号+2位，调距=距离2位+cm，停车=角度2位*/
			if (D->CamState || D->GoalMode)
			{
				/*第7列已经被Cam的角字母/Goal的空格占了，这里从第8列起清*/
				OLED_ShowChar(2, 8, ' ');
				OLED_ShowChar(2, 9, ' ');
				OLED_ShowChar(2, 10, ' ');
			}
			else if (D->Running)
			{
				FormatSigned(BufPwm, D->PwmL, 3);
				OLED_ShowChar(2, 7, BufPwm[0]);
				OLED_ShowChar(2, 8, BufPwm[1]);
				OLED_ShowChar(2, 9, BufPwm[2]);
			}
			else if (D->TuneMode)
			{
				OLED_ShowChar(2, 7, BufTune[2]);
				OLED_ShowChar(2, 8, BufTune[3]);
				OLED_ShowChar(2, 9, 'c');
			}
			else
			{
				OLED_ShowChar(2, 7, BufAng[2]);
				OLED_ShowChar(2, 8, BufAng[3]);
			}
			break;
		case 9:		/*第2行：cam/选目标=清格，运行=左PWM第3位+R标签，调距=cm收尾，停车=角度第4位+里程O标签*/
			if (D->CamState || D->GoalMode)
			{
				OLED_ShowChar(2, 11, ' ');
				OLED_ShowChar(2, 12, ' ');
				OLED_ShowChar(2, 13, ' ');
			}
			else if (D->Running)
			{
				OLED_ShowChar(2, 10, BufPwm[3]);
				OLED_ShowChar(2, 11, ' ');
				OLED_ShowChar(2, 12, 'R');
			}
			else if (D->TuneMode)
			{
				OLED_ShowChar(2, 10, 'm');
				OLED_ShowChar(2, 11, ' ');
				OLED_ShowChar(2, 12, ' ');
			}
			else
			{
				OLED_ShowChar(2, 9, BufAng[4]);
				OLED_ShowChar(2, 10, ' ');
				OLED_ShowChar(2, 11, 'O');
			}
			break;
		case 10:	/*第2行：cam/选目标=清格(14~16)，运行=右PWM符号+2位，调距=清格(13~15)，停车=里程:+符号+首位*/
			if (D->CamState || D->GoalMode)
			{
				OLED_ShowChar(2, 14, ' ');
				OLED_ShowChar(2, 15, ' ');
				OLED_ShowChar(2, 16, ' ');
			}
			else if (D->Running)
			{
				FormatSigned(BufPwm, D->PwmR, 3);
				OLED_ShowChar(2, 13, BufPwm[0]);
				OLED_ShowChar(2, 14, BufPwm[1]);
				OLED_ShowChar(2, 15, BufPwm[2]);
			}
			else if (D->TuneMode)
			{
				OLED_ShowChar(2, 13, ' ');
				OLED_ShowChar(2, 14, ' ');
				OLED_ShowChar(2, 15, ' ');
			}
			else
			{
				FormatSigned(BufOdom, D->OdomCm, 3);
				OLED_ShowChar(2, 12, ':');
				OLED_ShowChar(2, 13, BufOdom[0]);
				OLED_ShowChar(2, 14, BufOdom[1]);
			}
			break;
		case 11:	/*第2行：cam/选目标=清格，运行=右PWM第3位，调距=清格，停车=里程2位数字
					  注意顺序：Running 要排在 TuneMode 前面，和 case 8/9/10 保持一致——
					  "按过KEY3后3秒内又按了PB1启动"时 Running 和 TuneMode 会同时为真，
					  这里必须让 Running 优先，否则第2行会缺一个 PWM 字符*/
			if (D->CamState || D->GoalMode) OLED_ShowChar(2, 16, ' ');
			else if (D->Running) OLED_ShowChar(2, 16, BufPwm[3]);
			else if (D->TuneMode) OLED_ShowChar(2, 16, ' ');
			else
			{
				OLED_ShowChar(2, 15, BufOdom[2]);
				OLED_ShowChar(2, 16, BufOdom[3]);
			}
			break;
		case 12:	/*第3行：灰度条标签G:L*/
			OLED_ShowString(3, 1, "G:L");
			break;
		case 13:	/*第3行：灰度条ch0~ch1（最左端2路）*/
			OLED_ShowChar(3, 4, ' ');
			BarState = Gray_GetState();
			for (i = 0; i < 2; i++)
			{
				if (BarState & (0x01 << i)) OLED_ShowChar(3, 5 + i, '1');
				else OLED_ShowChar(3, 5 + i, '0');
			}
			break;
		case 14:	/*第3行：灰度条ch2~ch4*/
			for (i = 0; i < 3; i++)
			{
				if (BarState & (0x04 << i)) OLED_ShowChar(3, 7 + i, '1');
				else OLED_ShowChar(3, 7 + i, '0');
			}
			break;
		case 15:	/*第3行：灰度条ch5~ch7（最右端3路）*/
			for (i = 0; i < 3; i++)
			{
				if (BarState & (0x20 << i)) OLED_ShowChar(3, 10 + i, '1');
				else OLED_ShowChar(3, 10 + i, '0');
			}
			break;
		case 16:	/*第3行：右端标签R*/
			OLED_ShowChar(3, 13, ' ');
			OLED_ShowChar(3, 14, 'R');
			break;
		case 17:	/*第4行：L标签+左速度符号（停车也实时显示，推车看编码器方向）*/
			OLED_ShowString(4, 1, "L:");
			SpeedL = Encoder_GetLeft();
			FormatSigned(BufSpeed, SpeedL, 2);
			OLED_ShowChar(4, 3, BufSpeed[0]);
			break;
		case 18:	/*第4行：左速度2位+空隙*/
			OLED_ShowChar(4, 4, BufSpeed[1]);
			OLED_ShowChar(4, 5, BufSpeed[2]);
			OLED_ShowChar(4, 6, ' ');
			break;
		case 19:	/*第4行：R标签+右速度符号*/
			OLED_ShowString(4, 7, "R:");
			SpeedR = Encoder_GetRight();
			FormatSigned(BufSpeed, SpeedR, 2);
			OLED_ShowChar(4, 9, BufSpeed[0]);
			break;
		case 20:	/*第4行：右速度2位+空隙*/
			OLED_ShowChar(4, 10, BufSpeed[1]);
			OLED_ShowChar(4, 11, BufSpeed[2]);
			OLED_ShowChar(4, 12, ' ');
			break;
		case 21:	/*第4行：运行=循迹误差E标签+符号，停车=清格*/
			if (D->Running)
			{
				OLED_ShowChar(4, 13, 'E');
				FormatSigned(BufErr, D->LastError, 2);
				OLED_ShowChar(4, 14, BufErr[0]);
			}
			else
			{
				OLED_ShowChar(4, 13, ' ');
				OLED_ShowChar(4, 14, ' ');
				OLED_ShowChar(4, 15, ' ');
			}
			break;
		case 22:	/*第4行：运行=循迹误差2位数字，停车=清格*/
			if (D->Running)
			{
				OLED_ShowChar(4, 15, BufErr[1]);
				OLED_ShowChar(4, 16, BufErr[2]);
			}
			else OLED_ShowChar(4, 16, ' ');
			break;
	}
	OledChunk++;
	if (OledChunk >= 23) OledChunk = 0;		//23片刷完，从头再来
}
