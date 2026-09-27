#include "stm32f10x.h"                  // Device header
#include "MaixCam.h"

/*==================== 接收环形缓冲 ====================*/
static volatile uint8_t MaixRing[MAIXCAM_BUF_SIZE];		//环形缓冲区
static volatile uint8_t MaixHead = 0;					//写指针（中断里移动）
static volatile uint8_t MaixTail = 0;					//读指针（主循环里移动）

/*==================== 解析状态 ====================*/
static char MaixLine[MAIXCAM_LINE_MAX];		//行组装缓冲
static uint8_t MaixLineLen = 0;
static uint8_t MaixTarget = 0;				//最新识别目标：0=无，1/2/3/4=角A/B/C/D
static uint8_t MaixNew = 0;					//新消息标志
static uint8_t MaixLast = 0;				//【只给OLED显示用】最近一次"真的识别到"的目标
											//为什么单独存一个：MaixTarget每解析一行就会被重写，
											//杂数据/空行会把0写进去，OLED就会一闪一闪；这个只在
											//解析出有效目标时才更新，显示才稳定
static uint16_t MaixIdle = 0xFFFF;			//距上一帧有效消息过了多少拍（MaixCam_Poll每拍+1，
											//收到任何一帧就清0），MaixCam_IsAlive()用它判在线

/**
  * 函    数：MaixCam串口初始化（USART1）
  * 参    数：无
  * 返 回 值：无
  * 注意事项：PA9=TX复用推挽输出，PA10=RX浮空输入，115200 8N1；
  *           接收中断抢断优先级设1（不打断10ms主循环的实时性）
  */
void MaixCam_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		//开启USART1的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;					//PA9=TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;					//PA10=RX
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*USART1初始化：115200 8N1，收发都开（将来要回发指令可复用TX）*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/*NVIC：使能接收中断*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		//开启接收中断
	USART_Cmd(USART1, ENABLE);							//使能USART1
}

/**
  * 函    数：USART1接收中断服务
  * 参    数：无
  * 返 回 值：无
  * 注意事项：只做入环，不做解析（中断里干别的活会拖慢10ms主循环）；
  *           缓冲区满时丢最旧字节，保证缓冲区里的永远是最新的数据
  */
void MaixCam_IRQHandler(void)
{
	uint8_t Next;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)	//接收中断标志
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	//清标志
		Next = (uint8_t)((MaixHead + 1) % MAIXCAM_BUF_SIZE);
		if (Next == MaixTail)							//缓冲区满：丢最旧字节
		{
			MaixTail = (uint8_t)((MaixTail + 1) % MAIXCAM_BUF_SIZE);
		}
		MaixRing[MaixHead] = USART_ReceiveData(USART1);	//读数据（同时清RXNE）
		MaixHead = Next;
	}
}

/**
  * 函    数：解析一行识别消息
  * 参    数：Line 行缓冲，Len 行长度
  * 返 回 值：目标角号1/2/3/4=角A/B/C/D，5=相机在线但没识别到卡，0=未识别
  * 注意事项：罗马数字必须最长优先匹配（先VII/III再V/I），
  *           否则"III"会被先匹配到单字符"I"；$、*等帧头尾字节不进缓冲区。
  *           【返回值5的来历】分类模型没有"背景类"，画面里没卡片时它也会硬判成某张卡，
  *           所以 MaixCam 端程序在"没识别到"时会主动发一个单字符"0"当心跳，
  *           本函数把它认成5——这样 STM32 就能区分"相机在线但没卡"和"相机根本没连上"。
  *           限定"整行只有一个0"，是为了避免把日志里的 "ttyS0" 之类的杂串误判成心跳。
  */
static uint8_t MaixParse(char *Line, uint8_t Len)
{
	uint8_t i;
	if (Len == 1 && Line[0] == '0') return 5;		//MaixCam在线的"没识别到"心跳
	for (i = 0; i + 3 <= Len; i++)					//先匹配3字符：VII→D角、III→B角
	{
		if (Line[i] == 'V' && Line[i + 1] == 'I' && Line[i + 2] == 'I') return 4;
		if (Line[i] == 'I' && Line[i + 1] == 'I' && Line[i + 2] == 'I') return 2;
	}
	for (i = 0; i + 1 <= Len; i++)					//再匹配2字符：II→B角
	{
		if (Line[i] == 'I' && Line[i + 1] == 'I') return 2;
	}
	for (i = 0; i < Len; i++)						//再匹配单字符：V→C角
	{
		if (Line[i] == 'V') return 3;
	}
	for (i = 0; i < Len; i++)						//再匹配单字符：I→A角
	{
		if (Line[i] == 'I') return 1;
	}
	for (i = 0; i < Len; i++)						//最后匹配阿拉伯数字：7→D、5→C、3→B、1→A
	{
		if (Line[i] == '7') return 4;
		if (Line[i] == '5') return 3;
		if (Line[i] == '3') return 2;
		if (Line[i] == '1') return 1;
	}
	return 0;										//没有匹配到任何目标
}

/**
  * 函    数：把一行的解析结果落到模块状态上
  * 参    数：Ret MaixParse 的返回值（0=杂数据，1~4=真识别到，5=在线但没识别到）
  * 返 回 值：无
  * 注意事项：5（"相机在线但没识别到"的心跳）只刷新在线计时，**绝不能置 MaixNew**，
  *           否则任务三会被"没看到卡"触发发车；也不动 MaixLast，
  *           这样屏幕上的"上次识别到的数字"不会被心跳冲掉。
  *           Ret==0（杂数据/空行）什么都不做——以前这里会清 MaixTarget/MaixNew，
  *           结果杂数据能把还没被任务三取走的真目标冲掉，现在不动它，真目标能一直留着。
  */
static void MaixCommit(uint8_t Ret)
{
	if (Ret == 5)					//相机在线但没识别到（心跳）
	{
		MaixIdle = 0;				//只刷新在线计时
	}
	else if (Ret != 0)				//真识别到：1/2/3/4=角A/B/C/D
	{
		MaixIdle = 0;
		MaixTarget = Ret;
		MaixNew = 1;
		MaixLast = Ret;				//只有真识别到才更新"最近目标"（OLED显示用，防杂数据闪屏）
	}
}

/**
  * 函    数：主循环轮询解析（每10ms调一次）
  * 参    数：无
  * 返 回 值：无
  * 注意事项：从环形缓冲取字节组装成行，遇到\n、\r、*、$或行超长就解析一行；
  *           小写字母统一转大写；解析结果交给 MaixCommit 落到状态上。
  *           函数开头给在线计时+1（收到有效帧时 MaixCommit 会清零），
  *           MaixCam_IsAlive() 靠它判断相机还在不在。
  */
void MaixCam_Poll(void)
{
	uint8_t Data;
	if (MaixIdle < 0xFFFF) MaixIdle++;				//在线计时每拍+1（收到有效帧会被清零）
	while (MaixTail != MaixHead)					//环形缓冲里还有字节
	{
		Data = MaixRing[MaixTail];
		MaixTail = (uint8_t)((MaixTail + 1) % MAIXCAM_BUF_SIZE);
		if (Data >= 'a' && Data <= 'z') Data -= 32;	//统一转大写
		if (Data == '\n' || Data == '\r' || Data == '*' || Data == '$')	//行终止符
		{
			if (MaixLineLen > 0)					//有内容才解析（空行忽略）
			{
				MaixCommit(MaixParse(MaixLine, MaixLineLen));
				MaixLineLen = 0;
			}
			continue;
		}
		if (MaixLineLen < MAIXCAM_LINE_MAX - 1)		//行缓冲未满：正常追加
		{
			MaixLine[MaixLineLen++] = (char)Data;
		}
		else										//行超长：强制结束并解析，当前字节放进新行
		{
			MaixCommit(MaixParse(MaixLine, MaixLineLen));
			MaixLineLen = 0;
			MaixLine[MaixLineLen++] = (char)Data;
		}
	}
}

/**
  * 函    数：清空MaixCam串口缓存与解析状态
  * 参    数：无
  * 返 回 值：无
  * 注意事项：【为什么必须有这个函数】
  *           MaixCAM 的串口是它的系统日志口(ttyS0)，开机那几秒会往外吐一大串启动日志，
  *           里面夹着 "115200" 这种数字——本模块的解析是"看到 5 就当 C角、看到 1 就当 A角"，
  *           于是开机日志会被解析成一个假目标（比如 115200 里的 5 → C角）并置 MaixNew=1。
  *           而 MaixNew 要等 MaixCam_GetTarget() 被调用才清——任务三之前没人调它，
  *           所以用户一按 PB1 进任务三，MaixCam_GetTarget 立刻返回这个假目标，
  *           车不发车就直接按 C 角跑——查半天接线都查不出来是这个原因。
  *           （文档旧版写的"启动前发的旧消息不会误触发"其实并不成立，就是这个 bug）
  *           所以每次进任务三前先把"启动前积压的字节 + 已经解析出来的标志"一起清掉，
  *           之后只认进任务三之后新到的消息。
  *           关中断再清是为了不与接收中断抢 MaixHead/MaixTail，避免清一半又插进来一个字节
  */
void MaixCam_Flush(void)
{
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);		//先关接收中断，独占缓冲区
	MaixTail = MaixHead;								//读指针追平写指针=丢弃所有未读字节
	MaixLineLen = 0;									//行组装缓冲清零
	MaixTarget = 0;										//已解析的目标清零
	MaixNew = 0;										//清"有新消息"标志，防止旧结果误触发任务三
	MaixLast = 0;										//OLED显示的"最近目标"也清掉：
														//这样按PB1重跑时屏幕先空一下，0.5秒后显示的新值才是这一次的识别结果
	MaixIdle = 0xFFFF;									//在线计时置满：清完缓存后先按"不在线"算，
														//等 cam 重新发来一帧（识别到发数字、没识别到发0）才重新判为在线
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		//恢复接收中断
}

/**
  * 函    数：获取最新识别目标
  * 参    数：无
  * 返 回 值：0=没有新消息，1/2/3/4=角A/B/C/D
  * 注意事项：有新消息时返回目标角号并清标志；无新消息返回0
  *           （防止任务3被很久以前的旧消息误触发）
  */
uint8_t MaixCam_GetTarget(void)
{
	if (MaixNew)				//有新消息
	{
		MaixNew = 0;			//清标志
		return MaixTarget;		//返回目标角号
	}
	return 0;
}

/**
  * 函    数：查看最近一次识别到的目标（**只看不取**）
  * 参    数：无
  * 返 回 值：0=还没识别到，1/2/3/4=角A/B/C/D
  * 注意事项：【和 MaixCam_GetTarget 的区别，别用错】
  *           GetTarget 会清 MaixNew（把消息"取走"），是给任务三用的；
  *           本函数只读 MaixLast、不动任何标志，是给 OLED 显示用的。
  *           如果显示也用 GetTarget，主循环每拍都会把消息抢走，
  *           任务三那边永远收到 0，车就永远不动——这是最容易踩的坑。
  *           另外它返回的是 MaixLast（只在解析出有效目标时更新），
  *           所以串口上的杂数据不会把屏幕上的显示冲掉。
  */
uint8_t MaixCam_PeekTarget(void)
{
	return MaixLast;
}

/**
  * 函    数：MaixCam 是否在线
  * 参    数：无
  * 返 回 值：1=在线（最近 MAIXCAM_ALIVE_TICKS 拍内收到过消息），0=掉线/没连
  * 注意事项：【用途】把两种"屏幕上没数字"的情况区分开：
  *           在线但没识别到 → 屏幕显示 Cam:--（相机在看着，只是没卡）
  *           不在线        → 屏幕显示 Ang/Goal（相机没跑、线没通、或者刚按完PB1还没收到新帧）
  *           MaixCam 端程序每500ms发一帧（识别到发数字 1/3/5/7，没识别到发 "0"），
  *           所以2.5秒收不到就可以认定掉线——这个余量足够容忍偶尔丢一帧。
  */
uint8_t MaixCam_IsAlive(void)
{
	return (MaixIdle < MAIXCAM_ALIVE_TICKS) ? 1 : 0;
}
