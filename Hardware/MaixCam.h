#ifndef __MAIXCAM_H
#define __MAIXCAM_H

/*MaixCam视觉识别结果接收驱动（STM32只收信号，不写识别代码）：
  接线：MaixCam串口TX→STM32 PA10(RX)、MaixCam RX→STM32 PA9(TX)、共GND、3.3V电平
  协议：USART1 115200 8N1；识别结果格式宽容，支持"1/3/5/7"或"I/III/V/VII"，
        前后可带$、*、回车等帧头尾字符
  目标映射：1/I→A角、3/III→B角、5/V→C角、7/VII→D角
  中断里只把字节收进环形缓冲，解析在主循环MaixCam_Poll()完成，不拖慢10ms节拍*/

#define MAIXCAM_BUF_SIZE	64		//接收环形缓冲区大小（字节）
#define MAIXCAM_LINE_MAX	16		//单行消息最大长度（字节）
#define MAIXCAM_ALIVE_TICKS	250		//相机"在线"判定：最近250拍(2.5秒)内收到过消息才算在线
									//MaixCam 每500ms发一帧（识别到发数字、没识别到发"0"），2.5秒没动静=掉线

void MaixCam_Init(void);			//初始化USART1+NVIC（接收中断已开启）
void MaixCam_IRQHandler(void);		//USART1中断服务（在stm32f10x_it.c的USART1_IRQHandler中调用）
void MaixCam_Poll(void);			//主循环每拍调用：从环形缓冲取字节并解析整行消息
void MaixCam_Flush(void);			//清空历史字节+解析状态（每次进任务三前调用，见.c里的说明）
uint8_t MaixCam_GetTarget(void);	//取最新识别目标：0=无新消息，1/2/3/4=角A/B/C/D，取后清标志
uint8_t MaixCam_PeekTarget(void);	//只看"最近一次识别到的目标"，0=还没识别到，1/2/3/4=角A/B/C/D
									//【区别】GetTarget会"取走"消息（清标志），PeekTarget只看不取——
									//  OLED显示用这个，否则主循环会把消息从任务三手里抢走
uint8_t MaixCam_IsAlive(void);		//MaixCam 是否在线：1=最近2.5秒内收到过消息（含"没识别到"的心跳）
									//用来区分"相机在线但没看到卡"和"相机根本没连上"

#endif
