#ifndef __CONFIG_H
#define __CONFIG_H

/*==================== 全车可调参数总表（所有任务/模块的调参都在这里，不用翻别的文件） ====================*/

/*---- 通用/速度环 ----*/
#define TRIM_LEFT		0		//左轮目标速度微调，车往左偏就把这个值调大（或把TRIM_RIGHT调小）
#define TRIM_RIGHT		0		//右轮目标速度微调
#define SPEED_KP		2		//速度环比例系数：输出(%) = KP × 速度误差(计数/拍)
#define SPEED_KI		1		//速度环积分系数：输出(%) 累加 KI×积分÷16（弱积分，防起步/悬空时积分快速饱和）
#define INTEGRAL_LIMIT	1000	//积分限幅：1000÷16≈62，积分最多贡献约±62%占空比（配合KP项够用）
#define OUTPUT_LIMIT	100		//输出占空比限幅，最大±100
#define TARGET_LIMIT	60		//目标速度限幅，防止循迹大误差时转向项过大

/*---- 任务1：巡线（8路灰度） ----*/
#define LINE_BASE			16		//循迹基础速度（编码器计数/拍），直线想快点就调大，拐弯刹不住就调小
#define LINE_KP				3		//循迹误差系数：目标速度 = LINE_BASE ± LINE_KP×误差（8路误差±10）；KP=6太猛误差≥3就倒车，改3后误差±3内两轮都正转
#define FOLLOW_MIN_TARGET	0		//跟随阶段目标下限：循迹中绝不倒车（倒车只留给转弯、任务2倒库等专用阶段）
#define CORNER_DEBOUNCE		5		//跟随中连续5拍(50ms)全丢线=经过一个直角
#define START_DEBOUNCE_TICKS	10	//起步宽限期10拍(100ms)：起点压角A，防止起步瞬间判成经过直角
#define TURN_DUTY			15		//陀螺离线时的兜底开环转弯占空比（左+右-原地右转；陀螺在线时由角速度PID接管）
#define TURN_DUTY_MAX		35		//转弯角速度PID输出限幅：占空比±35封顶
#define TURN_RATE_CRUISE	82		//转向寻线角速度82LSB/拍=5°/s（16.4LSB≈1°/s）：丢线后原地慢慢右转，转多慢都没事，压到线就恢复循迹
#define TURN_RAMP_TICKS		30		//起步300ms内角速度从0线性升到寻线速度：软启动，不是猛的一下转起来
#define TURN_BLIND_TICKS	15		//转向刚开始150ms内不看探头（防扫到刚丢掉的旧线提前恢复）
#define TURN_TIMEOUT_TICKS	3000	//转向寻线超过30秒还没压到线，强制停车兜底（5°/s转90°要18秒，留足余量）
#define TURN_COOLDOWN_TICKS	30		//转完回跟随后冷却30拍(300ms)，期间不触发新转弯（防连转）
#define TASK1_SUB1_CORNERS	1		//子任务1-1：A→B，经过1个直角后自动停
#define TASK1_SUB2_CORNERS	4		//子任务1-2：绕场一周回A，经过4个直角后自动停
#define TASK1_SUBRUN_COUNT	2		//任务1子任务个数（PC13手动推进用）
#define TASK2_SUBRUN_COUNT	2		//任务2子任务个数（PC13手动推进用）

/*---- 任务2：停车入库（2-1 已按实测流程配置，其余参数待实车标定） ----*/
#define ENC_COUNTS_PER_CM	77		//每厘米脉冲数=1560/(π×65mm÷10)（MG513：13PPR×30减速比×4倍频=1560/圈，轮径65mm）；必须推尺实测标定（倒库成败关键，标定方法见文档）
#define TASK2_SUB1_CORNERS	1		//子任务2-1：A点发车沿AB边循迹到B角，经过1个直角（到角不停车，寻线转弯后继续走）
#define TASK2_BC_DIST_CM	45		//子任务2-1：B角转弯完成后沿BC边走45cm到车库（已实测定稿=45；1m场地中点距B=50cm，轮径误差测出45正好）；转弯完成瞬间起计里程，不记发车总距离；运行时可调：暂停态按KEY3调距键（PC14）微调（短按+1cm、长按−1cm），调好把值写回这里重烧
#define TASK2_TURN_ANGLE_DEG	90	//子任务2-1：走到车库旁后原地转90°（车尾对准车库）
#define TASK2_TURN_STOP_LEAD_DEG	25	//转到位判定提前角：提前25°（转到65°）就停、靠惯性滑到90°（实测提前20°仍多一点点，改25）
#define TASK2_REVERSE_CM	25		//子任务2-1：倒车入库深度cm（库深实测25cm；实测倒20差4~5cm不到位，改25）
#define TASK2_SUB2_CORNERS	3		//子任务2-2：A点发车绕场经过3个直角（A→B→C→D，到D角），转弯后沿DA走
#define TASK2_DA_DIST_CM	61		//子任务2-2：D角转弯完成后沿DA边走61cm到中点（已实测定稿=61）；运行时可调：暂停态按KEY3调距键（PC14）微调（短按+1cm、长按−1cm），调好把值写回这里重烧
#define TASK2_DA_BACK_TURN_DEG	45	//子任务2-2：右后倒车摆尾角度（车尾向右甩45°斜插进库，左修阶段再摆回0°=车身正——待标定，进库深浅靠它调）
#define TASK2_DA_BACK_FAST	20		//子任务2-2：摆尾快侧轮倒车速度（计数/拍，闭环）：右后倒车=左轮快倒（车尾向右摆）、左修=右轮快倒（车头向右摆回）
#define TASK2_DA_BACK_SLOW	5		//子任务2-2：摆尾慢侧轮倒车速度（计数/拍，闭环），越小摆尾越急（别低于5：太慢那侧轮带不动）
#define TASK2_DA_TURN_STOP_LEAD_DEG	5	//子任务2-2：右后倒车判定提前角（提前5°判定，惯性滑到位）
#define TASK2_DA_BACK_STRAIGHT_DEG	2	//子任务2-2：左修"正了"判定——偏角回到±2°内即车身正，停车
#define TASK2_TURN_DUTY		30		//原地转占空比（左+右-）
#define TASK2_DRIVE_DUTY	15		//直行/倒车目标速度（编码器计数/拍）
#define TASK2_TURN_TIMEOUT_TICKS	1500	//任务2原地转超过15秒没转到位，强制停车兜底

/*---- 陀螺（任务2原地转、任务1转向寻线共用；任务3全程不用陀螺，不受漂移影响） ----*/
#define GYRO_DIR			1		//陀螺方向符号：任务1转向寻线的角速度反馈、OLED第2行Ang显示用它（原地右转应为正，实测反了改成-1）；任务2/3转到位判定已改用角度绝对值，不受此参数影响
/*陀螺零漂已改为启动时自动标定：按PB1启动前车静止在起点，程序采样0.5秒取平均，
  之后每拍按标定值扣除零漂（Ang不会再自己慢慢涨），无需再手填GYRO_Z_BIAS*/

/*---- 任务3：视觉识别巡线（目标=cam串口1/3/5/7或KEY4按键按1~4下；仿任务1/2写法：启动找线→弧线慢转对准→任务1循迹；全程不用陀螺积分判定，不受零漂影响） ----*/
#define TASK3_FIND_DUTY		8		//启动找线直走占空比（灰度00000000=离线1~2cm：两轮同速慢慢直走前进到压线；一定要慢——冲过头就乱拐）
#define TASK3_FIND_TIMEOUT_TICKS	1000	//启动找线超过10秒还没压到线，强制停车兜底
#define TASK3_TURN_OUT_SPEED	12		//弧线慢转外轮目标速度（编码器计数/拍，闭环；边慢慢前进边转弯、外快内慢）
#define TASK3_TURN_IN_SPEED	3		//弧线慢转内轮目标速度（编码器计数/拍，闭环；实测5还是偏快，用户定为3——更慢的内轮让弧线半径更小、车头转得更稳）
#define TASK3_TURN_TIMEOUT_TICKS	2000	//弧线慢转超过20秒还没扫回线，强制停车兜底
#define TASK3_CORNERS_A		2		//A目标：弧线慢转后沿BC巡线，经过2个拐点停（B角左转拐上AB继续走到A角停；对准的转不算拐弯，拐弯从切巡线才开始数）
#define TASK3_CORNERS_B		1		//B目标：到第1个拐点B角停
#define TASK3_CORNERS_C		1		//C目标：到第1个拐点C角停
#define TASK3_CORNERS_D		2		//D目标：经过2个拐点停（C角右转拐上CD继续走到D角停）

#endif
