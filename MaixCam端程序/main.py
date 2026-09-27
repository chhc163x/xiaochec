# ============================================================================
#  MaixCAM 端程序 · 罗马数字识别 → 串口发给 STM32（竞赛巡线小车 任务三 专用）
#  配套 STM32 工程：桌面\新建文件夹 (2)\10-2 硬件I2C读写MPU6050（固件 V1，任务三为
#                  "启动找线 → 弧线慢转对准 → 复用任务1循迹"版本，全程不用陀螺）
#  模型：model_319768.maixcam.zip（MaixHub 训练，分类模型，输入 224×224，val_acc 95.7%）
# ============================================================================
#
#  【怎么用】
#    ⚠️ 最容易踩的坑：MaixVision 里"打开本地的 main.py"只会把**这一个 .py 文件**传到设备上
#       （放到 /tmp/maixpy_run/ 去跑），**模型文件不会跟着上去**，于是报
#       "[E] model path ... not exists" / "RuntimeError: Not found: load model failed"。
#       所以模型必须**单独传一次**到设备上，传完就一直在设备里了，以后只改 main.py 不用重传。
#
#    第 1 步（只需做一次）：把这两个模型文件传到设备的 /root/models/319768/ 目录下
#        model_319768.mud
#        model_319768.cvimodel          ← 两个必须在同一个目录！
#      用 MaixVision 右侧【设备文件管理器】：进 /root/models/ → 新建文件夹 319768 → 把两个文件拖进去
#
#    第 2 步：MaixVision 左下角"连接"连上 MaixCAM
#       （连接后设备屏幕会黑掉，这是正常的：IDE 把开机的应用选择界面退出了好把屏幕让给程序；
#        断开连接屏幕自动回来，不用管）
#
#    第 3 步：打开本文件 main.py，点左下角"▶ 播放"运行
#       启动日志里应该能看到一行 "找到模型: /root/models/319768/model_319768.mud"
#
#    第 4 步：屏幕出现摄像头画面 + 识别结果；卡片放正后约 0.5 秒显示"SEND 3"并开始发串口
#
#    第 5 步（上车）：把车停成 2-1 倒车入库的状态（灰度压着 BC 线），再按 PB1 启动任务三
#
#  【接线】（交叉 + 共地，缺一不可）
#    MaixCam 串口 TX  →  STM32 PA10（USART1_RX）
#    MaixCam 串口 RX  →  STM32 PA9 （USART1_TX）   ← 本程序不接收，RXD 可以不接
#    MaixCam GND      →  STM32 GND                 ← 共地是铁律，不接必收不到
#    MaixCam 供电      →  Type-C 5V/2A 或充电宝（自己单独供电）
#    ❌ 千万不要从 STM32 的 3.3V 给 MaixCam 供电：MaixCam 要 5V、1A 以上，
#       3.3V 带不动，现象是 MaixCam 反复重启 / 屏幕黑 / 串口发一半就断。
#       （文档里"MaixCam 用 3.3V 供电"那句是旧版留下的笔误，以本文件为准）
#
#  【协议】USART1 115200 8N1（和 STM32 端 MaixCam.c 约定好的）
#    每 SEND_MS 毫秒发一行 + 换行，一共 5 种内容：
#        识别到卡片 3  → 发 "3"      （4 种：1 / 3 / 5 / 7）
#        没识别到卡片  → 发 "0"      ← 心跳，告诉 STM32"我在线，只是没看到卡"
#    STM32 端 MaixCam_Poll 遇到 \n 解析这一行，映射：
#        1 → A 角      3 → B 角      5 → C 角      7 → D 角
#        0 → "相机在线但没识别到"（不触发任务三，只让 OLED 显示 Cam:--）
#    为什么发数字不发罗马数字：数字是**单字节**，串口丢半个字节也不会"错认成另一个角"；
#    而 VII 万一被截成 VI，STM32 的长短优先匹配会把它认成 V（C 角）——风险大得多。
#    为什么"0"要单独规定：STM32 那边只认"整行只有一个 0"才算心跳，
#    这样电源日志里的 "ttyS0" 之类的杂串不会把心跳骗出来。
#
#  【为什么必须"连续几帧都一样"才发】
#    这是分类模型，不是检测模型——画面里没有卡片时它也会硬选一个最像的类别输出。
#    所以这里加了双重保险：置信度 ≥ CONF_MIN 且连续 STABLE_N 帧结果一致，才算识别成立。
#    少一层都会出现"对着空气也能识别出数字"的误触发，车会跑到错误的角。
# ============================================================================

import os
from maix import camera, display, image, nn, app, uart, time

# ---------------------------- 可调参数 ----------------------------
SERIAL_DEV  = "/dev/ttyS0"      # 板载 4P 串口座子默认就是它（对应引脚 A16/A17）
                                # 如果被系统日志干扰严重，可改 "/dev/ttyS1" 并把杜邦线挪到 A19/A18
SERIAL_BAUD = 115200            # 必须和 STM32 的 MaixCam_Init 一致，别改

CONF_MIN    = 0.85              # 置信度阈值：低于它一律当作"没看到卡片"（0.0~1.0）
                                # 【为什么从 0.70 提到 0.85】实测发现：画面里没卡片时，
                                # 分类模型也能"很有信心地"指认某张卡（因为训练集里只有卡片、
                                # 没有"背景"这一类），0.70 挡不住，STM32 就会收到假目标。
                                # 真卡放正后置信度一般都在 0.95 以上，0.85 不会误伤。
                                # 如果真卡也常常认不出（屏幕一直是 NO CARD），就降回 0.75
STABLE_N    = 8                 # 连续 N 帧结果一致才算数：防抖、防一帧误判
                                # 【为什么从 5 提到 8】同上，卡越少不够、多凑几帧更稳。
                                # 这会让识别慢一点点（约 0.2 秒），对手动摆卡完全没感觉

SEND_MS     = 500               # 识别成立后每隔多少毫秒重发一次
                                # 重发是故意的：STM32 每次按 PB1 重跑任务三都会取一次最新结果，
                                # 只发一次的话第二次跑任务三就收不到了。
                                # 注意：STM32 的 Task3_Init 会先清空串口缓存（MaixCam_Flush），
                                # 所以"按 PB1 之前"发的旧消息不会误触发，按完 PB1 后 0.5 秒内收到新的。

# 模型标签 → 发给 STM32 的目标号
# 训练时标签名就是这几个中文，所以按"标签文本"映射而不是按索引号——
# 以后重新训练换了类别顺序也不会发错角。
LABEL_TO_TARGET = {
    "罗马数字1": "1",     # → A 角
    "罗马数字3": "3",     # → B 角
    "罗马数字5": "5",     # → C 角
    "罗马数字7": "7",     # → D 角
}

# ---------------------------- 初始化 ----------------------------
# 【模型文件放哪】两个文件必须永远待在同一个目录里：
#     model_319768.mud    —— 元数据（写明类别名）
#     model_319768.cvimodel —— 模型权重
# 因为 .mud 里写的是相对路径 model = model_319768.cvimodel，运行时是按 .mud 所在目录去找权重的，
# 所以拆开放就会报 "load model failed"。
# 下面自动依次找这几个位置（找到哪个用哪个，并把结果打印出来）：
#   ① 和 main.py 同一个目录（把整个文件夹传到设备、从设备上跑 main.py 时就是这种）
#   ② /root/models/319768/   ← **最推荐**，也是 MaixHub 官方约定；无论 main.py 在哪儿跑都能找到
#   ③ /root/models/ 和 /root/  （图省事直接丢根目录也行）
MODEL_FILE = "model_319768.mud"
CANDIDATES = [
    MODEL_FILE,
    os.path.join(os.getcwd(), MODEL_FILE),
    "/root/models/319768/" + MODEL_FILE,
    "/root/models/" + MODEL_FILE,
    "/root/" + MODEL_FILE,
]

model_path = None
for _p in CANDIDATES:
    if os.path.exists(_p):
        model_path = _p
        break

if model_path is None:
    # 找不到就别说一堆看不懂的英文报错，直接把要做的事写清楚
    print("=" * 60)
    print("[错误] 找不到模型文件 %s" % MODEL_FILE)
    print("")
    print("请把 MaixCam端程序 文件夹里的这两个文件一起传到设备：")
    print("    model_319768.mud")
    print("    model_319768.cvimodel")
    print("放到设备的 /root/models/319768/ 目录下（两个文件必须在同一个目录）。")
    print("")
    print("用 MaixVision 的【设备文件管理器】传最方便：")
    print("  1. 进到 /root/models/ ，右键新建文件夹 319768")
    print("  2. 把上面两个文件拖进去")
    print("  3. 回来重新点播放")
    print("")
    print("已经试过这些位置，都没有：")
    for _p in CANDIDATES:
        print("    -", _p)
    print("=" * 60)
    raise SystemExit(1)

print("找到模型:", model_path)

classifier = nn.Classifier(model=model_path)
cam  = camera.Camera(classifier.input_width(), classifier.input_height(), classifier.input_format())
disp = display.Display()
dev  = uart.UART(SERIAL_DEV, SERIAL_BAUD)

# 开机打印一次：串口助手自查时能确认设备认到的是哪几个类别、发的什么
print("模型类别:", classifier.labels)
print("串口:", SERIAL_DEV, SERIAL_BAUD, " 发送内容:", sorted(LABEL_TO_TARGET.values()))

last_label = ""     # 上一帧的标签
same_cnt   = 0      # 已经连续多少帧和上一帧一样
last_send  = 0      # 上次发送的时刻（ms，开机计时）

while not app.need_exit():
    img = cam.read()
    res = classifier.classify(img)
    idx, prob = res[0]                      # 分类结果按置信度从高到低排，res[0] 就是最高那个
    label = classifier.labels[idx]

    # ---- 双重保险：置信度够 + 连续帧一致 ----
    if prob >= CONF_MIN and label == last_label:
        same_cnt += 1                       # 和上一帧一致，累计
    else:
        last_label = label                  # 结果变了（或这一帧不合格）：重新开始数
        same_cnt = 1 if prob >= CONF_MIN else 0

    confirmed = same_cnt >= STABLE_N        # 连续帧数够了才算真的识别到
    target = LABEL_TO_TARGET.get(label, "") # 标签不在映射表里（比如以后加了新类别）→ 空串，不会误发

    # ---- 屏幕提示：调试时一眼就能看出现在为什么不发 ----
    if confirmed and target:
        img.draw_string(8, 8, "SEND %s" % target, image.COLOR_RED, scale=2)
    elif prob < CONF_MIN:
        img.draw_string(8, 8, "NO CARD %.2f" % prob, image.COLOR_YELLOW, scale=2)
    else:
        img.draw_string(8, 8, "WAIT %.2f" % prob, image.COLOR_YELLOW, scale=2)
    img.draw_string(8, 46, "%s %d/%d" % (label, same_cnt, STABLE_N), image.COLOR_GREEN)

    # ---- 按固定间隔发一帧：识别到发卡号，没识别到发 "0" ----
    # 【为什么没识别到也要发】不做这件事的话，STM32 那边分不清两种"屏幕上没数字"：
    #     相机在线但没看到卡  ／  相机根本没跑、线没通
    #   两种情况在 STM32 看来都是"没有消息"，显示会一样。发个 "0" 当心跳，
    #   STM32 就能显示 "Cam:--"（相机在线，只是在看着没卡），和"完全没连上"区分开。
    now = time.ticks_ms()                   # 用 ticks_ms 而不是 time_ms：后者会被 NTP 对时跳变
    if (now - last_send) >= SEND_MS:
        if confirmed and target:
            dev.write_str(target + "\n")    # 识别到：发卡片号码 1 / 3 / 5 / 7
        else:
            dev.write_str("0\n")            # 没识别到：发 0（心跳）
        last_send = now

    img = img.resize(disp.width(), disp.height(), image.Fit.FIT_CONTAIN)
    disp.show(img)
