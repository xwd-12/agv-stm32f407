# main.py — OpenMV H7 Plus (AprilTag/QR/AI/Color)

import sensor
import time
import pyb
import gc

UART_BAUD = 115200
IMG_W, IMG_H = 320, 240

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)

LED_RED   = pyb.LED(1)
LED_GREEN = pyb.LED(2)
LED_BLUE  = pyb.LED(3)
uart = pyb.UART(3, UART_BAUD)

mode = "QRCODE"
target_tag  = 0
target_color = 1

net = None
labels = []
AI_READY = False
ai_load_tried = False

qr_triggered = False
qr_text = ""


def send(msg):
    uart.write(msg + "\r\n")
    time.sleep_ms(1)


# 原 FOCAL=450: QR/COLOR/放置码(ID2/3/4) 用的原标定 (不乘码宽, 码为原标准尺寸)
FOCAL = 450
# 对接码(ID=1) 标定: 15cm 码 @35cm=121px -> 纯焦距 FOCAL_TAG = 35*121/15 = 282
# (焦距是相机光学属性, 与码尺寸无关; 换 10cm 码后保持 282, 只改 TAG_W_CM)
FOCAL_TAG = 282
TAG_W_CM = 10.0    # 对接码实际宽度 (cm)  -- 已从 15cm 换成 10cm


def estimate_distance(pixel_w):
    """原公式: QR/COLOR/放置码用 (FOCAL=450, 码为标准尺寸)"""
    if pixel_w <= 0:
        return -1
    return int(FOCAL / pixel_w)


def estimate_dist_tag(pixel_w):
    """对接码专用: FOCAL_TAG=282 纯焦距(px), 距离 = 282 * 码宽cm / 像素宽"""
    if pixel_w <= 0:
        return -1
    return int(FOCAL_TAG * TAG_W_CM / pixel_w)


def try_load_ai():
    global net, labels, AI_READY, ai_load_tried
    if AI_READY:
        send("$OK,BOOT")
        return

    gc.collect()
    send("$HB")

    sensor.set_framesize(sensor.QQVGA)
    sensor.skip_frames(time=100)
    gc.collect()

    # tf.load() 不存在于此固件, 直接用 ml.Model
    try:
        import ml
        net = ml.Model("model_mid_f32.tflite", load_to_fb=False)
    except Exception as e:
        send("$ERR,model_load_fail," + str(e)[:60])
        AI_READY = False
        ai_load_tried = False
        sensor.set_framesize(sensor.QVGA)
        return

    gc.collect()

    try:
        labels = [l.strip() for l in open("labels.txt")]
        gc.collect()
    except:
        send("$ERR,labels_read_fail")
        AI_READY = False
        ai_load_tried = False
        sensor.set_framesize(sensor.QVGA)
        return

    AI_READY = True
    ai_load_tried = True
    gc.collect()
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=300)
    send("$OK,BOOT")
    if hasattr(net, 'classify'):
        pyb.LED(1).on()
    if hasattr(net, 'predict'):
        pyb.LED(2).on()
    if hasattr(net, 'forward'):
        pyb.LED(3).on()


cmd_buf = ""


def read_cmds():
    global mode, target_tag, target_color, cmd_buf
    # 逐字节非阻塞读取, 避免 readline() 在缺 \n 时永久阻塞
    while True:
        ch = uart.readchar()
        if ch < 0:
            break
        c = chr(ch)
        if c == "\n" or c == "\r":
            if cmd_buf:
                line = cmd_buf
                cmd_buf = ""
                _process_cmd(line)
        else:
            if len(cmd_buf) < 128:
                cmd_buf += c


def _process_cmd(line):
    global mode, target_tag, target_color
    global _color_filt_cx, _color_filt_cy, _ai_sensor_ready
    line = line.strip()
    if not line.startswith("$CMD,"):
        return
    send("$OK,ECHO")
    parts = line[5:].split(",")
    cmd = parts[0].strip() if len(parts) > 0 else ""

    if cmd == "MODE":
        new_mode = parts[1].strip().upper() if len(parts) > 1 else "IDLE"
        if new_mode in ("IDLE", "APRILTAG", "QRCODE", "AI", "COLOR"):
            if new_mode == "AI":
                if not ai_load_tried:
                    try_load_ai()
                elif AI_READY:
                    send("$OK,BOOT")
            if mode == "AI" and new_mode != "AI":
                sensor.set_framesize(sensor.QVGA)
                sensor.skip_frames(time=200)
                _ai_sensor_ready = False
            if new_mode == "COLOR":
                pass  # 保持自动曝光开启, 深色物体需要
                # 切换颜色目标时重置 EMA 滤波器, 避免旧颜色历史值污染新颜色
                if len(parts) > 2:
                    try:
                        new_c = int(parts[2].strip())
                        if new_c != target_color:
                            _color_filt_cx = -1
                            _color_filt_cy = -1
                    except:
                        pass
            elif mode == "COLOR" and new_mode != "COLOR":
                sensor.set_auto_exposure(True)
            # APRILTAG 模式: 灰度+自动增益开+白平衡关 (实测检测最稳, 退到33cm那次就是这配置;
            # RGB565+白平衡开会 auto 乱跳, 码时隐时现)
            if new_mode == "APRILTAG":
                sensor.set_auto_gain(True)
                sensor.set_auto_whitebal(False)
                sensor.set_auto_exposure(True)
                sensor.set_pixformat(sensor.GRAYSCALE)
                sensor.skip_frames(time=300)
            elif mode == "APRILTAG" and new_mode != "APRILTAG":
                sensor.set_auto_gain(False)
                sensor.set_auto_whitebal(False)
                sensor.set_auto_exposure(True)
                sensor.set_pixformat(sensor.RGB565)
                sensor.skip_frames(time=300)
            mode = new_mode
            if new_mode == "COLOR" and len(parts) > 2:
                try:
                    target_color = int(parts[2].strip())
                except:
                    pass
            send("$OK,MODE," + mode)
            LED_BLUE.off()
            LED_GREEN.off()

    elif cmd == "TAG":
        target_tag = int(parts[1]) if len(parts) > 1 else 0

    elif cmd == "COLOR":
        new_c = int(parts[1]) if len(parts) > 1 else 1
        if new_c != target_color:
            _color_filt_cx = -1
            _color_filt_cy = -1
        target_color = new_c

    elif cmd == "FOCAL":
        global FOCAL
        try:
            FOCAL = int(parts[1].strip())
            send("$OK,FOCAL,%d" % FOCAL)
        except:
            pass

    elif cmd == "THRESH":
        try:
            cid = int(parts[1].strip())
            th = (int(parts[2]), int(parts[3]), int(parts[4]),
                  int(parts[5]), int(parts[6]), int(parts[7]))
            COLOR_THRESHOLDS[cid] = [th]
            send("$OK,THRESH,%d" % cid)
        except:
            pass


def run_apriltag():
    img = sensor.snapshot()
    # 默认 family 就是 TAG36H11, 不要显式传 families=image.TAG36H11 —
    # 固件未 import image 模块, 会抛 "name 'image' isn't defined" → 主循环 catch
    # → mode=IDLE → 只发 $HB 不发 $TAG (实测: vstat 显示 HB=697 TAG=0)
    tags = img.find_apriltags()
    if tags:
        # 优先找目标ID(1): 取第一个匹配的 (备份方式, 稳定; decision_margin 无括号是方法会崩)
        best = None
        for t in tags:
            if target_tag == 0 or t.id == target_tag:
                best = t
                break
        if best is not None:
            dist = estimate_dist_tag(best.w)   # 对接码专用(10cm码, TAG_W_CM=10)
            send("$TAG,%d,%d,%d,%d,%d,%d" % (
                best.id, best.cx, best.cy, dist, 0, best.w))
            send("T:%d,%d" % (best.cx, best.cy))   # LLM tuner 遥测格式
    else:
        send("N")                              # LLM tuner 无目标
    send("$HB")


qr_latch_cnt = 0
qr_hold_cnt = 0
qr_latch_text = ""
qr_latch_cx = 0
qr_latch_cy = 0
qr_latch_w = 0
QR_LATCH_FRAMES = 3
QR_HOLD_FRAMES  = 15
QR_MIN_WIDTH    = 20


def run_qrcode():
    global qr_triggered, qr_text, qr_latch_cnt, qr_latch_text
    global qr_latch_cx, qr_latch_cy, qr_latch_w, qr_hold_cnt

    img = sensor.snapshot()
    img.lens_corr(1.8)
    codes = img.find_qrcodes()

    if codes:
        best_w = 0
        best_code = None
        for code in codes:
            r = code.rect()
            w = r[2]
            if w > best_w:
                best_w = w
                best_code = code
        if best_code is not None:
            r = best_code.rect()
            payload = best_code.payload()
            w = r[2]
            if w >= QR_MIN_WIDTH and payload:
                qr_latch_text = payload
                qr_latch_cx = r[0] + w // 2
                qr_latch_cy = r[1] + r[3] // 2
                qr_latch_w = w
                qr_latch_cnt += 1
                qr_hold_cnt = 0
                if qr_latch_cnt >= QR_LATCH_FRAMES:
                    qr_text = qr_latch_text
                    qr_triggered = True
    else:
        if qr_triggered:
            qr_hold_cnt += 1
            if qr_hold_cnt >= QR_HOLD_FRAMES:
                qr_triggered = False
                qr_latch_cnt = 0
                qr_hold_cnt = 0
        else:
            if qr_latch_cnt > 0:
                qr_latch_cnt -= 1

    if qr_triggered:
        dist = estimate_distance(qr_latch_w)
        send("$QR," + qr_text)
        send("$TAG,0,%d,%d,%d,0,%d" % (qr_latch_cx, qr_latch_cy, dist, qr_latch_w))
    send("$HB")


_ai_err_sent = False
_ai_sensor_ready = False


def run_ai():
    global _ai_err_sent, _ai_sensor_ready
    if not AI_READY:
        send("$HB")
        return

    send("$HB")
    try:
        if not _ai_sensor_ready:
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_windowing(64, 64)
            sensor.skip_frames(time=100)
            _ai_sensor_ready = True
        img = sensor.snapshot()
        scores = net.predict([img])[0].tolist()[0]
        max_val = scores[0]
        max_idx = 0
        for i in range(1, len(scores)):
            if scores[i] > max_val:
                max_val = scores[i]
                max_idx = i
        send("$CLS,%d,%d,%d,%d,%d" % (max_idx, int(max_val * 100),
            int(scores[0]*100), int(scores[1]*100), int(scores[2]*100)))
        _ai_err_sent = False
    except Exception as e:
        if not _ai_err_sent:
            send("$ERR,rt,classify," + str(e)[:40])
            _ai_err_sent = True


COLOR_THRESHOLDS = {
    1: [(10, 100,  15, 127,  15, 127)],  # 深红: L下限10
    2: [(40, 100, -50,  -5, -20,   50)],  # 浅绿: A严格负值(绿), L偏高, 排除红/黄
    3: [(25, 100, -25,  50,  10,  127)],  # 黄: 放宽L/A/B范围, 之前太窄COLOR找不到
}
_GREEN_MIN_PIXELS = 20  # 浅绿偏白, 色块偏小
_color_filt_cx = -1
_color_filt_cy = -1
_COLOR_EMA_A = 0.35
_COLOR_EMA_A_GREEN = 0.15  # 绿色单独加大平滑, 减少cx抖动


def run_color():
    global _color_filt_cx, _color_filt_cy
    img = sensor.snapshot()
    th_list = COLOR_THRESHOLDS.get(target_color, COLOR_THRESHOLDS[1])
    pt = _GREEN_MIN_PIXELS if target_color == 2 else 80
    blobs = img.find_blobs(th_list, pixels_threshold=pt, area_threshold=pt)
    if blobs:
        best = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(best.rect())
        cx, cy = best.cx(), best.cy()
        ema = _COLOR_EMA_A_GREEN if target_color == 2 else _COLOR_EMA_A
        if _color_filt_cx < 0:
            _color_filt_cx, _color_filt_cy = cx, cy
        else:
            _color_filt_cx = int(ema * cx + (1.0 - ema) * _color_filt_cx)
            _color_filt_cy = int(ema * cy + (1.0 - ema) * _color_filt_cy)
        send("$TAG,%d,%d,%d,%d,%d,%d" % (
            target_color, _color_filt_cx, _color_filt_cy,
            estimate_distance(best.w()), 0, best.w()))
    else:
        _color_filt_cx = -1
        _color_filt_cy = -1
    send("$HB")


send("$OK,START")
try_load_ai()

status_tick = 0
_loop_err_sent = False

while True:
    try:
        status_tick += 1
        if status_tick >= 200:
            status_tick = 0
            if AI_READY:
                send("$OK,BOOT")

        read_cmds()

        if mode == "IDLE":
            sensor.snapshot()
            send("$HB")
        elif mode == "APRILTAG":
            LED_BLUE.on(); time.sleep_ms(2); LED_BLUE.off()
            run_apriltag()
        elif mode == "QRCODE":
            LED_GREEN.on(); time.sleep_ms(2); LED_GREEN.off()
            run_qrcode()
        elif mode == "AI":
            LED_RED.on(); time.sleep_ms(2); LED_RED.off()
            run_ai()
        elif mode == "COLOR":
            LED_BLUE.on(); time.sleep_ms(2); LED_BLUE.off()
            run_color()

        time.sleep_ms(8)
        _loop_err_sent = False
    except Exception as e:
        if not _loop_err_sent:
            send("$ERR,rt,loop," + str(e)[:40])
            _loop_err_sent = True
        mode = "IDLE"
        time.sleep_ms(100)
