# main.py — OpenMV 上电自动运行
# 所有功能一个文件, 监听 $CMD 指令切模式

import sensor
import time
import pyb
import os
import math
import gc

UART_BAUD = 115200
IMG_W, IMG_H = 320, 240

# ====== 最小初始化 (仅 UART + sensor 骨架) ======
sensor.reset()
uart = pyb.UART(3, UART_BAUD)

def send(msg):
    uart.write(msg + "\r\n")
    time.sleep_ms(1)

# ====== 立即加载 AI 模型 (堆最干净, 无碎片) ======
AI_READY = False
ai_load_tried = False
net = None
labels = []

def load_ai_now():
    global net, labels, AI_READY, ai_load_tried
    import tf
    import uos

    # Stage 1: 检查文件 (SD卡)
    try:
        f = open("/sd/model_small.tflite", "rb")
        f.close()
        f = open("/sd/labels.txt", "r")
        f.close()
    except Exception as e:
        send("$ERR,file_missing")
        return

    # Stage 2: 加载模型 (用旧API tf.load, 内存友好)
    try:
        gc.collect()
        net = tf.load("/sd/model_small.tflite")
        gc.collect()
    except Exception as e:
        send("$ERR,model_load_fail," + str(e)[:60])
        return

    # Stage 3: 加载标签
    try:
        labels = [l.strip() for l in open("/sd/labels.txt")]
    except Exception as e:
        send("$ERR,labels_fail")
        return

    AI_READY = True
    ai_load_tried = True
    gc.collect()
    send("$OK,BOOT")
    if hasattr(net, 'classify'): pyb.LED(1).on()
    if hasattr(net, 'predict'):  pyb.LED(2).on()
    if hasattr(net, 'forward'):  pyb.LED(3).on()

load_ai_now()

# ====== 完整初始化 ======
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)

LED_RED   = pyb.LED(1)
LED_GREEN = pyb.LED(2)
LED_BLUE  = pyb.LED(3)

mode = "QRCODE"
target_tag  = 0
target_color = 1

qr_triggered = False
qr_text = ""

FOCAL = 450

def estimate_distance(pixel_w):
    if pixel_w <= 0:
        return -1
    return int(FOCAL / pixel_w)


# ====== 尝试加载 (供 vm模式切换时调用) ======
def try_load_ai():
    global AI_READY, ai_load_tried
    if AI_READY:
        send("$OK,BOOT")
        return
    if not ai_load_tried:
        load_ai_now()


# ====== 读取 $CMD 指令 ======
def read_cmds():
    global mode, target_tag, target_color
    while uart.any():
        line = uart.readline().decode().strip()
        if not line.startswith("$CMD,"):
            continue
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
                    global _ai_sensor_ready
                    sensor.set_framesize(sensor.QVGA)
                    sensor.skip_frames(time=200)
                    _ai_sensor_ready = False
                if new_mode == "COLOR":
                    sensor.set_auto_exposure(False)
                elif mode == "COLOR" and new_mode != "COLOR":
                    sensor.set_auto_exposure(True)
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
            target_color = int(parts[1]) if len(parts) > 1 else 1

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


# ====== AprilTag 模式 ======
def run_apriltag():
    global qr_triggered
    img = sensor.snapshot()
    tags = img.find_apriltags()
    if tags:
        best = max(tags, key=lambda t: t.decision_margin)
        if target_tag == 0 or best.id() == target_tag:
            dist = estimate_distance(best.w())
            msg = "$TAG,%d,%d,%d,%d,%d,%d" % (
                best.id(), best.cx(), best.cy(), dist, 0, best.w())
            send(msg)
    send("$HB")


# ====== 二维码模式 ======
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


# ====== AI 分类模式 ======
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
        send("$CLS,%d,%d" % (max_idx, int(max_val * 100)))
        _ai_err_sent = False
    except Exception as e:
        if not _ai_err_sent:
            send("$ERR,rt,classify," + str(e)[:40])
            _ai_err_sent = True


# ====== 颜色追踪模式 ======
COLOR_THRESHOLDS = {
    1: [(30, 100,  15, 127,  15, 127)],
    2: [(20, 100, -80,  20, -50,   80)],
    3: [(40, 100, -10,  30,  20,  100)],
}
_GREEN_MIN_PIXELS = 30
_color_filt_cx = -1
_color_filt_cy = -1
_COLOR_EMA_A = 0.35

def run_color():
    global _color_filt_cx, _color_filt_cy
    img = sensor.snapshot()
    th_list = COLOR_THRESHOLDS.get(target_color, COLOR_THRESHOLDS[1])
    pt = _GREEN_MIN_PIXELS if target_color == 2 else 80
    blobs = img.find_blobs(th_list, pixels_threshold=pt, area_threshold=pt)
    if blobs:
        best = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(best.rect())
        cx = best.cx()
        cy = best.cy()
        if _color_filt_cx < 0:
            _color_filt_cx = cx
            _color_filt_cy = cy
        else:
            _color_filt_cx = int(_COLOR_EMA_A * cx + (1.0 - _COLOR_EMA_A) * _color_filt_cx)
            _color_filt_cy = int(_COLOR_EMA_A * cy + (1.0 - _COLOR_EMA_A) * _color_filt_cy)
        dist = estimate_distance(best.w())
        msg = "$TAG,%d,%d,%d,%d,%d,%d" % (
            target_color, _color_filt_cx, _color_filt_cy, dist, 0, best.w())
        send(msg)
    else:
        _color_filt_cx = -1
        _color_filt_cy = -1
    send("$HB")


# ====== 主循环 ======
print("OpenMV multi-mode ready, default: QRCODE")
send("$OK,START")

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
            LED_BLUE.on()
            time.sleep_ms(2)
            LED_BLUE.off()
            run_apriltag()
        elif mode == "QRCODE":
            LED_GREEN.on()
            time.sleep_ms(2)
            LED_GREEN.off()
            run_qrcode()
        elif mode == "AI":
            LED_RED.on()
            time.sleep_ms(2)
            LED_RED.off()
            run_ai()
        elif mode == "COLOR":
            LED_BLUE.on()
            time.sleep_ms(2)
            LED_BLUE.off()
            run_color()
        time.sleep_ms(8)
        _loop_err_sent = False
    except Exception as e:
        if not _loop_err_sent:
            send("$ERR,rt,loop," + str(e)[:40])
            _loop_err_sent = True
        mode = "IDLE"
        time.sleep_ms(100)
