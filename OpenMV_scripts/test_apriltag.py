# test_apriltag.py — 对接码识别测试
# 用途: 验证 AprilTag ID=1 (从车对接码) 能否被识别, 并返回识别结果
# 部署: 复制到 H: 盘改名为 main.py (OpenMV 上电自动运行), 或 OpenMV IDE 直接运行
#
# 输出 (UART3 + 串口打印):
#   $TAG,<id>,<cx>,<cy>,<dist_cm>,<angle>,<w>   识别到标签时
#   $HB                                          每帧心跳 (无标签时)
#   OK,MATCH,<id>                                识别到目标 ID=1 时
#   WARN,WRONG,<id>,<want>                       识别到其他 ID 时

import sensor
import time
import pyb

UART_BAUD = 115200
TARGET_TAG = 1        # 对接码 ID=1
# FOCAL 校准: 15cm 码 @35cm 实测 121px -> FOCAL = 35*121/15 = 282 (纯焦距, 与码尺寸无关)
FOCAL = 282           # 纯焦距 (像素)
TAG_W_CM = 10.0       # 对接码实际宽度 (cm)  -- 已从 15cm 换成 10cm

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)          # APRILTAG 模式: 开自动增益/白平衡
sensor.set_auto_whitebal(True)

uart = pyb.UART(3, UART_BAUD)
led = pyb.LED(1)


def estimate_distance(pixel_w):
    if pixel_w <= 0:
        return -1
    return int(FOCAL * TAG_W_CM / pixel_w)


def send(msg):
    uart.write(msg + "\r\n")
    print(msg)


send("$OK,START,apriltag_test,target=%d" % TARGET_TAG)
time.sleep_ms(200)

while True:
    img = sensor.snapshot()
    tags = img.find_apriltags()

    if tags:
        # 选 decision_margin 最高的标签
        best = max(tags, key=lambda t: t.decision_margin)
        dist = estimate_distance(best.w)
        # 在画面中画框标注 (用 cx/cy/w/h 计算左上角, 避免 rect 属性兼容问题)
        img.draw_rectangle((int(best.cx - best.w / 2),
                            int(best.cy - best.h / 2),
                            best.w, best.h))
        img.draw_string(best.cx, best.cy, str(best.id))

        # 返回识别结果 (与主固件 $TAG 协议一致)
        send("$TAG,%d,%d,%d,%d,%d,%d" % (
            best.id, best.cx, best.cy, dist, 0, best.w))

        # 判断是否为目标对接码
        if best.id == TARGET_TAG:
            send("OK,MATCH,%d,cx=%d,dist=%dcm,margin=%.0f" % (
                best.id, best.cx, dist, best.decision_margin))
            led.on()
        else:
            send("WARN,WRONG,%d,want=%d" % (best.id, TARGET_TAG))
            led.off()
    else:
        led.off()
        send("$HB")

    time.sleep_ms(50)
