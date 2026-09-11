# cal_apriltag.py — AprilTag 距离校准
# 用法: 把已知尺寸的码放到已知距离, 脚本实时显示像素宽, 用公式反推 FOCAL
#
# FOCAL = 实际距离(cm) x 码宽(cm) / 像素宽(px)
#
# 输出:
#   $CAL,<id>,<cx>,<cy>,<w_px>,<h_px>,<suggested_FOCAL>
#   每帧打印建议 FOCAL, 多次测量取平均更准

import sensor
import time
import pyb

UART_BAUD = 115200
TAG_W_CM = 10.0      # <-- 改成你码的实际宽度 (cm)  -- 已换 10cm 码
TAG_DIST_CM = 35.0   # <-- 改成码到摄像头的实际距离 (cm)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)

uart = pyb.UART(3, UART_BAUD)
led = pyb.LED(1)


def send(msg):
    uart.write(msg + "\r\n")
    print(msg)


send("$OK,CAL,start,tag_w=%.0fcm,dist=%.0fcm" % (TAG_W_CM, TAG_DIST_CM))
time.sleep_ms(200)

while True:
    img = sensor.snapshot()
    tags = img.find_apriltags()

    if tags:
        best = max(tags, key=lambda t: t.decision_margin)
        w_px = best.w
        if w_px > 0:
            # FOCAL = dist(cm) * w(px) / tag_w(cm)
            focal = int(TAG_DIST_CM * w_px / TAG_W_CM)
            send("$CAL,%d,cx=%d,cy=%d,w=%dpx,dist=%.0fcm,FOCAL=%d" % (
                best.id, best.cx, best.cy, w_px, TAG_DIST_CM, focal))
            led.on()
        else:
            led.off()
    else:
        led.off()
        send("$HB")

    time.sleep_ms(80)
