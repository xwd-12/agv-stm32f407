# capture_bg.py — 采集背景图片 (无物体)
# 放到 OpenMV 上运行，自动拍 200 张保存到 SD 卡 /background/

import sensor
import time
import pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

COUNT = 200
INTERVAL_MS = 200  # 每 0.2s 拍一张

uart = pyb.UART(3, 115200)

for i in range(COUNT):
    img = sensor.snapshot()
    fname = "/background/%04d.jpg" % (i + 1)
    img.save(fname)
    uart.write("OK %d/%d\r\n" % (i + 1, COUNT))
    time.sleep_ms(INTERVAL_MS)

uart.write("DONE: %d images saved to /background/\r\n" % COUNT)
