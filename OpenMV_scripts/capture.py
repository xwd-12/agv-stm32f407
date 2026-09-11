# capture.py — 自动拍照采集 (每2秒一张)
# IDE 运行或保存为 main.py 上电自启
#
# 用法:
#   1. 改 CLASS_NAME
#   2. IDE 点 Run → 蓝灯闪3次 → 每秒一张自动拍
#   3. 红灯闪 = 拍好了, 绿灯常亮 = 取景中
#   4. 断电换类

CLASS_NAME = "red_hexagon"

import sensor, time, pyb, os
from pyb import LED

LED_RED  = LED(1)
LED_BLUE = LED(3)
LED_GREEN = LED(2)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=15000)

SAVE_DIR = "/sd/dataset/" + CLASS_NAME

# 检查 SD 卡
try:
    sd = pyb.SDCard()
    os.mount(sd, "/sd")
except:
    pass

try:
    os.listdir("/sd")
    print("SD card OK")
except:
    SAVE_DIR = "dataset/" + CLASS_NAME
    print("No SD card, using internal flash: %s" % SAVE_DIR)

# 创建目录
try:
    os.mkdir("dataset")
except:
    pass
try:
    os.mkdir("/sd/dataset")
except:
    pass
try:
    os.mkdir(SAVE_DIR)
except:
    pass

# 清空旧照片
try:
    for f in os.listdir(SAVE_DIR):
        if f.endswith(".jpg"):
            os.remove(SAVE_DIR + "/" + f)
    print("Cleared old images")
except:
    pass

count = 0
print("Auto-capture every 2s | Saving to: %s" % SAVE_DIR)

# 蓝灯闪3次就绪
for i in range(3):
    LED_BLUE.on()
    time.sleep_ms(150)
    LED_BLUE.off()
    time.sleep_ms(150)

LED_GREEN.on()
last = time.ticks_ms()

TARGET = 200

while count < TARGET:
    img = sensor.snapshot()

    if time.ticks_diff(time.ticks_ms(), last) >= 2000:
        count += 1
        filename = "%s/%04d.jpg" % (SAVE_DIR, count)
        try:
            img.save(filename, quality=95)
            print("%d/%d  OK: %s" % (count, TARGET, filename))
        except Exception as e:
            print("ERR: %s" % str(e))

        LED_GREEN.off()
        LED_RED.on()
        time.sleep_ms(100)
        LED_RED.off()
        LED_GREEN.on()

        last = time.ticks_ms()

    time.sleep_ms(50)

# 拍完: 绿灯灭, 蓝灯常亮
LED_GREEN.off()
LED_BLUE.on()
print("Done! %d images saved." % TARGET)
while True:
    time.sleep_ms(500)
