# test_led.py — LED 诊断: 不用UART, 只看LED闪烁判断进度
# LED1(红)=阶段进度  LED2(绿)=成功  LED3(蓝)=失败
import sensor
import time
import pyb

LED_R = pyb.LED(1)  # 红
LED_G = pyb.LED(2)  # 绿
LED_B = pyb.LED(3)  # 蓝

# 全部灭→亮一次表示启动
LED_R.off(); LED_G.off(); LED_B.off()
time.sleep_ms(500)
LED_R.on(); LED_G.on(); LED_B.on()
time.sleep_ms(500)
LED_R.off(); LED_G.off(); LED_B.off()
time.sleep_ms(500)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# ===== 阶段1: import tf =====
LED_R.on()  # 红灯亮 = 准备 import tf
time.sleep_ms(300)

try:
    import tf
    LED_R.off()
    LED_G.on()  # 绿灯 = import tf 成功
    time.sleep_ms(1000)
    LED_G.off()
except:
    LED_R.off()
    # 红灯闪3次 = import tf 失败
    for i in range(3):
        LED_B.on(); time.sleep_ms(200)
        LED_B.off(); time.sleep_ms(200)
    # 进入死循环, 蓝灯常亮 = 失败状态
    LED_B.on()
    while True:
        time.sleep_ms(1000)

# ===== 阶段2: 加载模型 =====
LED_R.on()  # 红灯 = 准备加载模型
time.sleep_ms(300)

try:
    import gc
    gc.collect()
    net = tf.load("/sd/model_small.tflite")
    gc.collect()
    LED_R.off()
    LED_G.on()  # 绿灯 = 模型加载成功
    time.sleep_ms(1000)
    LED_G.off()
except:
    LED_R.off()
    # 蓝灯闪5次 = 模型加载失败
    for i in range(5):
        LED_B.on(); time.sleep_ms(200)
        LED_B.off(); time.sleep_ms(200)
    LED_B.on()
    while True:
        time.sleep_ms(1000)

# ===== 成功! 绿蓝交替 =====
while True:
    LED_G.on(); time.sleep_ms(500)
    LED_G.off(); LED_B.on(); time.sleep_ms(500)
    LED_B.off()
