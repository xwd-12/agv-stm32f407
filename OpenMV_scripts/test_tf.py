# test_tf.py — 最小诊断 (v2: 加UART排空和延迟)
import sensor
import time
import pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

uart = pyb.UART(3, 115200)

# 排空启动时STM32发来的残留数据
time.sleep_ms(500)
while uart.any():
    uart.readchar()

def send(msg):
    uart.write(msg + "\r\n")
    time.sleep_ms(10)  # 确保TX完成

send("$OK,START")
time.sleep_ms(100)
send("$OK,START2")  # 双重确认
time.sleep_ms(100)

# 排空UART RX后再继续
while uart.any():
    uart.readchar()

# --- 阶段1: import tf ---
send("$OK,S1_IMPORT")
time.sleep_ms(50)
try:
    import tf
    send("$OK,S1_OK")
except Exception as e:
    send("$ERR,S1_FAIL")
    # 即使 import tf 失败, 继续后面的测试

time.sleep_ms(100)
while uart.any():
    uart.readchar()

# --- 阶段2: 列出SD卡 ---
send("$OK,S2_SD")
time.sleep_ms(50)
try:
    import os
    files = os.listdir("/sd")
    send("$OK,S2_OK")
except Exception as e:
    send("$ERR,S2_FAIL")

time.sleep_ms(100)
while uart.any():
    uart.readchar()

# --- 阶段3: 加载小模型 ---
send("$OK,S3_LOAD")
time.sleep_ms(50)
try:
    import gc
    gc.collect()
    net = tf.load("/sd/model_small.tflite")
    gc.collect()
    send("$OK,S3_OK")
except Exception as e:
    send("$ERR,S3_FAIL")

# 进入空闲循环
while True:
    send("$HB")
    # 排空STM32发来的指令, 避免缓冲区溢出
    while uart.any():
        uart.readchar()
    time.sleep_ms(100)
