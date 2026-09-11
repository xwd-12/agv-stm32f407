# test_no_sensor.py — 不用摄像头, 纯测 UART + import tf
import time
import pyb
import gc

uart = pyb.UART(3, 115200)
time.sleep_ms(3000)  # 等 boot noise 结束 (旧 main.py 的 sensor 初始化有4秒缓冲)

uart.write("\r\n\r\n")    # 终结残留的无换行噪音
time.sleep_ms(200)

uart.write("$OK,BOOT\r\n")
time.sleep_ms(200)

uart.write("$OK,BEFORE_TF\r\n")
time.sleep_ms(300)

gc.collect()
try:
    import tf
    uart.write("$OK,TF_OK\r\n")
except Exception as e:
    uart.write("$ERR,TF_FAIL\r\n")

# 主循环: 每200ms发心跳
while True:
    uart.write("$HB\r\n")
    pyb.LED(1).toggle()  # 红灯闪烁证明在运行
    time.sleep_ms(200)
