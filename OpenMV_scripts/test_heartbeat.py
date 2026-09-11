# test_heartbeat.py — 只发心跳, 不 import tf, 不用摄像头
import time
import pyb

uart = pyb.UART(3, 115200)
time.sleep_ms(3000)

uart.write("\r\n\r\n")
time.sleep_ms(500)

uart.write("$OK,ALIVE\r\n")
time.sleep_ms(200)

while True:
    uart.write("$HB\r\n")
    pyb.LED(1).toggle()
    # 排空 STM32 发来的数据, 防止 RX 缓冲区溢出导致死机
    while uart.any():
        uart.readchar()
    time.sleep_ms(200)
