# test_ok_only.py — 只发 $OK, 不发 $HB, 测解析器
import time
import pyb

uart = pyb.UART(3, 115200)
time.sleep_ms(3000)
uart.write("\r\n\r\n")
time.sleep_ms(500)

uart.write("$OK,TEST1\r\n")
time.sleep_ms(1000)

uart.write("$OK,TEST2\r\n")
time.sleep_ms(1000)

uart.write("$OK,TEST3\r\n")
time.sleep_ms(1000)

# 空闲, 不发送任何数据
while True:
    time.sleep_ms(1000)
