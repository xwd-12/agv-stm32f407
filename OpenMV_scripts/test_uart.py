# test_uart.py — 最简测试: 验证 P4/P5 是否有输出
import pyb, time
uart = pyb.UART(3, 115200)
led = pyb.LED(1)

while True:
    led.toggle()
    uart.write("$HB\r\n")
    time.sleep_ms(500)
