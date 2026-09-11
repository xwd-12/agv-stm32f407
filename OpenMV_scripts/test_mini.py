# test_mini.py — v3: 只发不收, 排除 uart.read() 兼容性问题
import sensor, time, pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

uart = pyb.UART(3, 115200)
time.sleep_ms(500)

uart.write("$OK,START\r\n")
time.sleep_ms(300)

uart.write("$OK,BEFORE_TF\r\n")
time.sleep_ms(300)

try:
    import tf
    uart.write("$OK,TF_OK\r\n")
    time.sleep_ms(200)
except:
    uart.write("$ERR,TF_FAIL\r\n")
    time.sleep_ms(200)

while True:
    uart.write("$HB\r\n")
    time.sleep_ms(200)
