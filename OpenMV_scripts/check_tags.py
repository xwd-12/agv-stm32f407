import sensor, image, time, pyb

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

uart = pyb.UART(3, 115200)

def send(msg):
    uart.write(msg + "\r\n")

send("$HB")

while True:
    img = sensor.snapshot()
    tags = img.find_apriltags()
    if tags:
        for t in tags:
            send("$TAG," + str(t.id) + "," + str(t.cx) + "," +
                 str(t.cy) + ",-1,0," + str(t.w))
    send("$HB")
    time.sleep_ms(200)
