import board
import time
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    print(tof.range)
    time.sleep(1)
