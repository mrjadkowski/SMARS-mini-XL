import board
import busio
import time
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility

i2c = busio.I2C(board.SCL, board.SDA)

apds = APDS9960(i2c)
apds.enable_proximity = True
apds.enable_gesture = True
apds.enable_color = True

apds.gesture_proximity_threshold = 255

# Uncomment and set the rotation if depending on how your sensor is mounted.
# apds.rotation = 270 # 270 for CLUE

while True:
    print(apds.proximity)
    print(apds.color_data)
    r, g, b, c = apds.color_data
    lux = colorutility.calculate_lux(r, g, b)
    print(lux)
    time.sleep(1)

