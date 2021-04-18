from board import SCL, SDA
import busio
import time
from adafruit_apds9960.apds9960 import APDS9960

i2c = busio.I2C(SCL, SDA)

apds = APDS9960(i2c)
apds.enable_proximity = True
apds.enable_gesture = True
gesture = 0

apds.gesture_gain = 0
apds.gesture_proximity_threshold = 255
apds.enable_proximity_interrupt = False

# Uncomment and set the rotation if depending on how your sensor is mounted.
# apds.rotation = 270 # 270 for CLUE

while True:
    if apds.proximity == 51:
        apds.clear_interrupt()
    proximity = apds.proximity
    print(proximity)
    gesture = apds.gesture()

    if gesture > 0:
        print("gesture: ", gesture)
        gesture = 0
