# this is designed to run on an adafruit QT Py board running Circuit Python 6.1.0
# using a VL53L0X ToF sensor and a DRV8833 motor driver.

import board
import time
import busio
import adafruit_vl53l0x
import pwmio
import touchio
from adafruit_motor import motor

# set up PWM pins for left motor
PWM_PIN_A = board.D7
PWM_PIN_B = board.D8

# set up PWM pins for right motor
PWM_PIN_C = board.D9
PWM_PIN_D = board.D10

# set up left motor
pwm_a = pwmio.PWMOut(PWM_PIN_A, frequency=50)
pwm_b = pwmio.PWMOut(PWM_PIN_B, frequency=50)
leftmotor = motor.DCMotor(pwm_a, pwm_b)

# set up right motor
pwm_c = pwmio.PWMOut(PWM_PIN_C, frequency=50)
pwm_d = pwmio.PWMOut(PWM_PIN_D, frequency=50)
rightmotor = motor.DCMotor(pwm_c, pwm_d)

# initialize i2c and time of flight sensor
i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

# setup touch input
touch = touchio.TouchIn(board.D6)

run = False

while True:
    # invert run value when touched
    if touch.value:
        run = ~run
        time.sleep(.5)
    # if run is true, then drive
    if run:
        # turn right when 100mm or less from an obstacle, until 150mm or less from an obstacle
        while tof.range <= 100:
            while tof.range <= 150:
                print(tof.range)
                leftmotor.throttle = 0.5
                rightmotor.throttle = -0.5
                time.sleep(0.1)
        # drive straight forward when greater than 50mm from an obstacle
        else:
            print(tof.range)
            leftmotor.throttle = 0.5
            rightmotor.throttle = 0.5
            time.sleep(0.1)
    #if run is false, shut off motors
    else:
        leftmotor.throttle = 0
        rightmotor.throttle = 0
