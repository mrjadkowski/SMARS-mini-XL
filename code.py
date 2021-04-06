import board
import time
import busio
import adafruit_vl53l0x
import pwmio
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

while True:
    print(tof.range)
    time.sleep(1)
