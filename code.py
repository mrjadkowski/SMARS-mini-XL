# this is designed to run on an adafruit Feather RP2040 board running Circuit Python 6.2.0
# using a VL53L0X ToF sensor, VCNL-4040 proximity and light sensor, and a DRV8833 motor driver.

import board
import time
import busio
import adafruit_vl53l0x
import pwmio
import neopixel
import analogio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_debouncer import Debouncer
from adafruit_motor import motor
import adafruit_vcnl4040

# set up PWM pins for left motor
PWM_PIN_A = board.D12  # yellow wire
PWM_PIN_B = board.D11  # green wire

# set up PWM pins for right motor
PWM_PIN_C = board.D10  # white wire
PWM_PIN_D = board.D9  # blue wire

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

# initialize VCNL-4040
sensor = adafruit_vcnl4040.VCNL4040(i2c)

# initialize analog light sensors
light_front_right = analogio.AnalogIn(board.A2)
light_front_left = analogio.AnalogIn(board.A1)
light_rear = analogio.AnalogIn(board.A3)

# initialize NeoPixel
neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

# setup switch input
switch = DigitalInOut(board.D24)
switch.direction = Direction.INPUT
switch.pull = Pull.DOWN
switch_debounced = Debouncer(switch)

# set up turn and drive parameters
start_turn_distance = 5
stop_turn_distance = 3
turn_time = 1.5
abort_turn_time = 5
start_turn_time = -1
turn_flag = False
run_flag = False
turn_right_flag = True

# set up stuck decetion
stuck_check_last = 0
stuck_timer = 0
stuck_counter = 0
stuck_sequential_counter = 0
stuck_range = tof.range
stuck_variance = 10
stuck_flag = False

# set up light detection
last_lux = 0

while True:
    now = time.monotonic()

    # update light data, calcuate lux
    lux = sensor.lux

    # set run_flag to true if switch was switched on
    switch_debounced.update()
    if switch_debounced.rose:
        run_flag = True
    # set run_flag to false of switch was switched off
    if switch_debounced.fell:
        run_flag = False
        neopixel.fill((0, 0, 0))

    # when start_turn_distance or less from an obstacle, set turn flag
    if sensor.proximity >= start_turn_distance:
        turn_flag = True

    # if c > 600:
        # run_flag = False

    # if run_flag is true, then drive
    if run_flag:
        # if stuck for more than 20 seconds, stop driving
        if stuck_sequential_counter >= 3:
            run_flag = False
            neopixel.fill((255, 255, 255))

        elif stuck_flag:
            neopixel.fill((0, 0, 10))
            # back up for one seconds
            if now <= stuck_timer + 1.0:
                leftmotor.throttle = -0.5
                rightmotor.throttle = -0.5

            # turn left for two seconds
            elif now <= stuck_timer + 3.0:
                leftmotor.throttle = -0.5
                rightmotor.throttle = 0.5

            # go back to driving
            else:
                stuck_flag = False
                turn_flag = False
                start_turn_time = now
                stuck_counter = 0
                stuck_sequential_counter += 1

        elif turn_flag:
            neopixel.fill((10, 0, 0))

            # set stuck_flag if attempting to turn for more than abort_turn_time
            if now > abort_turn_time + start_turn_time:
                stuck_flag = True
                stuck_timer = now
                turn_flag = False

            # turn right for turn_time or until stop_turn_distance or more from an obstacle
            elif sensor.proximity > stop_turn_distance or now <= start_turn_time + turn_time:
                leftmotor.throttle = 0.5
                rightmotor.throttle = -0.5

            else:
                turn_flag = False
                print(turn_right_flag)
                turn_right_flag = not turn_right_flag
                start_turn_time = now

        # drive straight forward seeking light when greater than start_turn_distance from an obstacle
        else:
            neopixel.fill((0, 10, 0))
            if lux < last_lux:
                turn_right_flag = not turn_right_flag

            if turn_right_flag:
                leftmotor.throttle = 0.7
                rightmotor.throttle = 0.3

            if not turn_right_flag:
                leftmotor.throttle = 0.3
                rightmotor.throttle = 0.7

            # forward stuck logic
            # perform stuck check once every second
            if now >= stuck_check_last + 1:
                stuck_check_last = now

                # reset stuck_counter and stuck_sequential_counter if not stuck
                if tof.range > stuck_range + stuck_variance or tof.range >= 8000:
                    stuck_counter = 0
                    stuck_sequential_counter = 0

                # increment stuck_counter if stuck
                if tof.range <= stuck_range + stuck_variance and tof.range < 8000:
                    stuck_counter += 1

                # reset stuck_range for next check
                stuck_range = tof.range

                # set stuck_flag to True if stuck for seven consecutive seconds, increment stuck_sequential_counter
                if stuck_counter >= 7:
                    stuck_flag = True
                    stuck_timer = now

            start_turn_time = now
            last_lux = lux

    # if run_flag is false, shut off motors and reset flags
    else:
        leftmotor.throttle = 0
        rightmotor.throttle = 0
        turn_flag = False
        stuck_flag = False
        turn_right_flag = True
        stuck_counter = 0
        stuck_sequential_counter = 0
