# SMARS-mini-XL
A small wheeled or tracked robot running on Circuit Python


.STL and .STEP files are available at: https://www.thingiverse.com/thing:4810626

Current hardware BOM: https://docs.google.com/spreadsheets/d/167UUY43PvzWUNg7vgtmYUr31faELwSrEWL3U5X_r1nM/edit?usp=sharing

This repository has the code for the SMARS mini XL. Currently the code is designed for Circuit Python 6.1.0 running on an Adafruit QT Py. It uses a VL53L0X time-of-flight sensor connected with i2c, and a DRV8833 h-bridge motor driver. There is also an optional lipo power and charging circuit using an Adafruit Pro Trinket backpack, a SPST switch, and a power diode. This allows the lipo to charge from the QT Py USB connector, or power the robot when the USB is unplugged, depending on the switch position.

See the Fritzing diagram for wiring

See the code for the current required libraries; at the moment it requires:

  -  adafruit_vl53l0x
  - adafruit_motor
  - adafruit_debouncer

The following parameters can be changed in the setup section of the code to alter the behavior:

Turning parameters:
  - start_turn_distance: the distance from an obstacle that the robot will stop and begin to pivot right
  - turn_time: the amount of time the robot will pivot to the right once started
  - stop_turn_distance: the distance from an obstacle that the robot will stop pivoting and start driving forward again, assuming turn_time has elapsed
  - abort_turn_time: if the robot has not reached stop_turn_distance in this amount of time while pivoting, it will stop trying to drive

Obstacle detection parameters:
  - stuck_variance: if the robot moves less than this distance in 1 second, a stuck_counter event is recorded.
