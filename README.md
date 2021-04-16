# SMARS-mini-XL
A small wheeled or tracked robot running on Circuit Python

![handheld picture](https://github.com/mrjadkowski/SMARS-mini-XL/blob/main/Media/handheld.jpg?raw=true)

.STL and .STEP files are available in the CAD files folder (probably the most current) and at: https://www.thingiverse.com/thing:4810626

Current hardware BOM: https://docs.google.com/spreadsheets/d/167UUY43PvzWUNg7vgtmYUr31faELwSrEWL3U5X_r1nM/edit?usp=sharing
Current code is running on the 2.x.x hardware setup. For the QT Py version, see the 1.1.0 release

This repository has the 3D models, wiring details, and code for the SMARS mini XL. Currently the code is designed for Circuit Python 6.2.0 running on an Adafruit Feather RP2040. It uses a VL53L0X time-of-flight sensor connected with i2c, am DRV8833 h-bridge motor driver, a small lipo, and a SPST switch.
See the Fritzing diagram for wiring

See the code for the current required libraries; at the moment it requires:

  - adafruit_bus_device
  - adafruit_vl53l0x
  - adafruit_neopixel
  - adafruit_motor
  - adafruit_debouncer

The following parameters can be changed in the setup section of the code to alter the behavior:

Turning parameters:
  - start_turn_distance: the distance from an obstacle that the robot will stop and begin to pivot right
  - turn_time: the amount of time the robot will pivot to the right once started
  - stop_turn_distance: the distance from an obstacle that the robot will stop pivoting and start driving forward again, assuming turn_time has elapsed
  - abort_turn_time: if the robot has not reached stop_turn_distance in this amount of time while pivoting, it will attempt an unstuck maneuver
Obstacle detection parameters:
  - stuck_variance: if the robot moves less than this distance in 1 second, a stuck_counter event is recorded.

2.0.0 uses the NeoPixel to show different states. The color codes are:
  - Green: driving forward
  - Red: obstacle detected and turning
  - Blue: stuck event detected, performing unstuck maneuver
  - Bright white: three consecutive unstuck maneuvers have failed

### Upcoming planned releases:
  - 3.0.0: Addition of solar charging, light-seeking behavior, and sleep logic to the 2.0.0 driving behavior. The end goal is to allow the robot to run perpetually, always trying to keep the battery charged as much as possible.
