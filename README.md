# SMARS-mini-XL
A small wheeled or tracked robot running on Circuit Python
.STL and .STEP files are available at: https://www.thingiverse.com/thing:4810626

This repository has the code for the SMARS mini XL. Currently the code is designed for Circuit Python 6.1.0 running on an Adafruit QT Py. 
It uses a VL53L0X time-of-flight sensor connected with i2c, and a DRV8833 h-bridge motor driver.
See the code for the current required libraries; at the moment it requires adafruit_vl53l0x and adafruit_motor

Once they are available I will update the code to run on an Adafruit Feather RP2040, which the chassis is mechaincally designed for.
