# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import analogio

light_front_right = analogio.AnalogIn(board.A2)
light_front_left = analogio.AnalogIn(board.A1)
light_rear = analogio.AnalogIn(board.A3)

while True:
    print("Front Right: ", light_front_right)
    print("Front Left: ", light_front_left)
    print("Rear: ", light_rear)
    time.sleep(1)

