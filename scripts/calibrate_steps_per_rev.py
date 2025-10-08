#!/usr/bin/env python3
import os
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")  # use pigpio backend

import config as cfg
from encoder import WheelEncoder

def main():
    print("Calibration: Steps per **wheel** revolution")
    enc = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], steps_per_rev=1.0)
    print("1) Zeroing encoder...")
    enc.zero_odometry()
    input("2) Rotate the LEFT wheel by **exactly one full turn** (mark the tire). Then press ENTER...")
    # Read the underlying RotaryEncoder's steps directly (no need to call read())
    steps = abs(int(enc.re.steps))   # abs() in case direction is negative
    print(f"Measured steps for one wheel revolution: {steps}")
    print("Tip: Repeat a few times in both directions and average.")
    print("Put that value into ENCODER_STEPS_PER_REV in config.py")

if __name__ == '__main__':
    main()

