#!/usr/bin/env python3
import os, time
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from motor_driver import RoverMotors
from encoder import WheelEncoder

def test_side(name, motor_side, enc, invert_flag):
    print(f"\n[{name}] Commanding +0.6 for 2s...")
    enc.zero_odometry()
    motor_side(0.6)
    t0=time.time()
    signs=[]
    while time.time()-t0<2.0:
        ds, dt, rps = enc.read()
        signs.append(1 if rps>0 else (-1 if rps<0 else 0))
        time.sleep(0.02)
    motor_side(0.0)
    s=sum(signs)
    if s>0:
        print(f"OK: rps mostly positive.")
    elif s<0:
        print(f"WRONG SIGN: rps mostly NEGATIVE. Set INVERT_{name.upper()}=True in config.py")
    else:
        print("No motion detected? Check wiring/gains.")
    print(f"Sample rps (last): {rps:.2f}")

def main():
    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    m.enable()
    encL = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    encR = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)

    try:
        # Left wheel only
        test_side("left", lambda v: m.set_speeds(v,0.0), encL, cfg.INVERT_LEFT)
        time.sleep(0.8)
        # Right wheel only
        test_side("right", lambda v: m.set_speeds(0.0,v), encR, cfg.INVERT_RIGHT)
    finally:
        m.stop(); m.disable(); m.close()

if __name__=="__main__":
    main()

