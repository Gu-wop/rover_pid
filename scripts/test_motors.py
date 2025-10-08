#!/usr/bin/env python3
import time
import config as cfg
from motor_driver import RoverMotors

def main():
    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    print("Enabling driver (SLP HIGH)")
    m.enable()
    time.sleep(0.5)
    print("Both forward @0.6")
    m.set_speeds(0.6, 0.6); time.sleep(1.5)
    print("Left only forward @0.6")
    m.set_speeds(0.6, 0.0); time.sleep(1.2)
    print("Right only forward @0.6")
    m.set_speeds(0.0, 0.6); time.sleep(1.2)
    print("Both backward @0.6")
    m.set_speeds(-0.6, -0.6); time.sleep(1.5)
    print("Arc right (0.6, 0.3)")
    m.set_speeds(0.6, 0.3); time.sleep(1.2)
    print("In-place left (0.6, -0.6)")
    m.set_speeds(0.6, -0.6); time.sleep(1.2)
    print("Stop -> disable")
    m.stop()
    m.disable()
    m.close()
    print("Done.")

if __name__ == "__main__":
    main()
