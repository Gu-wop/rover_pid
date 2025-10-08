#!/usr/bin/env python3
import time
import config as cfg
from encoder import WheelEncoder

def main():
    left = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    right = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    print("Reading encoders. Ctrl+C to stop.")
    t0 = time.time()
    while True:
        dsl, dtl, rps_l = left.read()
        dsr, dtr, rps_r = right.read()
        print(f"t={time.time()-t0:6.2f}s  L: ds={dsl:5d} dt={dtl*1000:6.1f}ms rps={rps_l:5.2f} | R: ds={dsr:5d} dt={dtr*1000:6.1f}ms rps={rps_r:5.2f}", end="\r", flush=True)
        time.sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye")
