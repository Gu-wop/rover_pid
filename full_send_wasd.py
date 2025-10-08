#!/usr/bin/env python3
# full_send_wasd.py — raw full-speed WASD with arc turning + gentle slew
import os, sys, time, select, termios, tty
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from motor_driver import RoverMotors
from util.rate import Rate

SLEW_PER_STEP = 0.15      # increase for snappier response (0.1–0.25)
LOOP_HZ = 50.0            # control loop
TURN_FRAC = 0.7           # 0..1, how hard to arc at full speed

def clamp(x,a,b): return a if x<a else b if x>b else x
def slew(current, target, max_step):
    d = target - current
    if d >  max_step: d =  max_step
    if d < -max_step: d = -max_step
    return current + d

def read_key():
    dr,_,_ = select.select([sys.stdin], [], [], 0)
    if dr: return sys.stdin.read(1)
    return None

def main():
    print("FULL SEND driver: W forward, S backward, A/D arc turn, X straight, E stop, Q quit")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd); tty.setcbreak(fd)

    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    m.enable()

    # command components
    base_dir = 0     # -1 back, 0 stop, +1 fwd
    turn = 0.0       # -TURN_FRAC .. +TURN_FRAC
    outL = outR = 0.0

    rate = Rate(LOOP_HZ)
    try:
        while True:
            ch = read_key()
            if ch:
                c = ch.lower()
                if c == 'q': print("\nQuit."); break
                elif c == 'e': base_dir = 0; turn = 0.0; print("\n[STOP]")
                elif c == 'w': base_dir = +1; print("\n[BASE] forward")
                elif c == 's': base_dir = -1; print("\n[BASE] backward")
                elif c == 'a': turn = -TURN_FRAC; print("\n[TURN] left")
                elif c == 'd': turn = +TURN_FRAC; print("\n[TURN] right")
                elif c == 'x': turn = 0.0; print("\n[TURN] straight")

            # compose raw targets at FULL speed
            if base_dir == 0:
                tgtL = tgtR = 0.0
            else:
                # arc mix: left = 1 - turn, right = 1 + turn (clamped 0..1), then apply direction
                left_mag  = clamp(1.0 - turn, 0.0, 1.0)
                right_mag = clamp(1.0 + turn, 0.0, 1.0)
                tgtL = base_dir * left_mag
                tgtR = base_dir * right_mag

            # slew-limit to avoid twitch
            outL = slew(outL, tgtL, SLEW_PER_STEP)
            outR = slew(outR, tgtR, SLEW_PER_STEP)

            m.set_speeds(outL, outR)

            print(f"\rbase={base_dir:+d} turn={turn:+.2f}  outL/R={outL:+.2f}/{outR:+.2f}   ",
                  end="", flush=True)
            rate.sleep()
    finally:
        print("\nStopping…")
        m.stop(); m.disable(); m.close()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    main()

