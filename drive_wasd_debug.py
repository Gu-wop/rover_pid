#!/usr/bin/env python3
# drive_wasd_debug.py — Closed-loop WASD without curses (debug-friendly)

import os, sys, time, csv, datetime as dt, select, termios, tty
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate

# ===== User knobs =====
FORWARD_RPS   = 2.5    # W/S linear speed
TURN_RPS      = 0.8    # A/D in-place turn speed
ACCEL_RPS_S   = 3.0    # setpoint ramp (rev/s^2)
SLEW_PER_STEP = 0.05   # max PWM change per control step
LOOP_HZ       = cfg.LOOP_HZ  # typically 50

# Very gentle initial gains (increase Kp later)
START_KP = 0.6
START_KI = 0.0
START_KD = 0.0

def clamp(x,a,b): 
    return a if x<a else b if x>b else x

def ramp(current, target, max_delta):
    if target > current: return min(target, current + max_delta)
    if target < current: return max(target, current - max_delta)
    return current

def slew(current, target, max_step):
    delta = clamp(target - current, -max_step, +max_step)
    return current + delta

def make_logger():
    runs="runs"
    os.makedirs(runs, exist_ok=True)
    path=f"{runs}/DRIVE-WASD-DEBUG-{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    f=open(path,"w",newline=""); w=csv.writer(f)
    w.writerow(["t","cmd","setL","setR","rpsL","rpsR","outL","outR","Kp","Ki","Kd"])
    return f,w,path

def read_key_nonblocking():
    # returns one char if available, else None
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        ch = sys.stdin.read(1)
        return ch
    return None

def main():
    print("Starting drive_wasd_debug.py …")
    print("Controls: w/a/s/d to move (hold or tap). 'e' stop. 'q' quit.")
    print("Tip: no curses here; you should see live telemetry below.")
    # Put stdin in raw mode for single-char reads
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    # PIDs
    lp = PID(PIDConfig(Kp=START_KP, Ki=START_KI, Kd=START_KD, out_min=-1.0, out_max=1.0))
    rp = PID(PIDConfig(Kp=START_KP, Ki=START_KI, Kd=START_KD, out_min=-1.0, out_max=1.0))

    # Encoders
    le = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    re = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    le.zero_odometry(); re.zero_odometry()

    # Motors
    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    m.enable()

    # Logging
    logf, logw, logpath = make_logger()
    print(f"Logging to: {logpath}")

    rate = Rate(LOOP_HZ)
    t0 = time.time()

    setL = setR = 0.0
    outL = outR = 0.0
    cmd = "STOP"

    try:
        while True:
            ch = read_key_nonblocking()
            if ch:
                if ch == 'q':
                    print("Quitting…")
                    break
                elif ch == 'e':
                    setL = setR = 0.0
                    lp.reset(); rp.reset()
                    cmd = "STOP"
                    print("[CMD] STOP")
                elif ch in ('w','a','s','d'):
                    if ch == 'w':
                        set_tgtL, set_tgtR = +FORWARD_RPS, +FORWARD_RPS
                        cmd = "W"
                    elif ch == 's':
                        set_tgtL, set_tgtR = -FORWARD_RPS, -FORWARD_RPS
                        cmd = "S"
                    elif ch == 'a':
                        set_tgtL, set_tgtR = -TURN_RPS, +TURN_RPS
                        cmd = "A"
                    elif ch == 'd':
                        set_tgtL, set_tgtR = +TURN_RPS, -TURN_RPS
                        cmd = "D"
                    print(f"[CMD] {cmd} target {set_tgtL:.2f}/{set_tgtR:.2f} rps")
                else:
                    set_tgtL = setL
                    set_tgtR = setR
            else:
                # keep previous target if no new key
                set_tgtL = setL
                set_tgtR = setR

            # Ramp setpoints
            max_step = ACCEL_RPS_S / LOOP_HZ
            setL = ramp(setL, set_tgtL, max_step)
            setR = ramp(setR, set_tgtR, max_step)

            # Read speeds
            dsl, dtl, rpsL = le.read()
            dsr, dtr, rpsR = re.read()

            # PID control (fixed dt is fine at 50 Hz)
            rawL = lp.step(setL, rpsL, cfg.DT)
            rawR = rp.step(setR, rpsR, cfg.DT)

            # Slew-limit PWM
            outL = slew(outL, rawL, SLEW_PER_STEP)
            outR = slew(outR, rawR, SLEW_PER_STEP)

            m.set_speeds(outL, outR)

            t = time.time() - t0
            line = (f"t={t:5.2f}s  {cmd:4} | set {setL:5.2f}/{setR:5.2f} rps  "
                    f"meas {rpsL:5.2f}/{rpsR:5.2f}  out {outL:+.2f}/{outR:+.2f}")
            print(line, end="\r", flush=True)

            logw.writerow([f"{t:.3f}", cmd, f"{setL:.3f}", f"{setR:.3f}",
                           f"{rpsL:.3f}", f"{rpsR:.3f}",
                           f"{outL:.4f}", f"{outR:.4f}",
                           f"{lp.cfg.Kp:.3f}", f"{lp.cfg.Ki:.3f}", f"{lp.cfg.Kd:.3f}"])
            logf.flush()

            rate.sleep()

    finally:
        print("\nStopping…")
        m.stop(); m.disable(); m.close()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        logf.close()

if __name__ == "__main__":
    main()

