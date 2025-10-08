#!/usr/bin/env python3
# drive_w_nudge.py — Hold W for forward; tap A/D to nudge turn (with decay)
import os, sys, time, csv, datetime as dt, select, termios, tty, math
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate

# ===== Speeds (rev/s of wheel) =====
FORWARD_RPS       = 1.8     # base forward speed while holding 'w'
NUDGE_RPS         = 0.6     # how much turn to add per tap
NUDGE_MAX_RPS     = 1.2     # clamp for accumulated nudge
NUDGE_HALFLIFE_S  = 0.6     # how fast nudge decays (half-life seconds)

# ===== Ramps / Slew =====
ACCEL_RPS_S   = 3.0         # setpoint ramp (rev/s^2)
SLEW_PER_STEP = 0.10        # max PWM change per control iteration

# ===== Feed-forward (tune quickly if needed) =====
# from your data: ~0.6 PWM -> ~2.6 rps => kV ≈ 0.23
kV_L = 0.23; kS_L = 0.12     # static + slope for left
kV_R = 0.23; kS_R = 0.12     # static + slope for right

# ===== PID (trim only—keep gentle, add KD if needed) =====
KP = 0.6; KI = 0.00; KD = 0.05

LOOP_HZ = cfg.LOOP_HZ
DT      = cfg.DT

# ===== Hold 'w' logic =====
# Terminals only send key *presses* (no keyup). We treat 'w' as "held"
# if we received a 'w' press within this grace window:
W_HOLD_GRACE_S = 0.6   # works with OS key-repeat; increase if needed

def clamp(x,a,b): return a if x<a else b if x>b else x
def sgn(x): return 1.0 if x>0 else (-1.0 if x<0 else 0.0)

def ramp(current, target, max_delta):
    if target > current: return min(target, current + max_delta)
    if target < current: return max(target, current - max_delta)
    return current

def slew(current, target, max_step):
    d = target - current
    if d >  max_step: d =  max_step
    if d < -max_step: d = -max_step
    return current + d

def read_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr: return sys.stdin.read(1)
    return None

def make_logger():
    runs="runs"; os.makedirs(runs, exist_ok=True)
    path=f"{runs}/DRIVE-W-NUDGE-{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    f=open(path,"w",newline=""); w=csv.writer(f)
    w.writerow(["t","wHeld","nudge","setL","setR","rpsL","rpsR","uFFL","uFFR","uPIDL","uPIDR","outL","outR"])
    return f,w,path

def main():
    print("Controls: HOLD 'w' = forward | tap 'a'/'d' = nudge left/right | 'e' stop | 'q' quit")
    print("Tip: adjust FORWARD_RPS/NUDGE_RPS or kS/kV if you need more punch/speed.")
    # raw TTY
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    # PIDs
    lp = PID(PIDConfig(Kp=KP, Ki=KI, Kd=KD, out_min=-1.0, out_max=1.0))
    rp = PID(PIDConfig(Kp=KP, Ki=KI, Kd=KD, out_min=-1.0, out_max=1.0))

    # Encoders
    le = WheelEncoder(cfg.LEFT_ENCODER_PINS[0],  cfg.LEFT_ENCODER_PINS[1],  cfg.ENCODER_STEPS_PER_REV)
    re = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    le.zero_odometry(); re.zero_odometry()

    # Motors
    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    m.enable()

    logf, logw, path = make_logger()
    print("Logging to:", path)

    rate = Rate(LOOP_HZ)
    t0 = time.time()

    # State
    last_w_press_time = -1e9
    nudge = 0.0        # +right, -left
    setL=setR=0.0
    outL=outR=0.0

    try:
        while True:
            # key handling
            ch = read_key()
            now = time.time()
            if ch:
                if ch == 'q': print("\nQuit."); break
                elif ch == 'e':  # full stop & reset
                    setL=setR=0.0; nudge=0.0; lp.reset(); rp.reset()
                    print("\n[STOP]")
                elif ch == 'w':
                    last_w_press_time = now
                elif ch == 'a':
                    nudge = clamp(nudge - NUDGE_RPS, -NUDGE_MAX_RPS, NUDGE_MAX_RPS)
                elif ch == 'd':
                    nudge = clamp(nudge + NUDGE_RPS, -NUDGE_MAX_RPS, NUDGE_MAX_RPS)

            # W-hold detection with grace window
            w_held = (now - last_w_press_time) <= W_HOLD_GRACE_S
            base = FORWARD_RPS if w_held else 0.0

            # Nudge exponential decay
            if NUDGE_HALFLIFE_S > 1e-6:
                lam = math.pow(0.5, DT / NUDGE_HALFLIFE_S)  # per-step multiplier
                nudge *= lam

            # Desired per-wheel setpoints
            tgtL = base - nudge
            tgtR = base + nudge

            # Ramp setpoints (smooth accel)
            max_step = ACCEL_RPS_S / LOOP_HZ
            setL = ramp(setL, tgtL, max_step)
            setR = ramp(setR, tgtR, max_step)

            # Measure
            _, _, rpsL = le.read()
            _, _, rpsR = re.read()

            # Feed-forward
            uFFL = 0.0 if abs(setL) < 1e-6 else (kS_L*sgn(setL) + kV_L*setL)
            uFFR = 0.0 if abs(setR) < 1e-6 else (kS_R*sgn(setR) + kV_R*setR)

            # PID trim
            uPIDL = lp.step(setL, rpsL, DT)
            uPIDR = rp.step(setR, rpsR, DT)

            # Combine + slew
            rawL = clamp(uFFL + uPIDL, -1.0, 1.0)
            rawR = clamp(uFFR + uPIDR, -1.0, 1.0)
            outL = slew(outL, rawL, SLEW_PER_STEP)
            outR = slew(outR, rawR, SLEW_PER_STEP)
            m.set_speeds(outL, outR)

            # Telemetry
            t = now - t0
            print(f"\rt={t:5.2f}s wHeld={int(w_held)} nudge={nudge:+.2f} | "
                  f"set {setL:4.2f}/{setR:4.2f}  meas {rpsL:4.2f}/{rpsR:4.2f}  "
                  f"uFF {uFFL:+.2f}/{uFFR:+.2f}  uPID {uPIDL:+.2f}/{uPIDR:+.2f}  "
                  f"out {outL:+.2f}/{outR:+.2f}", end="", flush=True)

            logw.writerow([f"{t:.3f}", int(w_held), f"{nudge:.3f}",
                           f"{setL:.3f}", f"{setR:.3f}",
                           f"{rpsL:.3f}", f"{rpsR:.3f}",
                           f"{uFFL:.4f}", f"{uFFR:.4f}",
                           f"{uPIDL:.4f}", f"{uPIDR:.4f}",
                           f"{outL:.4f}", f"{outR:.4f}"])
            logf.flush()

            rate.sleep()

    finally:
        print("\nStopping…")
        m.stop(); m.disable(); m.close()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        logf.close()

if __name__ == "__main__":
    main()

