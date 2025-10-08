#!/usr/bin/env python3
# drive_arc_ff.py — Forward + turning overlay (A/D adds turn; W/S reset to straight)
import os, sys, time, csv, datetime as dt, select, termios, tty
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate

# ===== Speeds (rev/s of wheel) =====
BASE_FWD_RPS   = 1.8      # W sets +base, S sets -base
TURN_MAG_RPS   = 1.0      # A adds -TURN, D adds +TURN

# ===== Ramps / Slew =====
ACCEL_RPS_S    = 3.0
SLEW_PER_STEP  = 0.10

# ===== Feed-forward (from your ~0.6 PWM -> ~2.6 rps) =====
kV_L = 0.23; kS_L = 0.12
kV_R = 0.23; kS_R = 0.12

# ===== PID (trim only) =====
KP = 0.6; KI = 0.00; KD = 0.05

LOOP_HZ = cfg.LOOP_HZ
DT = cfg.DT

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

def make_logger():
    runs="runs"; os.makedirs(runs, exist_ok=True)
    path=f"{runs}/DRIVE-ARC-FF-{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    f=open(path,"w",newline=""); w=csv.writer(f)
    w.writerow(["t","base","turn","setL","setR","rpsL","rpsR","uFFL","uFFR","uPIDL","uPIDR","outL","outR"])
    return f,w,path

def read_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr: return sys.stdin.read(1)
    return None

def main():
    print("Controls: W=forward  S=backward  A=turn-left  D=turn-right  X=straight  E=stop  Q=quit")
    print("Tap W/S to go straight again (they reset any turn).")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd); tty.setcbreak(fd)

    lp = PID(PIDConfig(Kp=KP, Ki=KI, Kd=KD, out_min=-1.0, out_max=1.0))
    rp = PID(PIDConfig(Kp=KP, Ki=KI, Kd=KD, out_min=-1.0, out_max=1.0))

    le = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    re = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    le.zero_odometry(); re.zero_odometry()

    m = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    m.enable()

    logf, logw, path = make_logger()
    print("Logging to:", path)

    rate = Rate(LOOP_HZ); t0=time.time()
    base_tgt = 0.0; turn_tgt = 0.0
    base = 0.0; turn = 0.0
    setL=setR=0.0; outL=outR=0.0

    try:
        while True:
            ch = read_key()
            if ch:
                c = ch.lower()
                if c == 'q': print("\nQuit."); break
                elif c == 'e':
                    base_tgt = turn_tgt = 0.0
                    lp.reset(); rp.reset()
                    print("\n[STOP]")
                elif c == 'w':
                    base_tgt = +BASE_FWD_RPS
                    turn_tgt = 0.0          # <= reset turn on W
                    print("\n[BASE] +", BASE_FWD_RPS, " [TURN reset]")
                elif c == 's':
                    base_tgt = -BASE_FWD_RPS
                    turn_tgt = 0.0          # <= reset turn on S
                    print("\n[BASE] -", BASE_FWD_RPS, " [TURN reset]")
                elif c == 'd':
                    turn_tgt = -TURN_MAG_RPS
                    print("\n[TURN] left", TURN_MAG_RPS)
                elif c == 'a':
                    turn_tgt = +TURN_MAG_RPS
                    print("\n[TURN] right", TURN_MAG_RPS)
                elif c == 'x':
                    turn_tgt = 0.0
                    print("\n[TURN] straight")

            max_step = ACCEL_RPS_S / LOOP_HZ
            base = ramp(base, base_tgt, max_step)
            turn = ramp(turn, turn_tgt, max_step)

            setL = base - turn
            setR = base + turn

            _, _, rpsL = le.read()
            _, _, rpsR = re.read()

            uFFL = 0.0 if abs(setL) < 1e-6 else (kS_L*sgn(setL) + kV_L*setL)
            uFFR = 0.0 if abs(setR) < 1e-6 else (kS_R*sgn(setR) + kV_R*setR)

            uPIDL = lp.step(setL, rpsL, DT)
            uPIDR = rp.step(setR, rpsR, DT)

            rawL = clamp(uFFL + uPIDL, -1.0, 1.0)
            rawR = clamp(uFFR + uPIDR, -1.0, 1.0)
            outL = slew(outL, rawL, SLEW_PER_STEP)
            outR = slew(outR, rawR, SLEW_PER_STEP)

            m.set_speeds(outL, outR)

            t = time.time() - t0
            print(f"\r t={t:5.2f}s  base/turn {base:4.2f}/{turn:4.2f}  "
                  f"setL/R {setL:4.2f}/{setR:4.2f}  "
                  f"rpsL/R {rpsL:4.2f}/{rpsR:4.2f}  "
                  f"out {outL:+.2f}/{outR:+.2f}   ",
                  end="", flush=True)

            logw.writerow([f"{t:.3f}", f"{base:.3f}", f"{turn:.3f}",
                           f"{setL:.3f}", f"{setR:.3f}",
                           f"{rpsL:.3f}", f"{rpsR:.3f}",
                           "", "", f"{outL:.4f}", f"{outR:.4f}"])
            logf.flush()

            rate.sleep()

    finally:
        print("\nStopping…")
        m.stop(); m.disable(); m.close()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        logf.close()

if __name__ == "__main__":
    main()

