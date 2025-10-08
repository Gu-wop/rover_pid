#!/usr/bin/env python3
import os, time, curses, csv, datetime as dt
os.environ.setdefault("GPIOZERO_PIN_FACTORY","pigpio")

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate

# -------- User tuning --------
FORWARD_RPS = 1.0       # W/S linear speed
TURN_RPS    = 0.8       # A/D in-place turning speed
ACCEL_RPS_S = 1.5       # setpoint ramp (rev/s^2)
SLEW_PER_STEP = 0.04    # max PWM change per control step
LOOP_HZ = cfg.LOOP_HZ

# Start with very gentle gains (increase Kp later)
START_KP = 0.6
START_KI = 0.0
START_KD = 0.0

def clamp(x,a,b): return a if x<a else b if x>b else x

def make_logger():
    runs="runs"; os.makedirs(runs, exist_ok=True)
    path=f"{runs}/DRIVE-WASD-{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    f=open(path,"w",newline=""); w=csv.writer(f)
    w.writerow(["t","cmd","setL","setR","rpsL","rpsR","outL","outR","Kp","Ki","Kd"])
    return f,w,path

def key_to_cmd(keys):
    # returns (label, setL, setR)
    if keys.get(ord('w')) and not keys.get(ord('s')):
        return "W", +FORWARD_RPS, +FORWARD_RPS
    if keys.get(ord('s')) and not keys.get(ord('w')):
        return "S", -FORWARD_RPS, -FORWARD_RPS
    if keys.get(ord('a')) and not keys.get(ord('d')):
        return "A", -TURN_RPS, +TURN_RPS
    if keys.get(ord('d')) and not keys.get(ord('a')):
        return "D", +TURN_RPS, -TURN_RPS
    return "STOP", 0.0, 0.0

def ramp(current, target, max_delta):
    if target > current: return min(target, current + max_delta)
    if target < current: return max(target, current - max_delta)
    return current

def slew(current, target, max_step):
    delta = clamp(target - current, -max_step, +max_step)
    return current + delta

def main(stdscr):
    curses.cbreak(); stdscr.nodelay(1); stdscr.keypad(1)

    # PIDs (start gentle)
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
    stdscr.addstr(0, 2, "W/A/S/D to drive (hold to move). e=stop, q=quit")
    stdscr.addstr(1, 2, f"Loop {LOOP_HZ:.0f} Hz | Kp={START_KP:.2f} Ki={START_KI:.2f} Kd={START_KD:.2f}")
    stdscr.addstr(2, 2, f"Logging to {logpath}")

    rate = Rate(LOOP_HZ); t0=time.time()

    setL=setR=0.0
    outL=outR=0.0

    try:
        while True:
            # gather pressed keys
            keys={}
            while True:
                k=stdscr.getch()
                if k==-1: break
                keys[k]=True
            if ord('q') in keys: break
            if ord('e') in keys:
                setL=setR=0.0; lp.reset(); rp.reset()

            # desired command from keys
            cmd, tgtL, tgtR = key_to_cmd(keys)

            # ramp setpoints
            max_step = ACCEL_RPS_S / LOOP_HZ
            setL = ramp(setL, tgtL, max_step)
            setR = ramp(setR, tgtR, max_step)

            # measurements
            dsl, dtl, rpsL = le.read()
            dsr, dtr, rpsR = re.read()

            # control (use actual dt if you want: dtl/dtr; cfg.DT is fine at steady 50Hz)
            rawL = lp.step(setL, rpsL, cfg.DT)
            rawR = rp.step(setR, rpsR, cfg.DT)

            # slew-limit motor outputs (prevents twitch)
            outL = slew(outL, rawL, SLEW_PER_STEP)
            outR = slew(outR, rawR, SLEW_PER_STEP)
            m.set_speeds(outL, outR)

            t=time.time()-t0
            stdscr.addstr(4,2,f"{cmd:4} | set L/R {setL:5.2f}/{setR:5.2f} rps  meas {rpsL:5.2f}/{rpsR:5.2f}  out {outL:+.2f}/{outR:+.2f}   ")
            logw.writerow([f"{t:.3f}", cmd, f"{setL:.3f}", f"{setR:.3f}", f"{rpsL:.3f}", f"{rpsR:.3f}", f"{outL:.4f}", f"{outR:.4f}", f"{lp.cfg.Kp:.3f}", f"{lp.cfg.Ki:.3f}", f"{lp.cfg.Kd:.3f}"])
            logf.flush()

            rate.sleep()
    finally:
        m.stop(); m.disable(); m.close()
        logf.close()
        curses.nocbreak(); stdscr.keypad(0); curses.endwin()

if __name__=="__main__":
    import curses
    curses.wrapper(main)

