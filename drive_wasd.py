#!/usr/bin/env python3
# drive_wasd.py — Closed-loop WASD driver (accurate, straight, repeatable)
import os, time, curses, datetime as dt, csv

# Prefer pigpio backend for solid edge timing
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "pigpio")

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate
from battery_monitor import BatteryMonitor, BatteryConfig

# --- User-tunable constants ---
BASE_RPS_FORWARD = 1.2   # forward/back linear speed (rev/s of wheel)
TURN_RPS         = 0.9   # differential speed for turning in place (rev/s)
LOOP_HZ          = cfg.LOOP_HZ  # 50 Hz by default
SHOW_BATTERY     = True  # set False if you don't have MCP3008 wired

def make_logger():
    runs_dir = "runs"; os.makedirs(runs_dir, exist_ok=True)
    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = f"{runs_dir}/DRIVE-{ts}.csv"
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t","mode","setL","setR","rpsL","rpsR","outL","outR","Kp","Ki","Kd","Vbat"])
    return f, w, path

def drive_mode_from_key(key: int):
    """
    Returns (label, set_left_rps, set_right_rps) for WASD.
    'w' = forward, 's' = backward, 'a' = left in place, 'd' = right in place.
    Any other key => stop/hold.
    """
    if key == ord('w'):
        return "FWD",  +BASE_RPS_FORWARD, +BASE_RPS_FORWARD
    if key == ord('s'):
        return "BACK", -BASE_RPS_FORWARD, -BASE_RPS_FORWARD
    if key == ord('a'):
        return "TURNL", -TURN_RPS, +TURN_RPS
    if key == ord('d'):
        return "TURNR", +TURN_RPS, -TURN_RPS
    return "STOP", 0.0, 0.0

def main(stdscr):
    curses.cbreak()
    stdscr.nodelay(1)
    stdscr.keypad(1)

    stdscr.addstr(0, 2, "W/A/S/D drive (closed-loop). 'e' stop, 'q' quit. (Kp/Ki/Kd from config.py)")
    stdscr.addstr(1, 2, f"Forward={BASE_RPS_FORWARD:.2f} rps, Turn={TURN_RPS:.2f} rps, Loop={LOOP_HZ:.0f} Hz")

    # PID per wheel
    left_pid  = PID(PIDConfig(**cfg.LEFT_PID))
    right_pid = PID(PIDConfig(**cfg.RIGHT_PID))

    # Encoders
    left_enc  = WheelEncoder(cfg.LEFT_ENCODER_PINS[0],  cfg.LEFT_ENCODER_PINS[1],  cfg.ENCODER_STEPS_PER_REV)
    right_enc = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    left_enc.zero_odometry(); right_enc.zero_odometry()

    # Motors
    motors = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS,
                         cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    motors.enable()

    # Battery (optional)
    batt = BatteryMonitor(BatteryConfig(adc_channel=cfg.ADC_CHANNEL, vref=cfg.VREF, divider_scale=cfg.DIVIDER_SCALE))

    # Logger
    log_f, log_w, log_path = make_logger()
    stdscr.addstr(2, 2, f"Logging to {log_path}")

    rate = Rate(LOOP_HZ)
    t0 = time.time()
    last_mode = "STOP"
    setL = setR = 0.0

    try:
        while True:
            # --- Input ---
            k = stdscr.getch()
            if k == ord('q'):
                break
            elif k == ord('e'):
                last_mode, setL, setR = "STOP", 0.0, 0.0
                left_pid.reset(); right_pid.reset()
            elif k in (ord('w'), ord('a'), ord('s'), ord('d')):
                last_mode, setL, setR = drive_mode_from_key(k)
            # no key pressed => hold last command (keeps moving until 'e' or another key)

            # --- Measurements ---
            dsl, dtl, rps_l = left_enc.read()
            dsr, dtr, rps_r = right_enc.read()

            # --- Control ---
            out_l = left_pid.step(setL, rps_l, cfg.DT)
            out_r = right_pid.step(setR, rps_r, cfg.DT)
            motors.set_speeds(out_l, out_r)

            # --- Telemetry ---
            t = time.time() - t0
            vb = batt.read_vin() if SHOW_BATTERY else None

            stdscr.addstr(4, 2, f"Mode: {last_mode:5} | Set L/R: {setL:5.2f}/{setR:5.2f} rps   ")
            stdscr.addstr(5, 2, f"Meas L/R: {rps_l:5.2f}/{rps_r:5.2f} rps | PWM out L/R: {out_l:+.2f}/{out_r:+.2f}   ")
            stdscr.addstr(6, 2, f"Gains: Kp={left_pid.cfg.Kp:.2f} Ki={left_pid.cfg.Ki:.3f} Kd={left_pid.cfg.Kd:.2f}      ")
            if vb is not None:
                stdscr.addstr(7, 2, f"Battery: {vb:5.2f} V     ")
            stdscr.refresh()

            log_w.writerow([f"{t:.3f}", last_mode, f"{setL:.3f}", f"{setR:.3f}",
                            f"{rps_l:.3f}", f"{rps_r:.3f}",
                            f"{out_l:.4f}", f"{out_r:.4f}",
                            f"{left_pid.cfg.Kp:.4f}", f"{left_pid.cfg.Ki:.4f}", f"{left_pid.cfg.Kd:.4f}",
                            f"{vb:.3f}" if vb is not None else ""])
            log_f.flush()

            rate.sleep()

    finally:
        motors.stop()
        motors.disable()
        motors.close()
        log_f.close()
        curses.nocbreak(); stdscr.keypad(0); curses.endwin()

if __name__ == "__main__":
    curses.wrapper(main)

