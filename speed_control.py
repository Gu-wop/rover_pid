#!/usr/bin/env python3
import os, csv, time, math, curses, datetime as dt
from dataclasses import dataclass

import config as cfg
from pid import PID, PIDConfig
from encoder import WheelEncoder
from motor_driver import RoverMotors
from util.rate import Rate
from battery_monitor import BatteryMonitor, BatteryConfig

@dataclass
class WheelState:
    set_rps: float = 0.0
    meas_rps: float = 0.0
    out: float = 0.0

def make_logger():
    runs_dir = "runs"
    os.makedirs(runs_dir, exist_ok=True)
    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(runs_dir, f"LOG-{ts}.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "set_left_rps", "set_right_rps", "meas_left_rps", "meas_right_rps", "out_left", "out_right",
                "Kp", "Ki", "Kd", "battery_v"])
    return f, w, path

def run(stdscr):
    curses.cbreak()
    stdscr.nodelay(1)
    stdscr.keypad(1)
    stdscr.addstr(0, 2, "PID Speed Control: arrows change setpoint, [ ] Kp, ; ' Ki, , . Kd, SPACE pause, e stop, r reset, v battery, q quit")

    left_pid = PID(PIDConfig(**cfg.LEFT_PID))
    right_pid = PID(PIDConfig(**cfg.RIGHT_PID))

    left_enc = WheelEncoder(cfg.LEFT_ENCODER_PINS[0], cfg.LEFT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)
    right_enc = WheelEncoder(cfg.RIGHT_ENCODER_PINS[0], cfg.RIGHT_ENCODER_PINS[1], cfg.ENCODER_STEPS_PER_REV)

    motors = RoverMotors(cfg.SLP_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.INVERT_LEFT, cfg.INVERT_RIGHT)
    motors.enable()

    batt = BatteryMonitor(BatteryConfig(adc_channel=cfg.ADC_CHANNEL, vref=cfg.VREF, divider_scale=cfg.DIVIDER_SCALE))

    rate = Rate(cfg.LOOP_HZ)
    t0 = time.time()

    wl = WheelState()
    wr = WheelState()
    base = 1.0  # rps
    turn = 0.0  # differential rps

    paused = False

    log_f, log_w, log_path = make_logger()
    stdscr.addstr(1, 2, f"Logging to {log_path}")

    try:
        while True:
            # --- UI ---
            k = stdscr.getch()
            if k != -1:
                if k == ord('q'):
                    break
                elif k == ord('e'):  # emergency stop
                    wl.set_rps = wr.set_rps = 0.0
                    motors.stop()
                elif k == ord(' '):  # pause toggle
                    paused = not paused
                elif k == ord('r'):
                    left_pid.reset(); right_pid.reset()
                    left_enc.zero_odometry(); right_enc.zero_odometry()
                elif k == ord('v'):
                    vb = batt.read_vin()
                    if vb is not None:
                        stdscr.addstr(3, 2, f"Battery: {vb:5.2f} V   ")
                    else:
                        stdscr.addstr(3, 2, "Battery: (no ADC)     ")
                elif k == curses.KEY_UP:
                    base += 0.2
                elif k == curses.KEY_DOWN:
                    base = max(0.0, base - 0.2)
                elif k == curses.KEY_LEFT:
                    turn -= 0.2
                elif k == curses.KEY_RIGHT:
                    turn += 0.2
                elif k == ord('['):
                    left_pid.cfg.Kp = right_pid.cfg.Kp = max(0.0, left_pid.cfg.Kp - 0.05)
                elif k == ord(']'):
                    left_pid.cfg.Kp = right_pid.cfg.Kp = left_pid.cfg.Kp + 0.05
                elif k == ord(';'):
                    left_pid.cfg.Ki = right_pid.cfg.Ki = max(0.0, left_pid.cfg.Ki - 0.01)
                elif k == ord("'"):
                    left_pid.cfg.Ki = right_pid.cfg.Ki = left_pid.cfg.Ki + 0.01
                elif k == ord(','):
                    left_pid.cfg.Kd = right_pid.cfg.Kd = max(0.0, left_pid.cfg.Kd - 0.01)
                elif k == ord('.'):
                    left_pid.cfg.Kd = right_pid.cfg.Kd = left_pid.cfg.Kd + 0.01

            wl.set_rps = max(0.0, base - turn)
            wr.set_rps = max(0.0, base + turn)

            # --- Measurements ---
            dsl, dtl, rps_l = left_enc.read()
            dsr, dtr, rps_r = right_enc.read()
            wl.meas_rps = rps_l
            wr.meas_rps = rps_r

            # --- Control ---
            if paused:
                out_l = out_r = 0.0
            else:
                out_l = left_pid.step(wl.set_rps, wl.meas_rps, cfg.DT)
                out_r = right_pid.step(wr.set_rps, wr.meas_rps, cfg.DT)

            motors.set_speeds(out_l, out_r)
            wl.out, wr.out = out_l, out_r

            # --- Render & log ---
            t = time.time() - t0
            vb = batt.read_vin()
            stdscr.addstr(2, 2, f"t={t:6.2f}s | set L/R: {wl.set_rps:4.1f} / {wr.set_rps:4.1f} rps | meas L/R: {wl.meas_rps:4.1f} / {wr.meas_rps:4.1f} rps | out L/R: {wl.out:+.2f}/{wr.out:+.2f}   ")
            stdscr.addstr(4, 2, f"Gains: Kp={left_pid.cfg.Kp:.2f} Ki={left_pid.cfg.Ki:.2f} Kd={left_pid.cfg.Kd:.2f}    ")
            if vb is not None:
                stdscr.addstr(5, 2, f"Battery: {vb:5.2f} V     ")
            else:
                stdscr.addstr(5, 2, "Battery: (no ADC)        ")

            log_w.writerow([f"{t:.3f}", f"{wl.set_rps:.3f}", f"{wr.set_rps:.3f}", f"{wl.meas_rps:.3f}", f"{wr.meas_rps:.3f}",
                            f"{wl.out:.4f}", f"{wr.out:.4f}", f"{left_pid.cfg.Kp:.4f}", f"{left_pid.cfg.Ki:.4f}", f"{left_pid.cfg.Kd:.4f}",
                            f"{vb:.3f}" if vb is not None else ""])
            log_f.flush()

            rate.sleep()

    finally:
        motors.stop()
        motors.disable()
        motors.close()
        log_f.close()

def main():
    curses.wrapper(run)

if __name__ == "__main__":
    main()
