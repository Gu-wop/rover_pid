# Rover PID Lab (Raspberry Pi + gpiozero)

This repo gives you a **working baseline** to complete the lab page you shared:
- Read quadrature **encoders** (A/B) for each wheel
- Compute **speed** and **distance** (odometer/speedometer)
- Run **two independent PID loops** (left & right wheel) for speed control
- Optional **battery voltage monitor** via MCP3008 + resistor divider
- **Keyboard** control (curses) to change setpoints while running
- **CSV logging** for tuning

> All pins/constants are in `config.py`. Adjust them to your wiring.  
> Quick start below assumes the defaults match your build.

---

## Hardware assumptions (edit to your setup)
- Raspberry Pi (Zero 2 W or similar)
- Two DC gearmotors driven by an H-bridge (enable/sleep pin exposed)
- Two **quadrature encoders** (A/B) — 3.3V logic
- Optional MCP3008 ADC for battery voltage monitor (voltage divider)
- Power to motors/battery through motor driver
- Pull-ups for encoder lines if your encoder outputs are open drain (many are)

### Default pins (change in `config.py` if needed)
- **Motor sleep/enable**: `GPIO26`
- **Left motor**: forward=`GPIO12`, backward=`GPIO18`
- **Right motor**: forward=`GPIO13`, backward=`GPIO19`
- **Left encoder**: A=`GPIO5`,  B=`GPIO6`
- **Right encoder**: A=`GPIO20`, B=`GPIO21`

### Encoder calibration
You need the **steps per wheel revolution**. If you don't know it, run:
```bash
python scripts/calibrate_steps_per_rev.py
```
Rotate a wheel **exactly one full turn** by hand and press ENTER. It prints the step delta.
Put that number (or an average of a few) into `ENCODER_STEPS_PER_REV` in `config.py`.

---

## Install (on Raspberry Pi)
```bash
sudo apt update
sudo apt install -y python3-pip python3-gpiozero python3-spidev python3-dev
pip3 install adafruit-circuitpython-mcp3xxx
```

> If you prefer `pigpio` for more reliable edge timing, also do:
```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

---

## Quick tests
1) **Motor sanity check**
```bash
python scripts/test_motors.py
```
It toggles SLP (sleep) and runs each wheel forward/backward briefly.

2) **See live encoder counts & speed**
```bash
python scripts/read_encoders.py
```

3) **Run closed-loop speed control with PID (keyboard UI)**
```bash
python speed_control.py
```
Controls (while the window has focus):
- `q` quit
- `e` emergency stop (motors stop; SLP stays on)
- `SPACE` toggle run/pause (holds current PWM)
- Arrow **Up/Down**: increase/decrease **both** wheel setpoints by 0.2 rps
- Arrow **Left/Right**: add **turn**: left decreases & right increases setpoint (and vice versa)
- `[` `]` decrease/increase **Kp**
- `;` `'` decrease/increase **Ki**
- `,` `.` decrease/increase **Kd**
- `r` reset integrators & zero odometry
- `v` print battery voltage (if MCP3008 wired)

Logs a `runs/LOG-YYYYmmdd-HHMMSS.csv` with time, setpoints, speeds, outputs, and gains.

---

## Battery voltage monitor (optional)
Wire a resistor divider from **motor supply** (Vm) to **GND** and read the midpoint with **MCP3008**.
Set your actual `R1` and `R2` in `config.py`. We compute:
```
Vin (battery) = Vout * (R1 + R2) / R2
```
For the lab values R1≈6.1kΩ and R2≈3.3kΩ, a 9V input yields ~3.3V at the ADC.

---

## Files
- `config.py` – pins, PID gains, encoder constants, loop rate, battery divider
- `pid.py` – robust discrete PID with anti‑windup & derivative on measurement
- `encoder.py` – wrapper around `gpiozero.RotaryEncoder` with convenience methods
- `motor_driver.py` – `RoverMotors` for left/right `gpiozero.Motor` + SLP pin
- `battery_monitor.py` – MCP3008 reader and Vin calculation (safe fallback if absent)
- `speed_control.py` – **main** PID loop with curses keyboard UI & CSV logging
- `scripts/test_motors.py` – quick motor directions & enable test
- `scripts/read_encoders.py` – prints counts/speeds continuously
- `scripts/calibrate_steps_per_rev.py` – helps you find steps per wheel revolution

---

## Tuning recipe (fast & safe)
1. **P-only**: start with Ki=Kd=0. Raise Kp until speed tracks but starts to mildly oscillate.
2. **Add D**: increase Kd until oscillation damps quickly after setpoint changes.
3. **Add I**: raise Ki slowly until steady-state error disappears, without slow oscillation.
4. Repeat small tweaks. Save your gains back into `config.py`.

### Notes
- Battery sag changes motor behavior; retune if needed. The logger helps.
- If an encoder reads zero while the wheel spins, check wiring and pull-ups.
- If one wheel runs backward for positive PWM, swap that motor’s pins or set `INVERT_LEFT/RIGHT` in `config.py`.
- Always test on the floor — controllers can overshoot on stands.

Good luck!