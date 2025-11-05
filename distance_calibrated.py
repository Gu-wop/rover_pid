#!/usr/bin/env python3
# distance_sensor_logger_calibrated.py
#
# Reads MCP3008 CH0 and prints:
# - Raw 16-bit ADC value
# - ADC voltage (V)
# - Calibrated distance (cm), using your measured points
#
# Wiring assumptions:
#   - MCP3008 CS -> D5 (BCM 5)  [change CS_PIN if needed]
#   - MCP3008 CH0 -> sensor Vout
#   - Vref ~ 3.3 V
#
# Ctrl+C to quit.

import time
from collections import deque
from typing import List, Tuple

import board
import busio
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


# --------------------------- Calibration ---------------------------

class SensorCalibrator:
    """
    Calibrates sensor voltage (V) -> distance (cm).

    Modes:
      - "interp": piecewise linear interpolation using your measured points
      - "model" : smooth fit  d ≈ A*(1/V) + B

    Optional moving-average smoothing on voltage to reduce jitter.
    """

    def __init__(self, smoothing_window: int = 3):
        self._buf = deque(maxlen=max(1, int(smoothing_window)))

        # ===== Your measured means (Voltage -> Distance) =====
        # Averages computed from the samples you posted:
        #   20 cm: 2.4182 V
        #   30 cm: 1.8333 V
        #   40 cm: 1.3633 V
        #   50 cm: 1.0550 V
        #   60 cm: 0.8533 V
        self._table_v_to_d: List[Tuple[float, float]] = sorted([
            (0.8533333333333333, 60.0),
            (1.0550000000000000, 50.0),
            (1.3633333333333333, 40.0),
            (1.8333333333333333, 30.0),
            (2.4181818181818184, 20.0),
        ], key=lambda t: t[0])

        # Smooth reciprocal fit d ≈ A*(1/V) + B (fitted to the above points)
        self._A = 51.6335156493558
        self._B = 0.6322633923064813

        self._v_min = self._table_v_to_d[0][0]
        self._v_max = self._table_v_to_d[-1][0]

    def distance_from_voltage(self, v: float, mode: str = "interp") -> float:
        # Moving-average smoothing (on voltage)
        self._buf.append(float(v))
        v_smooth = sum(self._buf) / len(self._buf)

        if mode == "interp":
            return self._interp(v_smooth)
        elif mode == "model":
            return self._model(v_smooth)
        else:
            raise ValueError('mode must be "interp" or "model"')

    # ----- internals -----
    def _model(self, v: float) -> float:
        if v <= 0.0:
            return float("inf")
        return self._A * (1.0 / v) + self._B

    def _interp(self, v: float) -> float:
        pts = self._table_v_to_d
        # outside measured range -> fall back to smooth model
        if v <= self._v_min or v >= self._v_max:
            return self._model(v)

        for i in range(len(pts) - 1):
            v0, d0 = pts[i]
            v1, d1 = pts[i + 1]
            if v0 <= v <= v1:
                t = (v - v0) / (v1 - v0)
                return d0 + t * (d1 - d0)

        # Shouldn't reach here; guard with model as fallback.
        return self._model(v)


# --------------------------- Hardware setup ---------------------------

# Chip Select pin for MCP3008 (adjust if your wiring differs)
CS_PIN = board.D5
ADC_VREF = 3.3           # for info; AnalogIn.voltage already returns volts
READ_HZ = 10             # readings per second
MODE = "interp"          # "interp" (best within 20–60 cm) or "model"

# SPI + MCP3008 + channel 0
spi = busio.SPI(clock=board.SCLK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(CS_PIN)
mcp = MCP.MCP3008(spi, cs)
chan = AnalogIn(mcp, MCP.P0)  # CH0

cal = SensorCalibrator(smoothing_window=3)


# --------------------------- Main loop ---------------------------

def main():
    print("Starting calibrated distance logger. Press Ctrl+C to stop.")
    print(f"Mode={MODE}  Rate={READ_HZ} Hz  (interp range ≈ 20–60 cm)")

    period = 1.0 / READ_HZ
    while True:
        t0 = time.time()

        raw = chan.value          # 0..65535 (16-bit)
        v = chan.voltage          # volts (scaled by Vref)
        d_cm = cal.distance_from_voltage(v, mode=MODE)

        # Optional: highlight very close obstacles (uncomment if you like)
        # warn = "  !!! CLOSE (<20cm)" if d_cm < 20 else ""

        print(
            f"Raw ADC Value: {raw:6d} | "
            f"ADC Voltage: {v:4.2f} V | "
            f"Calibrated Distance: {d_cm:4.0f} cm"
            # f"{warn}"
        )

        # pace the loop
        dt = time.time() - t0
        sleep_left = period - dt
        if sleep_left > 0:
            time.sleep(sleep_left)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye!")

