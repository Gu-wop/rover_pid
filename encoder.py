from gpiozero import RotaryEncoder
from time import time

class WheelEncoder:
    def __init__(self, pin_a: str, pin_b: str, steps_per_rev: float):
        # max_steps=None means unbounded (no wrap)
        self.re = RotaryEncoder(a=pin_a, b=pin_b, max_steps=0)
        self.steps_per_rev = float(steps_per_rev)
        self._last_steps = 0
        self._last_time = time()
        self.total_steps = 0  # mirror of re.steps (in case we want to zero independently)

    def reset(self):
        # Keep underlying re at current but zero our relative counters
        self._last_steps = self.re.steps
        self._last_time = time()

    def zero_odometry(self):
        # set "relative zero" by capturing current steps
        self.re.steps = 0
        self._last_steps = 0
        self.total_steps = 0
        self._last_time = time()

    def read(self):
        """Return tuple: (delta_steps, delta_t, rps_estimate) since last read."""
        now = time()
        steps = self.re.steps
        ds = steps - self._last_steps
        dt = now - self._last_time if (now - self._last_time) > 0 else 1e-6

        self._last_steps = steps
        self._last_time = now
        self.total_steps = steps

        rps = (ds / self.steps_per_rev) / dt
        return ds, dt, rps

    @property
    def revs(self) -> float:
        return self.re.steps / self.steps_per_rev
