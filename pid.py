import math
from dataclasses import dataclass

@dataclass
class PIDConfig:
    Kp: float = 0.0
    Ki: float = 0.0
    Kd: float = 0.0
    out_min: float = -1.0
    out_max: float = 1.0

class PID:
    """
    Discrete PID controller with:
    - anti-windup via integral clamping
    - derivative on measurement (robust to setpoint steps)
    - optional output limits
    """
    def __init__(self, cfg: PIDConfig):
        self.cfg = cfg
        self.integral = 0.0
        self.prev_meas = None
        self.out = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_meas = None
        self.out = 0.0

    def step(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement

        # Proportional
        p = self.cfg.Kp * error

        # Integral with simple anti-windup clamping
        self.integral += error * dt * self.cfg.Ki
        # clamp integral so P+I stays within limits even before D
        i_unclamped = self.integral
        i_min = self.cfg.out_min - p
        i_max = self.cfg.out_max - p
        self.integral = max(min(self.integral, i_max), i_min)
        i = self.integral

        # Derivative on measurement (reduces derivative kick)
        if self.prev_meas is None:
            d_meas = 0.0
        else:
            d_meas = (measurement - self.prev_meas) / dt
        d = - self.cfg.Kd * d_meas  # minus sign because derivative on measurement
        self.prev_meas = measurement

        out = p + i + d
        # final clamp
        out = max(min(out, self.cfg.out_max), self.cfg.out_min)
        self.out = out
        return out
