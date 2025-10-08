from gpiozero import DigitalOutputDevice, Motor

class RoverMotors:
    def __init__(self, slp_pin: str, left_pins: tuple[str, str], right_pins: tuple[str, str],
                 invert_left: bool=False, invert_right: bool=False):
        self.slp = DigitalOutputDevice(slp_pin, active_high=True, initial_value=False)
        lfwd, lback = left_pins
        rfwd, rback = right_pins
        # gpiozero Motor: forward=pin1, backward=pin2 (PWM capable)
        self.left = Motor(forward=lfwd, backward=lback, pwm=True)
        self.right = Motor(forward=rfwd, backward=rback, pwm=True)
        self.invert_left = invert_left
        self.invert_right = invert_right

    def enable(self):
        self.slp.on()

    def disable(self):
        self.stop()
        self.slp.off()

    def set_speeds(self, left: float, right: float):
        # clamp and apply inversion
        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right

        # gpiozero Motor.value is in [-1,1]
        self.left.value = left
        self.right.value = right

    def stop(self):
        self.left.stop()
        self.right.stop()

    def close(self):
        self.stop()
        self.slp.close()
        self.left.close()
        self.right.close()
