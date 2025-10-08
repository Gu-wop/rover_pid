import time

class Rate:
    """Simple ROS-style rate helper."""
    def __init__(self, hz: float):
        self.period = 1.0 / hz
        self._next = time.time()

    def sleep(self):
        now = time.time()
        if self._next <= now:
            self._next = now + self.period
            return
        delay = self._next - now
        time.sleep(delay)
        self._next += self.period
