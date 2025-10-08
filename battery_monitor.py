from dataclasses import dataclass

@dataclass
class BatteryConfig:
    adc_channel: int = 0
    vref: float = 3.3
    divider_scale: float = (6100.0 + 3300.0) / 3300.0

class BatteryMonitor:
    """
    Optional MCP3008-based battery voltage monitor.
    If MCP3008 libs are missing or SPI not available, it degrades gracefully and returns None.
    """
    def __init__(self, cfg: BatteryConfig):
        self.cfg = cfg
        self._ok = False
        try:
            import busio
            import digitalio
            import board
            from adafruit_mcp3xxx.mcp3008 import MCP3008
            from adafruit_mcp3xxx.analog_in import AnalogIn
            spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
            cs = digitalio.DigitalInOut(board.CE0)  # use CE0 by default
            self.mcp = MCP3008(spi=spi, cs=cs)
            self.chan = AnalogIn(self.mcp, getattr(self.mcp, f"P{self.cfg.adc_channel}"))
            self._ok = True
        except Exception as e:
            self._ok = False
            self._err = e

    def read_vout(self):
        if not self._ok:
            return None
        # AnalogIn gives .voltage already scaled to Vref
        return float(self.chan.voltage)

    def read_vin(self):
        vout = self.read_vout()
        if vout is None:
            return None
        return vout * self.cfg.divider_scale
