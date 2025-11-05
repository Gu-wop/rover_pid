import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import select
import sys


# Distance estimation function
def estimate_distance(voltage):
    if voltage <= 0.4:
        return "> 150 cm"
    try:
        distance = 60 / (voltage - 0.1)
        return f"{int(distance)} cm"
    except ZeroDivisionError:
        return "Out of range"


# Remap function (not currently used, but kept if needed later)
def remap_range(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = int(value - left_min) / int(left_span)
    return int(right_min + (value_scaled * right_span))


# SPI and MCP3008 setup
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE0)  # if MCP3008 CS is on GPIO 8
mcp = MCP.MCP3008(spi, cs)
chan0 = AnalogIn(mcp, MCP.P0)

# Read and print data repeatedly until 'q' is pressed
while True:
    print("Raw ADC Value: ", chan0.value)
    print(f"ADC Voltage: {chan0.voltage:.2f} V")
    print("Estimated Distance:", estimate_distance(chan0.voltage))
    time.sleep(0.5)  # Wait 0.5 seconds before next reading
    
    # Check if 'q' is pressed
    if select.select([sys.stdin], [], [], 0)[0]:
        if sys.stdin.read(1) == 'q':
            break

