# config.py (motor pins set to match your working script)

SLP_PIN = "GPIO26"

# Match your exact order: Motor(forward, backward)
LEFT_MOTOR_PINS  = ("GPIO18", "GPIO12")
RIGHT_MOTOR_PINS = ("GPIO19", "GPIO13")

# Keep encoders here; we’ll fix those next:
LEFT_ENCODER_PINS  = (27, 22)      # try ints; we can switch if needed
RIGHT_ENCODER_PINS = (24, 23)

INVERT_LEFT = False
INVERT_RIGHT = False

ENCODER_STEPS_PER_REV = 689.25  # placeholder, we’ll calibrate
WHEEL_DIAMETER_M = 0.065
WHEEL_CIRCUMFERENCE_M = 3.1415926535 * WHEEL_DIAMETER_M

LOOP_HZ = 50.0
DT = 1.0 / LOOP_HZ
LEFT_PID  = dict(Kp=0.9, Ki=0.0, Kd=0.05, out_min=-1.0, out_max=1.0)
RIGHT_PID = dict(Kp=0.9, Ki=0.0, Kd=0.05, out_min=-1.0, out_max=1.0)

ADC_CHANNEL = 0
VREF = 3.3
R1 = 6100.0
R2 = 3300.0
DIVIDER_SCALE = (R1 + R2) / R2

