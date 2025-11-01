"""
SO100 Demo Scripts — DRV8825 stepper driver + Raspberry Pi 5
Author: Generated for Automated Venipuncture SO100 project
Driver: DRV8825 (STEP/DIR/ENABLE interface assumed)
Target: Raspberry Pi (RPi.GPIO library). Run on Pi with appropriate permissions (sudo).

IMPORTANT SAFETY NOTES:
 - Ensure motor power (VMOT) is connected and that logic GND is common with Pi GND.
 - Set current limit on the DRV8825 drivers before powering motors.
 - Use appropriate heat-sinking and do not stall motors for long durations.
 - These scripts are for demonstration and judging; adapt current/voltage and limits before production use.

Each script is heavily commented line-by-line to explain what it does.
"""

# 03_six_motor_test.py
# Sequentially test six stepper motors one after another.
# Useful to verify wiring and get a visual test of each joint.

import time
try:
    import RPi.GPIO as GPIO
except Exception:
    class _GPIOStub:
        BCM = BOARD = OUT = IN = LOW = HIGH = None
        def setmode(self, *a, **k): pass
        def setup(self, *a, **k): pass
        def output(self, *a, **k): pass
        def cleanup(self, *a, **k): pass
    GPIO = _GPIOStub()

# Assign pins for 6 motors: each motor uses (DIR, STEP, EN)
MOTORS = [
    {"DIR": 5,  "STEP": 6,  "EN": 13},
    {"DIR": 19, "STEP": 26, "EN": 21},
    {"DIR": 16, "STEP": 12, "EN": 20},
    {"DIR": 25, "STEP": 24, "EN": 23},
    {"DIR": 17, "STEP": 27, "EN": 22},
    {"DIR": 18, "STEP": 4,  "EN": 3}
]

STEPS = 100 * 16  # demo microsteps per joint to move

def setup():
    """Configure GPIOs for all 6 motors and enable each driver."""
    GPIO.setmode(GPIO.BCM)
    for m in MOTORS:
        GPIO.setup(m["DIR"], GPIO.OUT)
        GPIO.setup(m["STEP"], GPIO.OUT)
        GPIO.setup(m["EN"], GPIO.OUT)
        GPIO.output(m["EN"], GPIO.LOW)  # enable

def move_motor(motor, steps, delay=0.001):
    """Move a single motor by 'steps' microsteps at given delay."""
    GPIO.output(motor["DIR"], GPIO.HIGH)  # set direction
    for _ in range(steps):
        GPIO.output(motor["STEP"], GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(motor["STEP"], GPIO.LOW)
        time.sleep(delay)

def main():
    """Sequential demonstration across each of the 6 motors."""
    try:
        setup()
        print("6-motor sequential test starting...")
        for idx, m in enumerate(MOTORS):
            print(f" -> Moving motor {idx+1}")
            move_motor(m, STEPS, delay=0.0009)
            time.sleep(0.4)
            # Move back
            GPIO.output(m["DIR"], GPIO.LOW)
            move_motor(m, STEPS // 2, delay=0.001)
            time.sleep(0.2)
        print("6-motor test complete.")
    finally:
        # Disable all and cleanup
        for m in MOTORS:
            GPIO.output(m["EN"], GPIO.HIGH)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
