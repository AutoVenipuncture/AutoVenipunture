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

# 02_dual_motor_test.py
# Demonstrates simple simultaneous control of two steppers by interleaving step pulses.
# This is not hardware-timed synchronized motion (Pi is not a hard real-time device),
# but is sufficient for demo purposes.

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

# Pin assignments for two motors (change as per wiring)
DIR1, STEP1, EN1 = 5, 6, 13
DIR2, STEP2, EN2 = 19, 26, 21

STEPS = 200 * 16  # one revolution at 1/16 microstep

def setup():
    """Configure all GPIO pins for two motors and enable drivers."""
    GPIO.setmode(GPIO.BCM)
    for pin in (DIR1, STEP1, EN1, DIR2, STEP2, EN2):
        GPIO.setup(pin, GPIO.OUT)
    # Enable both drivers (active low)
    GPIO.output(EN1, GPIO.LOW)
    GPIO.output(EN2, GPIO.LOW)

def step_pair(steps, delay=0.001):
    """
    Interleave steps between motor1 and motor2 to approximate simultaneous motion.
    - steps: microsteps per motor
    - delay: pulse half-period
    """
    # Set both directions forward
    GPIO.output(DIR1, GPIO.HIGH)
    GPIO.output(DIR2, GPIO.HIGH)

    for i in range(steps):
        # Motor 1 pulse
        GPIO.output(STEP1, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP1, GPIO.LOW)

        # Motor 2 pulse
        GPIO.output(STEP2, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP2, GPIO.LOW)

def main():
    """Run demo: forward, then backward."""
    try:
        setup()
        print("Dual motor interleaved test: forward")
        step_pair(STEPS, delay=0.0009)

        time.sleep(0.5)

        # Reverse directions
        GPIO.output(DIR1, GPIO.LOW)
        GPIO.output(DIR2, GPIO.LOW)
        print("Dual motor interleaved test: reverse")
        step_pair(STEPS, delay=0.0011)

        print("Dual motor test done.")
    finally:
        GPIO.output(EN1, GPIO.HIGH)
        GPIO.output(EN2, GPIO.HIGH)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
