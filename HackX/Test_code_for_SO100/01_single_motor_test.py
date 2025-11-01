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

# 01_single_motor_test.py
# Simple one-stepper test: rotates the motor a given number of steps in both directions at different speeds.

import time
try:
    import RPi.GPIO as GPIO  # Raspberry Pi GPIO library
except Exception as e:
    # If not running on Raspberry Pi, we still want the file to be readable/executable for demo.
    # The stub below helps in testing on non-Pi environments (no GPIO actions performed).
    class _GPIOStub:
        BCM = BOARD = OUT = IN = LOW = HIGH = None
        def setmode(self, *a, **k): pass
        def setup(self, *a, **k): pass
        def output(self, *a, **k): pass
        def cleanup(self, *a, **k): pass
    GPIO = _GPIOStub()

# Pin assignment (change as wired on your Pi)
DIR_PIN = 27    # Direction GPIO pin
STEP_PIN = 17   # Step GPIO pin
EN_PIN = 22     # Enable pin (active LOW on DRV8825)

# Motor parameters
STEPS_PER_REV = 200        # full steps per revolution (1.8 degree motor)
MICROSTEP = 16             # microstepping set on driver (MS1-MS3) — ensure this matches your driver jumper settings
STEPS_PER_REV_MICRO = STEPS_PER_REV * MICROSTEP  # effective microsteps per revolution

def setup_gpio():
    """Initialize GPIO pins for the motor test."""
    GPIO.setmode(GPIO.BCM)               # use Broadcom pin numbering
    GPIO.setup(DIR_PIN, GPIO.OUT)        # DIR pin as output
    GPIO.setup(STEP_PIN, GPIO.OUT)       # STEP pin as output
    GPIO.setup(EN_PIN, GPIO.OUT)         # ENABLE pin as output
    GPIO.output(EN_PIN, GPIO.LOW)        # Enable the driver (DRV8825 ENABLE is active LOW)

def step_motor(steps, delay_s=0.001, direction=True):
    """
    Step the motor a given number of microsteps.
    - steps: number of microsteps to move (int)
    - delay_s: delay between step pulses in seconds (use to control speed)
    - direction: True for one direction, False for the other
    """
    # Set direction pin according to desired rotation
    GPIO.output(DIR_PIN, GPIO.HIGH if direction else GPIO.LOW)

    # Generate step pulses
    for i in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)  # rising edge -> step
        time.sleep(delay_s)               # keep pulse high for 'delay_s'
        GPIO.output(STEP_PIN, GPIO.LOW)   # falling edge
        time.sleep(delay_s)               # delay before next pulse

def main():
    """Main demo routine: rotates clockwise, then counter-clockwise at different speeds."""
    try:
        setup_gpio()
        print("Single motor test starting...")
        # One full revolution slow
        print("-> One full revolution (slow)")
        step_motor(STEPS_PER_REV_MICRO, delay_s=0.002, direction=True)

        time.sleep(0.5)  # brief pause

        # Two revolutions faster
        print("-> Two revolutions (faster)")
        step_motor(2 * STEPS_PER_REV_MICRO, delay_s=0.0008, direction=False)

        print("Single motor test completed.")
    finally:
        # Always disable motors and cleanup GPIO on exit for safety
        GPIO.output(EN_PIN, GPIO.HIGH)  # disable driver
        GPIO.cleanup()

if __name__ == "__main__":
    main()
