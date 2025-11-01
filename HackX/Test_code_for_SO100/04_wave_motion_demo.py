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

# 04_wave_motion_demo.py
# Demonstrates a smooth "wave" like motion across multiple joints.
# This script uses simple timing-based profiles to create a visually pleasing demo.

import time
import math
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

# Basic pin map for 6 motors (DIR, STEP, EN)
MOTORS = [
    {"DIR":5,"STEP":6,"EN":13},
    {"DIR":19,"STEP":26,"EN":21},
    {"DIR":16,"STEP":12,"EN":20},
    {"DIR":25,"STEP":24,"EN":23},
    {"DIR":17,"STEP":27,"EN":22},
    {"DIR":18,"STEP":4,"EN":3},
]

MICROSTEPS = 16
BASE_STEPS = 40 * MICROSTEPS  # base amplitude of motion (microsteps)
SPEED = 0.0007  # base delay between step pulses (smaller = faster)

def setup():
    """Enable all drivers and configure GPIO."""
    GPIO.setmode(GPIO.BCM)
    for m in MOTORS:
        GPIO.setup(m["DIR"], GPIO.OUT)
        GPIO.setup(m["STEP"], GPIO.OUT)
        GPIO.setup(m["EN"], GPIO.OUT)
        GPIO.output(m["EN"], GPIO.LOW)

def wave_cycle(cycle_count=3):
    """
    Create 'cycle_count' repetitions of a wave passing through joints.
    The idea: each motor moves a computed offset number of steps based on a sine function,
    producing a wave-like motion from base to tip.
    """
    for cycle in range(cycle_count):
        # t progresses 0..2pi across all motors to make the wave move
        for phase in range(0, 360, 10):
            # convert phase to radians
            radians = math.radians(phase)
            # for each motor, compute target microsteps offset using sine wave with phase shift
            for idx, m in enumerate(MOTORS):
                offset = int(BASE_STEPS * (0.5 + 0.5 * math.sin(radians + idx * 0.5)))
                # choose direction based on sign of (sin) for interesting motion
                direction = (math.sin(radians + idx * 0.5) >= 0)
                GPIO.output(m["DIR"], GPIO.HIGH if direction else GPIO.LOW)
                # perform offset microsteps (small bursts)
                for _ in range(offset // 8):
                    GPIO.output(m["STEP"], GPIO.HIGH)
                    time.sleep(SPEED)
                    GPIO.output(m["STEP"], GPIO.LOW)
                    time.sleep(SPEED)

def main():
    """Run wave demo."""
    try:
        setup()
        print("Running wave motion demo (visual demo for judges)...")
        wave_cycle(2)
        print("Wave demo finished.")
    finally:
        for m in MOTORS:
            GPIO.output(m["EN"], GPIO.HIGH)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
