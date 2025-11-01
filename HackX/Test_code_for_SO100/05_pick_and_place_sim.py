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

# 05_pick_and_place_sim.py
# Simulated pick-and-place sequence across 6 joints.
# This is a timing-based scripted motion (no sensors) that mimics:
#  - Move to approach
#  - Lower (joint sequence)
#  - "Grip" (simulated)
#  - Lift and move to place location
#  - Release and return home

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

# Motor mapping for joints (DIR, STEP, EN)
MOTORS = [
    {"DIR":5,"STEP":6,"EN":13},    # base
    {"DIR":19,"STEP":26,"EN":21},  # shoulder
    {"DIR":16,"STEP":12,"EN":20},  # elbow
    {"DIR":25,"STEP":24,"EN":23},  # wrist pitch
    {"DIR":17,"STEP":27,"EN":22},  # wrist roll
    {"DIR":18,"STEP":4,"EN":3}     # end-effector (would be gripper rotation here)
]

MICRO = 16
SPEED = 0.0009

def setup():
    """Enable drivers and configure GPIO."""
    GPIO.setmode(GPIO.BCM)
    for m in MOTORS:
        GPIO.setup(m["DIR"], GPIO.OUT)
        GPIO.setup(m["STEP"], GPIO.OUT)
        GPIO.setup(m["EN"], GPIO.OUT)
        GPIO.output(m["EN"], GPIO.LOW)

def joint_motion(motor, microsteps, direction=True, delay=SPEED):
    """Move a single joint by a given number of microsteps."""
    GPIO.output(motor["DIR"], GPIO.HIGH if direction else GPIO.LOW)
    for _ in range(microsteps):
        GPIO.output(motor["STEP"], GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(motor["STEP"], GPIO.LOW)
        time.sleep(delay)

def pick_and_place_sequence():
    """
    High-level scripted sequence:
    1) Move base and shoulder to approach position
    2) Lower elbow/wrist to 'pick' position
    3) Simulate grip (pause)
    4) Lift and move to place position
    5) Simulate release
    """
    # Approach: rotate base and raise shoulder
    print("Approach: rotating base and raising shoulder")
    joint_motion(MOTORS[0], 120 * MICRO, direction=True)
    joint_motion(MOTORS[1], 80 * MICRO, direction=False)

    # Lower wrist and elbow
    print("Lowering arm to pick position")
    joint_motion(MOTORS[2], 100 * MICRO, direction=True)
    joint_motion(MOTORS[3], 60 * MICRO, direction=True)

    # Simulate grip
    print("Simulating grip (pause)")
    time.sleep(0.8)

    # Lift object: reverse some joints and move base to place
    print("Lifting and moving to place position")
    joint_motion(MOTORS[2], 80 * MICRO, direction=False)
    joint_motion(MOTORS[0], 200 * MICRO, direction=False)

    # Simulate release
    print("Releasing object (pause)")
    time.sleep(0.6)

    # Return to home-ish pose (rough)
    print("Returning to home pose")
    for m in MOTORS:
        joint_motion(m, 60 * MICRO, direction=False)

def main():
    try:
        setup()
        print("Starting pick-and-place demo...")
        pick_and_place_sequence()
        print("Pick-and-place demo complete.")
    finally:
        # disable drivers and cleanup
        for m in MOTORS:
            GPIO.output(m["EN"], GPIO.HIGH)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
