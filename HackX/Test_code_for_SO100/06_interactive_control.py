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

# 06_interactive_control.py
# Simple keyboard-based interactive control for 6 joints.
# Use keys to move specific joints while holding the key.
# Controls (example):
#  - q/a: joint1 +/-
#  - w/s: joint2 +/-
#  - e/d: joint3 +/-
#  - r/f: joint4 +/-
#  - t/g: joint5 +/-
#  - y/h: joint6 +/-
#
# This uses `curses` for a minimal text UI and non-blocking key reads.

import time
import curses

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

# Joint pin map
MOTORS = [
    {"DIR":5,"STEP":6,"EN":13},
    {"DIR":19,"STEP":26,"EN":21},
    {"DIR":16,"STEP":12,"EN":20},
    {"DIR":25,"STEP":24,"EN":23},
    {"DIR":17,"STEP":27,"EN":22},
    {"DIR":18,"STEP":4,"EN":3}
]

STEP_DELAY = 0.0009  # delay between microstep pulses when key held

KEYMAP = {
    ord('q'): (0, True),   # joint 1 positive
    ord('a'): (0, False),  # joint 1 negative
    ord('w'): (1, True),
    ord('s'): (1, False),
    ord('e'): (2, True),
    ord('d'): (2, False),
    ord('r'): (3, True),
    ord('f'): (3, False),
    ord('t'): (4, True),
    ord('g'): (4, False),
    ord('y'): (5, True),
    ord('h'): (5, False),
}

def setup_gpio():
    """Configure GPIO and enable all drivers."""
    GPIO.setmode(GPIO.BCM)
    for m in MOTORS:
        GPIO.setup(m["DIR"], GPIO.OUT)
        GPIO.setup(m["STEP"], GPIO.OUT)
        GPIO.setup(m["EN"], GPIO.OUT)
        GPIO.output(m["EN"], GPIO.LOW)

def step_once(motor_idx, direction=True):
    """Do a single microstep on the selected motor."""
    m = MOTORS[motor_idx]
    GPIO.output(m["DIR"], GPIO.HIGH if direction else GPIO.LOW)
    GPIO.output(m["STEP"], GPIO.HIGH)
    time.sleep(STEP_DELAY)
    GPIO.output(m["STEP"], GPIO.LOW)
    time.sleep(STEP_DELAY)

def ui_loop(stdscr):
    """
    Curses-based UI loop.
    - stdscr: curses standard screen provided by curses.wrapper
    """
    stdscr.nodelay(True)         # do not block on getch()
    stdscr.clear()
    stdscr.addstr(0,0,"SO100 Interactive Control — press 'x' to exit")
    stdscr.addstr(1,0,"Controls: q/a (j1), w/s (j2), e/d (j3), r/f (j4), t/g (j5), y/h (j6)")
    stdscr.refresh()

    try:
        while True:
            c = stdscr.getch()   # non-blocking key read
            if c == ord('x'):
                break
            if c in KEYMAP:
                motor_idx, direction = KEYMAP[c]
                stdscr.addstr(3,0,f"Stepping joint {motor_idx+1} {'+' if direction else '-'}   ")
                stdscr.refresh()
                step_once(motor_idx, direction)
            else:
                # small sleep to reduce CPU when idle
                time.sleep(0.01)
    finally:
        # on exit, disable motors
        for m in MOTORS:
            GPIO.output(m["EN"], GPIO.HIGH)

def main():
    """Initialize GPIO and launch curses UI."""
    try:
        setup_gpio()
        curses.wrapper(ui_loop)
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
