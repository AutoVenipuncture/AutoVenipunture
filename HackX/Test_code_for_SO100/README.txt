
SO100 Demo Scripts (DRV8825) - README
====================================

Contents:
 - 01_single_motor_test.py    : Single motor basic test
 - 02_dual_motor_test.py      : Two motor interleaved demo
 - 03_six_motor_test.py       : Sequential test across all six motors
 - 04_wave_motion_demo.py     : Wave motion pattern across multiple joints
 - 05_pick_and_place_sim.py   : Scripted pick-and-place motion sequence (no sensors)
 - 06_interactive_control.py  : Keyboard interactive control (curses based)

How to use:
 - Place scripts on Raspberry Pi 5.
 - Install dependencies: RPi.GPIO is standard on Raspberry Pi OS.
 - Run with sudo, e.g.: sudo python3 01_single_motor_test.py
 - Ensure DRV8825 microstep jumpers match MICROSTEP settings assumed in the scripts (16 microsteps).
 - Set current limit on DRV8825 according to your motor's rated current.

Safety:
 - Always power down before wiring.
 - Test with motors unloaded and at low speeds first.
 - Monitor driver temperature and current draw.

