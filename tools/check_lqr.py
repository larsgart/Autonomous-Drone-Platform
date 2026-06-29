"""
LQR gain sanity-check. Run before first flight:
    python3 tools/check_lqr.py

Prints the full K matrix and flags coupling gains that are suspiciously near
zero — which would mean the controller has no authority over that channel.
"""

import sys
import os
import importlib.util
import numpy as np

# Load lqr.py directly to avoid models/__init__.py pulling in hardware deps
# (serial, pyzed) that aren't available on dev machines.
_repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("lqr", os.path.join(_repo, "models", "lqr.py"))
_mod  = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
LQR = _mod.LQR

STATE  = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r']
OUTPUT = ['T_cmd', 'roll_cmd', 'pitch_cmd', 'yaw_cmd']

# (output_row, state_col, description)
# Each pair must have |K[row,col]| > THRESHOLD or something is broken in the
# A/B matrices or Q/R weights.
KEY_COUPLINGS = [
    (0,  4, 'T_cmd    ← vy   : altitude velocity control'),
    (1,  3, 'roll_cmd ← vx   : lateral velocity control'),
    (2,  5, 'pitch_cmd ← vz  : forward velocity control'),
    (3, 11, 'yaw_cmd  ← r    : yaw rate control'),
    (0,  1, 'T_cmd    ← y    : altitude position correction'),
    (1,  6, 'roll_cmd ← phi  : roll stabilisation'),
    (2,  7, 'pitch_cmd ← theta: pitch stabilisation'),
]

THRESHOLD = 0.01


def main():
    lqr = LQR()
    K = lqr.K

    print("K matrix  (rows = outputs, cols = states)\n")
    header = f"{'':>12}" + "".join(f"{s:>9}" for s in STATE)
    print(header)
    print("-" * len(header))
    for i, label in enumerate(OUTPUT):
        row = "".join(f"{K[i, j]:>9.4f}" for j in range(12))
        print(f"{label:>12}{row}")

    print("\nKey coupling checks:")
    all_ok = True
    for row, col, desc in KEY_COUPLINGS:
        val = K[row, col]
        if abs(val) > THRESHOLD:
            status = "OK"
        else:
            status = "WARN — gain near zero"
            all_ok = False
        print(f"  K[{row},{col:2d}] = {val:8.4f}   {desc}   {status}")

    print()
    if all_ok:
        print("All key gains look reasonable.")
    else:
        print("WARNING: one or more gains are near zero. Check A/B matrices and Q/R weights.")

    print("""
Yaw sign convention check (do this before arming):
  1. Set drone flat on the ground, ZED powered on, log psi from get_state()[8].
  2. Rotate the drone CCW when viewed from above (left turn).
  3. psi should INCREASE (positive = CCW in ZED RIGHT_HANDED_Y_UP).
     If psi DECREASES, the body-to-world rotation in flight_controller.py
     has the wrong sign — negate psi in the sin/cos terms.

Hover feedforward check (bench, props off):
  1. Center throttle stick (1500 µs).
  2. hover feedforward = 50 * throttle_scale = {:.0f} motor-speed units.
  3. Raise throttle_scale in FlightController.__init__ if the drone sinks
     at center stick after the LQR is tuned.
""".format(50 * 0.5))


if __name__ == '__main__':
    main()
