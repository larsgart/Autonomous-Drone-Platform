"""
ZED state verification script. Run with ZED plugged in, props off:
    python3 tools/verify_zed.py

Streams the 12-element state vector so you can manually verify:

  1. YAW SIGN — rotate drone CCW (left turn, viewed from above).
     psi should INCREASE. If it decreases, negate psi in the sin/cos
     terms in flight_controller.py lines 71-73.

  2. ANGULAR VELOCITY UNITS — rotate briskly (~90°/s by hand).
     p/q/r should read roughly ±1.5 rad/s at that rate.
     If you see ±90, the deg→rad conversion is missing in zed.py.
     If you see ±0.02, it's being applied twice — remove one.

  3. PITCH COUPLING — tilt nose down (forward pitch).
     theta should go NEGATIVE (negated in get_state()).
     After holding ~0.5 s, vz should go NEGATIVE (forward = -Z).

  4. ROLL COUPLING — tilt right.
     phi should go NEGATIVE (negated in get_state()).
     After holding ~0.5 s, vx should go POSITIVE (+X = right).

Press Ctrl-C to exit.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.zed import Zed

LABELS = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r']
UNITS  = ['m', 'm', 'm', 'm/s', 'm/s', 'm/s', 'rad', 'rad', 'rad', 'r/s', 'r/s', 'r/s']
COL_W  = 10

HEADER = "  " + "".join(f"{f'{l}({u})':>{COL_W}}" for l, u in zip(LABELS, UNITS))
SEP    = "  " + "-" * (COL_W * len(LABELS))


def render(x, i):
    vals = "  " + "".join(f"{v:>+{COL_W}.3f}" for v in x)
    # Reprint header every 20 rows so it's always visible after a resize
    if i % 20 == 0:
        print(f"\n{HEADER}\n{SEP}")
    print(vals)


def main():
    print("Opening ZED camera...")
    zed = Zed()
    print("ZED ready.\n")
    print(__doc__)
    input("Press Enter to start streaming...")

    i = 0
    try:
        while True:
            x = zed.get_state()
            if x is None:
                continue
            render(x, i)
            i += 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        zed.close()


if __name__ == '__main__':
    main()
