# Autonomous Drone Platform — Agent Context

## Architecture

The flight controller is a continuous-time LQR stabilizer fed by ZED camera state estimation.

```
drone.py  →  FlightController  →  LQR  →  Motors
                   ↑                  ↑
                  RX                 Zed
```

- **`drone.py`** — entry point; sets up logging, instantiates `FlightController`
- **`models/flight_controller.py`** — main loop: reads RC sticks, gets ZED state, runs LQR, outputs motor speeds
- **`models/lqr.py`** — 12-state continuous-time LQR; gain computed once at init via scipy CARE solver
- **`models/zed.py`** — ZED camera wrapper; produces the 12-element state vector
- **`models/rx.py`** — FlySky iBUS RC receiver; outputs [roll, pitch, throttle, yaw] in µs (1000–2000)
- **`models/motors.py`** — UART motor ESC driver; hardware reorder applied internally in `output_speeds()`

## LQR State and Control

**State (12):** `[x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]`
**Control (4):** `[T_cmd, roll_cmd, pitch_cmd, yaw_cmd]` — all in motor-speed units (0–100)

Control output feeds directly into the motor mixer in `FlightController.run()`:
```
motor 0 (front-right): hover + T + pitch + roll - yaw
motor 1 (rear-right):  hover + T - pitch + roll + yaw
motor 2 (rear-left):   hover + T - pitch - roll - yaw
motor 3 (front-left):  hover + T + pitch - roll + yaw
```

## Coordinate Frame

ZED camera uses **RIGHT_HANDED_Y_UP**: Y is altitude, X is lateral-right, Z is backward.

- `get_state()` in `zed.py` maps ZED axes directly — Y is the altitude channel (state index 1, velocity index 4)
- Raw ZED roll and pitch are negated to match flight-controller sign convention
- Angular velocity assumed to be in **deg/s** and converted to rad/s — verify this before flight

## Motor Reorder

Physical ESC wiring does not match logical motor order. The mapping `[0, 3, 1, 2]` is applied inside `Motors.output_speeds()` — callers always use logical order (front-right=0, rear-right=1, rear-left=2, front-left=3).

## Placeholder Values — Require Hardware Measurement

All physical parameters in `models/lqr.py` are estimates. Do not fly without verifying:

| Constant | Current value | What it needs |
|---|---|---|
| `MASS` | 0.5 kg | Weigh the airframe |
| `IXX`, `IYY` | 0.005 kg·m² | Measure or estimate moment of inertia |
| `IZZ` | 0.009 kg·m² | Measure or estimate moment of inertia |
| `COLLECTIVE_ACCEL` | 2g / 100 | Thrust test: measure g-force at full throttle |
| `ROLL_ACCEL`, `PITCH_ACCEL` | 0.50 rad/s² | Bench test: spin motors, measure angular response |
| `YAW_ACCEL` | 0.10 rad/s² | Bench test: spin motors, measure angular response |

## Pre-Flight Checklist

- [ ] Replace placeholder LQR physical params with measured values (see table above)
- [ ] Confirm ZED `get_angular_velocity()` output is in deg/s (not rad/s); if already rad/s, remove the conversion in `zed.py`
- [ ] Verify A-matrix coupling signs: tilting the drone should produce lateral velocity in the expected direction per the ZED frame. `A[3,6]=g` (roll → X accel), `A[5,7]=-g` (pitch → -Z accel)
- [ ] Tune Q/R matrices in `lqr.py` based on flight behavior: high Q → aggressive correction, high R → soft actuation

## Package Setup

`pip install -e .` from the repo root installs the `models` package and all dependencies (numpy, scipy, pyserial). Scripts import directly from `models.*` — no sys.path manipulation needed. pyzed is installed separately via the ZED SDK installer.
