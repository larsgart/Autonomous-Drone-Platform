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

## Setpoint Mode: Velocity Tracking

RC sticks command **world-frame velocity setpoints** (not attitude angles). The flight controller rotates stick inputs from body frame to world frame using the current yaw (`psi`) before writing `x_ref`:

```
x_ref[3] (vx) = -v_fwd·sin(ψ) + v_lat·cos(ψ)   ← roll + pitch sticks, rotated
x_ref[4] (vy) = throttle deviation from centre    ← climb/descend rate
x_ref[5] (vz) = -v_fwd·cos(ψ) - v_lat·sin(ψ)   ← roll + pitch sticks, rotated
x_ref[8] (ψ)  = x[8]  (current yaw — rate-only yaw control, no position restore)
x_ref[11](r)  = yaw stick                         ← unchanged
```

- **Centre throttle (1500 µs) = hold altitude** (`x_ref[4] = 0`).  Push up to climb, down to descend.
- **Hover feedforward** is a fixed `50 × throttle_scale` motor-speed units (default 25). Adjust `throttle_scale` if the drone sinks at centre stick after gains are tuned.
- **Yaw**: releasing the yaw stick holds the current heading. The LQR applies no yaw position restoration — only yaw-rate damping.
- **Velocity limits**: `MAX_FORWARD_VEL = MAX_LATERAL_VEL = 2.0 m/s`, `MAX_VERT_VEL = 1.0 m/s`. Tune conservatively before raising.

The Q matrix is reweighted for this mode: velocity states `[5, 5, 5]`, attitude states `[5, 5, 1]` (was `[10, 10, 1]`). Attitude is now an intermediate variable rather than a direct setpoint.

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

### LQR gain sanity check (no hardware needed)
- [ ] Run `python3 tools/check_lqr.py` — all key coupling gains should be non-zero. A near-zero gain on any of `K[0,4]` (altitude), `K[1,3]` (lateral), `K[2,5]` (forward), `K[3,11]` (yaw) means the controller has no authority over that channel

### ZED / sensor verification (props off, ZED running)
- [ ] Confirm ZED `get_angular_velocity()` output is in deg/s (not rad/s); if already rad/s, remove the `math.radians()` conversion in `zed.py:92`
- [ ] **Yaw sign convention**: rotate the drone CCW (left turn) when viewed from above — `get_state()[8]` (psi) should *increase*. If it decreases, negate `psi` in the `sin`/`cos` terms in `flight_controller.py:71-73`
- [ ] Verify A-matrix coupling signs: tilting the drone forward should produce negative vz (forward = −Z). `A[3,6]=g` (roll → X accel), `A[5,7]=-g` (pitch → −Z accel)

### Physical parameter measurement (required before first flight)
- [ ] Replace placeholder LQR physical params with measured values (see table above)

### Bench test (props off, armed)
- [ ] Centre throttle stick (1500 µs) — confirm motor outputs are non-zero (hover feedforward active) but low
- [ ] Push pitch stick forward — confirm front motors increase, rear motors decrease (forward tilt)
- [ ] Push roll stick right — confirm right motors increase, left motors decrease
- [ ] Yaw the drone by hand after a small yaw-stick input; confirm the velocity setpoint rotates with heading (strafe test)

### First flight
- [ ] Tune Q/R matrices in `lqr.py` based on flight behavior: high Q → aggressive correction, high R → soft actuation
- [ ] Adjust `throttle_scale` in `FlightController.__init__` if drone sinks or climbs at centre stick
- [ ] Raise `MAX_FORWARD_VEL` / `MAX_LATERAL_VEL` / `MAX_VERT_VEL` conservatively once hover is stable

## Package Setup

`pip install -e .` from the repo root installs the `models` package and all dependencies (numpy, scipy, pyserial). Scripts import directly from `models.*` — no sys.path manipulation needed. pyzed is installed separately via the ZED SDK installer.
