# Tuning and calibration

All settings are the first section of `hardware/alliedwork.py`. **Nothing here has been tuned on a
real robot yet**: the defaults are starting guesses. Do this once per robot build.

## Settings
| Setting | Default | What it does |
|---|---|---|
| `LEFT_PORT/DIR`, `RIGHT_PORT/DIR` | F ccw, B cw | drive motors (from the team's original `main.py`) |
| `ATTACHMENTS` | A, E | attachment motors, used as `attachment("A", ...)` |
| `COLOR_PORTS` | C, D | where to look for the color sensor |
| `WHEEL_DIAMETER_MM` | 56 | **measure it**; wrong value = wrong distances |
| `AXLE_TRACK_MM` | 112 | center-to-center of the wheels; **measure it** |
| `DIST_KP/KI/KD` | 3.0 / 0 / 0.1 | distance error (in) → speed (in/s) |
| `HEAD_KP/KI/KD` | 4.0 / 0 / 0.1 | heading error (°) → turn rate (°/s) |
| `CROSS_TRACK_GAIN` / `_MAX` | 3.0 / 10° | steering back onto the straight line |
| `MAX_SPEED` / `MAX_ACCEL` | 14 in/s / 30 in/s² | speed limit and ramp |
| `MAX_TURN_RATE` / `MAX_TURN_ACCEL` | 180 °/s / 720 °/s² | turn limits |
| `POS_TOL` / `HEAD_TOL` / `SETTLE_MS` | 0.25 in / 1° / 100 ms | "close enough" and how long it must stay there |
| `STALL_MS` | 1500 | no progress for this long = stuck |
| `CONTROL_MS` | 10 | control loop period |
| `TELEMETRY_MS` | 200 | `T,...` print interval (0 = off) |

## First-day checklist (needs the real robot)
1. **Ports/directions.** Run the dashboard; spin each drive wheel forward by hand: its angle must go **up**. Fix `LEFT_DIR`/`RIGHT_DIR` if not.
2. **Distance.** `python run.py test`, measure the real side length of the square with a tape measure (it should be 24 in).
3. **Turns.** The square should end facing the start direction (± 2°). The gyro does most of the work, so turns are usually good first.
4. **Stop.** Press CENTER during a run: the robot must stop.
5. **No PC.** `python run.py --no-start`, unplug, press CENTER on the hub: the menu appears.
Write the results down and report anything off (copy error text).

## Calibrating distance
Drive a long straight line (`drive(48)`), measure what it really travelled:

```
new WHEEL_DIAMETER_MM = old × (measured distance ÷ commanded distance)
```
Example: told 48 in, travelled 46.5 in → `56 × 46.5 / 48 = 54.25`. (Or just measure the wheel again.)

## Axle track
The gyro controls turning, so axle track matters little for `turn()`. It still affects how
the wheel speeds are split while steering. Measure it and leave it.

## Tuning the PID (only if the defaults misbehave)
Change **one number at a time**, test with `python run.py test`.
- **Overshoots or wobbles at the end of a drive** → lower `DIST_KP` or raise `DIST_KD` a little.
- **Creeps slowly at the end** → raise `DIST_KP`; `POS_TOL` can go to 0.3.
- **Turns are slow** (about 4 s for a 90° turn at the defaults) → raise `HEAD_KP` (try 6, then 8).
- **Turns overshoot or oscillate** → lower `HEAD_KP`, raise `HEAD_KD`.
- **Drifts sideways on long drives** → raise `CROSS_TRACK_GAIN`.
- **Wheels slip at the start** → lower `MAX_ACCEL`.
- Keep `KI` at 0 unless the robot consistently stops short of its target.

## Squaring to a wall
Back into a wall at 3 in/s: `square_to_wall(heading=0, y=3)`. The `y` value is the distance from
the wheel axle to the back of the robot. Do it after long drives to remove drift.

Use the simulator to test logic, not tuning: its numbers are ideal. See [SIMULATOR](SIMULATOR.md).
