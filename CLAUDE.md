# FLL Mission Control: Claude context

This file gives Claude what it needs to write robot missions for our team.
Keep it in the root of the project. Update it whenever the robot, the field
file or the library changes.

## What this project is
Pybricks-based toolkit for an FLL SPIKE Prime robot:
- `hardware/alliedwork.py` runs on the hub: ONE file with the settings, PID, pose tracking and the whole mission API. `hardware/sensor_stream.py` streams sensor readings.
- `run.py` (repo root) bundles `alliedwork.py` + all missions + a generated menu `main.py` and uploads it: `python run.py` (menu), `python run.py run1` (one run), `--sim` (fake robot), `--no-start` (upload only).
- On the hub the light matrix shows a run number; LEFT/RIGHT change it, CENTER starts it (`run_menu` in alliedwork.py). The number is the N in `runN.py`.
- `work/pc/` runs on the PC (everything else PC-side lives under `work/`): Bluetooth to the hub, WebSocket to the browser.
- `work/dashboard/index.html` is the localhost page: live gyro, motors, color sensor (done). Field map / run controls come later.
- `work/field/missions.json` is the map of the mat: missions, launch areas, obstacles, runs.
- `hardware/missions/` has one file per run named `runN.py`, each defining `def run():`. Copy `_template.py`.

Docs: `work/docs/` (start at README.md). Version lives in `hardware/alliedwork.py` (`VERSION`); log every change in `work/docs/CHANGELOG.md`.

## Coordinate system (always use this)
- Units: **inches**. Grid: 1 in x 1 in squares.
- Mat size: see `work/field/missions.json` → `mat` (about 93 x 45 in).
- Origin (0, 0): bottom-left corner of the mat, seen from the team's launch side.
- +x = right along the long edge. +y = up toward the far wall.
- Heading: 0° = facing +y. Positive = clockwise. 90° = facing +x, -90° = facing -x.
- Robot position = the point given by `robot.center_offset_in` (default: middle of the drive axle).

## Robot API (missions do `from alliedwork import *` and use only these)
```python
set_pose(x, y, heading)           # first line of every run (launch area default_start_pose)
drive(dist, speed=8)              # inches, + forward / - backward; speed in/s. Returns False if stuck
turn(angle, speed=90)             # degrees, + = clockwise (right)
turn_to(heading, speed=90)        # absolute heading in field coordinates
drive_to(x, y, speed=8)           # turn toward (x, y), then drive
square_to_wall(heading, x=None, y=None, speed=3, back=True)  # push into wall, then fix heading / x / y
attachment(port, angle, speed=300)     # port "A" or "E", relative
attachment_to(port, angle, speed=300)  # absolute
wait(ms), beep(), log(text), mission(id), stop()
pose(), heading(), color(), reflection()
```
- Commands are blocking: each finishes before the next starts. A 10 ms PID loop runs inside each
  command (pose from encoders + gyro, steering via a gyro-assisted `DriveBase`).
- `turn()` is relative to the last commanded heading, so errors don't pile up across turns.
- A command that makes no progress for 1.5 s stops, prints `E,..,STALLED`, returns False.
- All settings (ports, wheel size, PID gains, limits) are at the top of `hardware/alliedwork.py`.

## How to turn waypoints into code
From pose (x0, y0, h0) to point (x1, y1):
- dx = x1 - x0, dy = y1 - y0
- distance = sqrt(dx² + dy²)
- target heading = atan2(dx, dy) in degrees (0 = +y, clockwise positive)
- turn = target heading - h0, wrapped to (-180, 180]
- Emit `turn(turn, speed)` then `drive(distance, speed)`.
Round distances to 0.1 in and angles to 0.5°.

## Rules for writing missions
1. Read `work/field/missions.json` first. Use mission `approach` poses and `polygon`s for
   distances. Don't guess positions that aren't in the file; ask instead.
2. Plan paths that don't cross any mission or obstacle polygon, allowing for the
   robot's footprint (`robot.footprint_in`) plus 1 in of clearance.
3. Re-align (square to a wall or line) after long drives or before precise missions.
4. Default speeds: 8 in/s for travel, 4 in/s near models. Turns at 90°/s.
5. Comment each step with the mission ID and the expected pose after the step.
6. Keep each run under the match time budget and end back in a launch area.

## Mission file template
```python
from alliedwork import *

def run():
    set_pose(x=0, y=0, heading=0)   # home_left default_start_pose
    mission("M00")
    # M00_example: drive to approach pose (43, 16), heading 0
    drive_to(43, 16, 8)
    turn_to(0, 90)
    drive(4, 4)                     # push into model
    drive(-4, 4)
```

## Current robot measurements
Kept in `work/field/missions.json` → `robot`. If they're still 0 or TODO, ask before writing code
that depends on them.
