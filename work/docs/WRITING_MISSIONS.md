# Writing missions

A mission ("run") is one Python file in `hardware/missions/` named **`runN.py`**.
The **N is the number shown on the robot's light matrix.**

```python
from alliedwork import *

def run():
    set_pose(x=30, y=8, heading=0)    # 1. say where the robot starts
    mission("M01")                    # 2. (optional) label for the log
    drive_to(43, 16, 8)               # 3. go to the model...
    turn_to(0)                        #    ...face it...
    drive(4, 4)                       #    ...push slowly
    drive(-4, 4)
    attachment("A", 90)               # 4. move an attachment motor
```

## The map in your head
- Units: **inches**, **degrees**, **inches per second**.
- `(0, 0)` is the **bottom-left** corner of the mat as seen from your launch side.
  **x goes right, y goes up** (toward the far wall).
- **Heading 0** = facing the far wall (+y). **Positive = clockwise.** 90 = facing right,
  -90 = facing left, 180 = facing you.
- `turn(90)` turns **right**. `turn(-90)` turns **left**.

## Rules that keep runs reliable
1. **Start with `set_pose`**, using the launch area's start pose. The robot only knows where it is because you told it.
2. **Check the field file before using a coordinate.** Use a mission's `approach` pose from `work/field/missions.json`; don't guess.
3. **Slow down near models:** 8 in/s to travel, 4 in/s when touching things, turns at 90°/s.
4. **Re-align** after long drives or before precise missions: `square_to_wall(...)` against a wall fixes heading and one coordinate.
5. Comment every step with the mission and where the robot should be after it.
6. End back in a launch area, inside the match time.
7. Keep paths away from models: the robot is wider than its center point (see `robot.footprint_in`), leave ~1 in clearance.

## Doing waypoints by hand
From pose (x0, y0) facing h0 to point (x1, y1):
`distance = sqrt(dx² + dy²)`, `target heading = atan2(dx, dy)` in degrees, `turn = target − h0` wrapped to (−180, 180].
`drive_to(x1, y1)` does exactly this for you.

## When something goes wrong in a run
- A command that makes **no progress for 1.5 s** (stuck, pushed into a wall) stops, prints `STALLED`, and **returns `False`**, so the run continues. You can react: `if not drive(10, 4): ...`.
- A command that takes far too long prints `TIMEOUT` and returns `False`.
- A bug (exception) in a run is caught by the menu: red light, low beep, menu returns.
- Typos give a plain message, e.g. `drive(): speed must be > 0, got 0`.

## Try before you drive
```
python run.py run3 --sim       # fake robot, prints the final true position
python run.py run3             # upload and run just this run
python run.py                  # upload all runs + the menu (competition flow)
```

## Adding your own command
Write a normal function in `hardware/alliedwork.py` (it can call `drive`, `attachment`, ...)
and add its name to `__all__` at the bottom so missions can use it:

```python
def drop_cargo():
    attachment("A", -90, 400)
    wait(300)
    attachment("A", 90, 400)
```
