# Command reference (`from alliedwork import *`)

Units: inches, degrees, inches/second (attachments: degrees, degrees/second).
Heading: 0 = +y (far wall), clockwise positive. All motion commands **block**: the next line
runs after the move finishes. Motion commands return `True` when done, `False` if they gave up
(stalled or timed out).

## Where am I?

### `set_pose(x, y, heading=0)`
Tell the robot its position and heading. Resets the gyro heading and the distance counter.
**Call it first in every run.** Returns nothing.

### `pose()` → `(x, y, heading)`
The robot's own estimate (encoders + gyro). Heading is wrapped to (−180, 180].

### `heading()` → degrees

## Driving

### `drive(dist, speed=8)` → `bool`
Drive `dist` inches along the **last commanded heading**. Positive = forward, negative = backward.
`speed` in in/s, must be `> 0` and `≤ MAX_SPEED` (14). Corrects sideways drift (cross-track) and
holds heading with the gyro. Ends when within 0.25 in for 100 ms.
Gives up (`False`) after 1.5 s without progress (`E,...,STALLED,drive`) or after
`dist/speed × 3 + 3` seconds (`TIMEOUT`).

### `turn(angle, speed=90)` → `bool`
Turn in place by `angle` degrees. **+ = right (clockwise), − = left.** Relative to the last
**commanded** heading (not where the robot happens to be), so small errors don't pile up over
many turns. `speed` in deg/s (> 0, capped at 180).

### `turn_to(heading, speed=90)` → `bool`
Turn in place to an absolute heading. Takes the short way round. Ends within 1° for 100 ms.
Gives up after 1.5 s without turning or after 8 s.

### `drive_to(x, y, speed=8)` → `bool`
Turn toward the point, then drive there. Returns `True` immediately if already within 0.25 in.

### `square_to_wall(heading, x=None, y=None, speed=3, back=True)`
Creep into a wall (backward by default) until the robot stops moving, then fix the pose:
- `heading`: the heading the robot has when it sits flat against that wall.
- `x`, `y`: the coordinate the wall fixes. Give one (e.g. `y=3` when backing into the south wall
  with 3 in between the axle and the robot's back). Leave the other `None` to keep the estimate.
Gives up after 6 s. Prints `E,...,SQUARED,x,y`.

### `stop()`
Brake both drive motors now.

## Attachments

### `attachment(port, angle, speed=300)`
Turn an attachment motor by `angle` degrees (relative). `port` is `"A"` or `"E"` (or `Port.A`).
Blocks until finished, then holds position.

### `attachment_to(port, angle, speed=300)`
Turn to an absolute angle (relative to where the motor was at program start).

An unknown port raises `ValueError: attachment(): no motor on port B (have: A, E)`.

## Sensors

| Call | Returns |
|---|---|
| `color()` | name under the color sensor: `"RED"`, `"WHITE"`, … or `"NONE"`; `None` if no sensor found |
| `reflection()` | reflected light 0–100, or `None` |
| `hub` | the Pybricks `PrimeHub` object (for anything else, e.g. `hub.battery.voltage()`) |

The color sensor is auto-detected on ports C and D (`COLOR_PORTS`).

## Helpers

| Call | Does |
|---|---|
| `wait(ms)` | pause (Pybricks `wait`) |
| `beep()` | short beep |
| `log(text)` | print `L,ms,text` (shows in the terminal) |
| `mission(id)` | print `E,ms,MISSION,id` to mark a mission step |

## For library authors (not exported to missions)
`run_menu(missions)`, `wrap_angle`, `clamp`, `sign`, `distance`, `heading_to_point`,
`move_point`, `waypoint_step`, `PID`, `VERSION`. Import them explicitly: `from alliedwork import wrap_angle`.

## Errors you may see
| Message | Meaning |
|---|---|
| `drive(): speed must be > 0, got 0` | speeds must be positive; use a negative **distance** to go backward |
| `drive(): speed 99 is too fast (max 14.0)` | over `MAX_SPEED` |
| `attachment(): no motor on port B (have: A, E)` | wrong port letter or motor unplugged |
| `run_menu(): no missions found ...` | no `runN.py` files in `hardware/missions/` |
