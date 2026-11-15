"""alliedwork.py: everything the robot needs, in one file.

Missions only need:   from alliedwork import *

Units: INCHES, DEGREES, inches/second.
Map:   (0, 0) = bottom-left of the mat. +x right, +y up (far wall).
Angle: 0 = facing +y, positive = clockwise (right), 90 = facing +x.

Commands (blocking: each one finishes before the next starts):
    set_pose(x, y, heading)     tell the robot where it is (start of every run)
    drive(dist, speed=8)        + forward / - backward
    turn(angle, speed=90)       + right (clockwise) / - left
    turn_to(heading, speed=90)  face an absolute heading
    drive_to(x, y, speed=8)     turn toward the point, then drive there
    square_to_wall(heading, x=None, y=None, speed=3, back=True)
    attachment(port, angle, speed=300)      relative move, port "A" or "E"
    attachment_to(port, angle, speed=300)   absolute move
    wait(ms), beep(), log(text), mission(id), stop()
Menu:     run_menu(missions)  (used by the generated main.py: pick with arrows, start with center)
Sensors:  heading(), pose(), color(), reflection()

How it works: a PID loop runs every 10 ms inside each command. It estimates
x, y, heading (wheel encoders + gyro), then steers a gyro-assisted DriveBase
so the robot goes where you asked. If the robot stops making progress for
1.5 s (stuck / pushed into a wall) the command gives up, logs STALLED and
returns False, so the run keeps going.
"""
from math import atan2, cos, degrees, radians, sin, sqrt

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

# ============================================================================
# 1. SETTINGS: change these for your robot
# ============================================================================
LEFT_PORT, LEFT_DIR = Port.F, Direction.COUNTERCLOCKWISE
RIGHT_PORT, RIGHT_DIR = Port.B, Direction.CLOCKWISE
ATTACHMENTS = {"A": Port.A, "E": Port.E}      # name -> port
COLOR_PORTS = (Port.C, Port.D)                # where a color sensor may be

WHEEL_DIAMETER_MM = 56
AXLE_TRACK_MM = 112

# PID gains (tune on the real robot)
DIST_KP, DIST_KI, DIST_KD = 3.0, 0.0, 0.1     # error in -> speed in/s
HEAD_KP, HEAD_KI, HEAD_KD = 4.0, 0.0, 0.1     # error deg -> turn rate deg/s
CROSS_TRACK_GAIN = 3.0                        # deg of steering per inch off the line
CROSS_TRACK_MAX = 10.0

# Limits
MAX_SPEED = 14.0       # in/s
MAX_ACCEL = 30.0       # in/s^2
MAX_TURN_RATE = 180.0  # deg/s
MAX_TURN_ACCEL = 720.0  # deg/s^2

# When is a move "done" / "stuck"
POS_TOL = 0.25         # inches
HEAD_TOL = 1.0         # degrees
SETTLE_MS = 100
STALL_MS = 1500
CONTROL_MS = 10
TELEMETRY_MS = 200     # prints "T,ms,x,y,heading"; 0 = off

VERSION = "0.2.00"
MM_PER_IN = 25.4

# ============================================================================
# 2. MATH HELPERS
# ============================================================================


def wrap_angle(a):
    """Wrap degrees into (-180, 180]."""
    a = a % 360
    if a > 180:
        a -= 360
    return a


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def sign(v):
    return 1 if v > 0 else -1 if v < 0 else 0


def distance(x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    return sqrt(dx * dx + dy * dy)


def heading_to_point(x0, y0, x1, y1):
    """Heading (0 = +y, clockwise +) that faces (x1, y1) from (x0, y0)."""
    return degrees(atan2(x1 - x0, y1 - y0))


def move_point(x, y, heading_deg, dist):
    """Point reached by moving dist along a heading (dx = d*sin h, dy = d*cos h)."""
    h = radians(heading_deg)
    return x + dist * sin(h), y + dist * cos(h)


def waypoint_step(x0, y0, h0, x1, y1):
    """From a pose to a point: returns (turn_deg, dist_in)."""
    d = distance(x0, y0, x1, y1)
    if d < 1e-9:
        return 0.0, 0.0
    return wrap_angle(heading_to_point(x0, y0, x1, y1) - h0), d


class PID:
    """PID with anti-windup and output clamp."""

    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def step(self, err, dt):
        self.integral = clamp(self.integral + self.ki * err * dt, -self.limit, self.limit)
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return clamp(self.kp * err + self.integral + self.kd * d, -self.limit, self.limit)


# ============================================================================
# 3. HARDWARE SETUP
# ============================================================================
hub = PrimeHub()
_left = Motor(LEFT_PORT, LEFT_DIR)
_right = Motor(RIGHT_PORT, RIGHT_DIR)
_base = DriveBase(_left, _right, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
_base.use_gyro(True)  # gyro keeps turns and straight lines accurate
_base.settings(MAX_SPEED * MM_PER_IN, MAX_SPEED * MM_PER_IN * 3,
               MAX_TURN_RATE, MAX_TURN_RATE * 4)
_attach = {}
for _name, _port in ATTACHMENTS.items():
    try:
        _attach[_name] = Motor(_port)
    except OSError:
        pass  # no motor on that port: using it later gives a clear error
_color = None
for _p in COLOR_PORTS:
    try:
        _color = ColorSensor(_p)
        break
    except OSError:
        pass

_clock = StopWatch()
_x = _y = 0.0
_h = 0.0              # estimated heading
_goal_h = 0.0         # heading we last commanded (so turns don't accumulate error)
_last_dist_mm = 0
_last_tel = -10000

# ============================================================================
# 4. POSE (where am I?)
# ============================================================================


def _update_pose():
    """Read encoders + gyro, update x, y, heading. Call every control tick."""
    global _x, _y, _h, _last_dist_mm
    mm = _base.distance()
    d = (mm - _last_dist_mm) / MM_PER_IN
    _last_dist_mm = mm
    h_new = wrap_angle(hub.imu.heading())
    h_mid = _h + wrap_angle(h_new - _h) / 2  # midpoint heading for the step
    _x += d * sin(radians(h_mid))
    _y += d * cos(radians(h_mid))
    _h = h_new
    if TELEMETRY_MS and _clock.time() - _last_tel >= TELEMETRY_MS:
        _telemetry()


def _telemetry():
    global _last_tel
    _last_tel = _clock.time()
    print("T,{},{:.2f},{:.2f},{:.1f}".format(_last_tel, _x, _y, _h))


def set_pose(x, y, heading=0):
    """Tell the robot where it is. Do this first in every run."""
    global _x, _y, _h, _goal_h, _last_dist_mm
    hub.imu.reset_heading(heading)
    _base.reset()
    _last_dist_mm = 0
    _x, _y, _h, _goal_h = float(x), float(y), wrap_angle(heading), wrap_angle(heading)


def pose():
    """(x, y, heading) the robot thinks it is at."""
    _update_pose()
    return _x, _y, _h


def heading():
    _update_pose()
    return _h


# ============================================================================
# 5. MOTION
# ============================================================================


def _check(name, speed, max_speed=None):
    """Clear errors for bad speeds (max_speed only for in/s commands)."""
    if not speed > 0:
        raise ValueError("{}(): speed must be > 0, got {}".format(name, speed))
    if max_speed is not None and speed > max_speed:
        raise ValueError("{}(): speed {} is too fast (max {})".format(name, speed, max_speed))


def _stop():
    _base.brake()


def stop():
    """Stop both drive motors right now."""
    _stop()


def _progress_watch():
    return [_clock.time(), _x, _y, _h]


def _stalled(mark, moving):
    """True if we've been told to move but haven't moved for STALL_MS."""
    now = _clock.time()
    moved = distance(mark[1], mark[2], _x, _y) > 0.1 or abs(wrap_angle(_h - mark[3])) > 1.0
    if moved or not moving:
        mark[0], mark[1], mark[2], mark[3] = now, _x, _y, _h
        return False
    return now - mark[0] > STALL_MS


def drive(dist, speed=8):
    """Drive dist inches along the current heading. Returns True, or False if stuck."""
    _check("drive", speed, MAX_SPEED)
    h_hold = _goal_h
    sgn = 1 if dist >= 0 else -1
    _update_pose()
    sx, sy = _x, _y
    x1, y1 = move_point(sx, sy, h_hold, dist)
    dx, dy = sin(radians(h_hold)) * sgn, cos(radians(h_hold)) * sgn
    pid_d = PID(DIST_KP, DIST_KI, DIST_KD, MAX_SPEED)
    pid_h = PID(HEAD_KP, HEAD_KI, HEAD_KD, MAX_TURN_RATE)
    mark = _progress_watch()
    t_end = _clock.time() + int((abs(dist) / speed * 3 + 3) * 1000)
    settled_since = None
    vramp = 0.0
    dt = CONTROL_MS / 1000
    while True:
        _update_pose()
        rem = (x1 - _x) * dx + (y1 - _y) * dy            # inches left along the path
        ct = (_x - sx) * cos(radians(h_hold)) - (_y - sy) * sin(radians(h_hold))
        aim = h_hold - sgn * clamp(CROSS_TRACK_GAIN * ct, -CROSS_TRACK_MAX, CROSS_TRACK_MAX)
        vramp = min(speed, vramp + MAX_ACCEL * dt)
        vlim = min(vramp, sqrt(2 * MAX_ACCEL * abs(rem)))
        v = clamp(pid_d.step(rem, dt), -vlim, vlim)       # speed along the path
        w = pid_h.step(wrap_angle(aim - _h), dt)
        _base.drive(sgn * v * MM_PER_IN, w)               # sgn: backward if dist < 0
        if abs(rem) < POS_TOL:
            settled_since = settled_since or _clock.time()
            if _clock.time() - settled_since >= SETTLE_MS:
                break
        else:
            settled_since = None
        if _stalled(mark, abs(v) > 0.5):
            _stop()
            _event("STALLED", "drive")
            return False
        if _clock.time() > t_end:
            _stop()
            _event("TIMEOUT", "drive")
            return False
        wait(CONTROL_MS)
    _stop()
    return True


def turn_to(target, speed=90):
    """Turn in place to face an absolute heading. Returns True, or False if stuck."""
    _check("turn_to", speed)
    global _goal_h
    _goal_h = wrap_angle(target)
    pid = PID(HEAD_KP, HEAD_KI, HEAD_KD, MAX_TURN_RATE)
    rate_cap = min(speed, MAX_TURN_RATE)
    ramp = 0.0
    dt = CONTROL_MS / 1000
    mark = _progress_watch()
    settled_since = None
    t_end = _clock.time() + 8000
    while True:
        _update_pose()
        err = wrap_angle(_goal_h - _h)
        ramp = min(rate_cap, ramp + MAX_TURN_ACCEL * dt)
        lim = min(ramp, sqrt(2 * MAX_TURN_ACCEL * abs(err)))
        w = clamp(pid.step(err, dt), -lim, lim)
        _base.drive(0, w)
        if abs(err) < HEAD_TOL:
            settled_since = settled_since or _clock.time()
            if _clock.time() - settled_since >= SETTLE_MS:
                break
        else:
            settled_since = None
        if _stalled(mark, abs(w) > 5):
            _stop()
            _event("STALLED", "turn")
            return False
        if _clock.time() > t_end:
            _stop()
            _event("TIMEOUT", "turn")
            return False
        wait(CONTROL_MS)
    _stop()
    return True


def turn(angle, speed=90):
    """Turn by angle degrees. + = right (clockwise), - = left."""
    _check("turn", speed)
    return turn_to(_goal_h + angle, speed)


def drive_to(x, y, speed=8):
    """Turn toward (x, y), then drive there."""
    _check("drive_to", speed, MAX_SPEED)
    _update_pose()
    t, d = waypoint_step(_x, _y, _goal_h, x, y)
    if d < POS_TOL:
        return True
    if abs(t) > HEAD_TOL and not turn_to(_goal_h + t):
        return False
    return drive(d, speed)


def square_to_wall(heading, x=None, y=None, speed=3, back=True):
    """Drive slowly into a wall until stuck, then fix heading (and x / y).

    heading: the heading the robot has when it sits flat against the wall.
    x, y:    the robot's coordinate along the wall's axis (give the one that
             the wall fixes, e.g. y=3 when backing into the south wall with
             3 in between the axle and the back of the robot).
    """
    _check("square_to_wall", speed, MAX_SPEED)
    v = -speed if back else speed
    t0 = _clock.time()
    ref = (_x, _y, t0)
    while _clock.time() - t0 < 6000:
        _update_pose()
        _base.drive(v * MM_PER_IN, 0)
        now = _clock.time()
        if now - ref[2] >= 300:  # moved less than 0.05 in in 300 ms = touching the wall
            if now - t0 > 500 and distance(ref[0], ref[1], _x, _y) < 0.05:
                break
            ref = (_x, _y, now)
        wait(CONTROL_MS)
    _stop()
    _update_pose()
    set_pose(_x if x is None else x, _y if y is None else y, heading)
    _event("SQUARED", "{},{}".format(x, y))


# ============================================================================
# 6. ATTACHMENTS, SENSORS, MISC
# ============================================================================


def _motor(port):
    name = port.name if hasattr(port, "name") else str(port).upper()[-1]
    if name not in _attach:
        raise ValueError("attachment(): no motor on port {} (have: {})".format(
            name, ", ".join(sorted(_attach)) or "none"))
    return _attach[name]


def attachment(port, angle, speed=300):
    """Turn an attachment motor by angle degrees (relative)."""
    _check("attachment", speed)
    _motor(port).run_angle(speed, angle, Stop.HOLD, True)


def attachment_to(port, angle, speed=300):
    """Turn an attachment motor to an absolute angle."""
    _check("attachment_to", speed)
    _motor(port).run_target(speed, angle, Stop.HOLD, True)


def color():
    """Color name under the sensor, e.g. 'RED' (or None if no sensor)."""
    if _color is None:
        return None
    c = _color.color()
    return str(c).split(".")[-1]


def reflection():
    """Reflected light 0-100 (or None if no sensor)."""
    return None if _color is None else _color.reflection()


def beep():
    hub.speaker.beep()


def _event(kind, detail=""):
    print("E,{},{},{}".format(_clock.time(), kind, detail))


def log(text):
    """Show a message in the terminal / dashboard."""
    print("L,{},{}".format(_clock.time(), text))


def mission(mission_id):
    """Mark the start of a mission (shows in the log)."""
    _event("MISSION", mission_id)


# ============================================================================
# 7. HUB MENU: pick a mission with the arrow buttons, start with the center
# ============================================================================


def _stop_button(button):
    """Which hub button stops the program (the menu needs CENTER for itself)."""
    try:
        hub.system.set_stop_button(button)
    except Exception:
        pass


def _wait_press():
    """Wait for a fresh button press; returns the set of buttons pressed."""
    while hub.buttons.pressed():  # let go of the last press first
        wait(20)
    while True:
        pressed = hub.buttons.pressed()
        if pressed:
            wait(30)  # debounce
            return pressed
        wait(20)


def run_menu(missions):
    """Show mission numbers on the hub. LEFT / RIGHT choose, CENTER starts.

    missions: list of (number, name, function). While a mission runs, pressing
    CENTER stops the program (press CENTER again to bring the menu back).
    """
    if not missions:
        raise ValueError("run_menu(): no missions found (make hardware/missions/run1.py)")
    missions = sorted(missions, key=lambda m: m[0])
    log("alliedwork " + VERSION)
    n = len(missions)
    i = 0
    while True:
        _stop_button(Button.BLUETOOTH)  # frees CENTER for the menu
        hub.light.on(Color.GREEN)
        hub.display.number(missions[i][0])
        pressed = _wait_press()
        if Button.RIGHT in pressed:
            i = (i + 1) % n
        elif Button.LEFT in pressed:
            i = (i - 1) % n
        elif Button.CENTER in pressed:
            number, name, func = missions[i]
            _stop_button(Button.CENTER)  # now CENTER = emergency stop
            hub.light.on(Color.ORANGE)
            _event("RUN_START", name)
            hub.speaker.beep(800, 100)
            try:
                func()
                _event("RUN_END", name)
                hub.speaker.beep(1000, 150)
            except Exception as e:  # a bug in a mission must not kill the menu
                _stop()
                _event("ERROR", "{}: {}".format(name, e))
                hub.light.on(Color.RED)
                hub.speaker.beep(200, 400)
                wait(1500)
            _stop()


__all__ = [
    "set_pose", "drive", "turn", "turn_to", "drive_to", "square_to_wall",
    "attachment", "attachment_to", "wait", "beep", "log", "mission", "stop",
    "pose", "heading", "color", "reflection", "hub", "Port",
]
