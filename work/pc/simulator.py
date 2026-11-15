"""Kinematic robot simulator with fake ``pybricks`` modules.

Lets the code in ``robot/`` and ``missions/`` run unchanged on the PC.

Usage::

    world = SimWorld(SimConfig())
    world.install()            # puts fake pybricks.* into sys.modules
    from pybricks.pupdevices import Motor   # now the fake one
    ...
    world.uninstall()

Time is virtual (1 ms physics ticks). ``realtime=False`` runs as fast as
possible (tests); ``realtime=True`` sleeps so sim time matches the wall clock.

Model: differential drive. Positive motor angle = wheel rolls forward (we
assume the team chose motor directions so this is true, as on the real robot).
Walls: a step that pushes the robot footprint deeper into a mat edge is
rejected and the wheels stall, like a real robot against a wall.
"""
from __future__ import annotations

import inspect
import json
import math
import random
import sys
import time
import types
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Optional

MM_PER_IN = 25.4
TICK_MS = 1


# --------------------------------------------------------------------------
# pybricks.parameters
# --------------------------------------------------------------------------
class Port(Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"
    F = "F"


class Direction(Enum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Stop(Enum):
    NONE = 0
    COAST = 1
    COAST_SMART = 2
    BRAKE = 3
    HOLD = 4


class Button(Enum):
    LEFT = 1
    RIGHT = 2
    CENTER = 3
    BLUETOOTH = 4


class Axis(Enum):
    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)


class Color(Enum):
    NONE = 0
    BLACK = 1
    GRAY = 2
    WHITE = 3
    RED = 4
    ORANGE = 5
    YELLOW = 6
    GREEN = 7
    CYAN = 8
    BLUE = 9
    VIOLET = 10
    MAGENTA = 11


# --------------------------------------------------------------------------
# Config + world
# --------------------------------------------------------------------------
@dataclass
class SimConfig:
    """All simulator knobs. Distances in inches, angles in degrees."""

    mat_width_in: float = 93.0
    mat_height_in: float = 45.0
    wheel_diameter_mm: float = 56.0
    axle_track_mm: float = 112.0
    # Footprint relative to the drive-axle midpoint (placeholder until measured).
    robot_width_in: float = 6.0
    robot_front_in: float = 5.0
    robot_rear_in: float = 3.0
    start_x_in: float = 0.0
    start_y_in: float = 0.0
    start_heading_deg: float = 0.0
    left_port: Port = Port.F
    right_port: Port = Port.B
    color_port: Optional[Port] = Port.C  # None = no color sensor plugged in
    slip_pct: float = 0.0  # average % of wheel travel lost to slipping
    gyro_drift_deg_per_min: float = 0.0
    gyro_noise_deg: float = 0.0
    seed: int = 1
    max_motor_speed_dps: float = 1000.0
    motor_tau_ms: float = 30.0  # speed response lag
    motor_accel_dps2: float = 4000.0  # decel used when stopping on an angle
    battery_mv: int = 8000
    realtime: bool = False
    max_runtime_ms: int = 600_000  # safety net against endless loops


class _Task:
    def __init__(self, coro: Any, now: int) -> None:
        self.coro = coro
        self.wake = now
        self.pred: Optional[Callable[[], bool]] = None
        self.done = False
        self.result: Any = None
        self.exc: Optional[BaseException] = None


async def _wrap(aw: Any) -> Any:
    return await aw


class SimWorld:
    """The simulated robot, mat and clock."""

    current: Optional["SimWorld"] = None

    def __init__(self, cfg: Optional[SimConfig] = None) -> None:
        self.cfg = cfg or SimConfig()
        c = self.cfg
        self.rng = random.Random(c.seed)
        self.t_ms = 0
        self.x = c.start_x_in
        self.y = c.start_y_in
        self.heading = c.start_heading_deg  # true heading, unwrapped
        self.motors: dict[Port, "Motor"] = {}
        self.hooks: list[Callable[[float], None]] = []  # run each tick before motors move
        self.tasks: list[_Task] = []
        self.in_task = False
        self.blocked = False  # True if the last step was stopped by a wall
        self.events: list[tuple[int, str, Any]] = []
        self.pressed: set[Button] = set()
        self.light: Any = None
        self.display_text: Any = None
        self.stop_button: Any = None
        self.imu_offset = -c.start_heading_deg  # hub reads 0 at boot
        self.yaw_rate = 0.0  # true deg/s, for imu.angular_velocity
        self._wall_t0 = time.monotonic()
        self._circ_in = math.pi * c.wheel_diameter_mm / MM_PER_IN
        self._track_in = c.axle_track_mm / MM_PER_IN

    # -- construction helpers ------------------------------------------------
    @classmethod
    def from_field(cls, path: str, **overrides: Any) -> "SimWorld":
        """Build a world from field/missions.json (mat + robot sections)."""
        with open(path) as f:
            data = json.load(f)
        mat, robot = data.get("mat", {}), data.get("robot", {})
        kw: dict[str, Any] = {}
        if "width_in" in mat:
            kw["mat_width_in"] = mat["width_in"]
        if "height_in" in mat:
            kw["mat_height_in"] = mat["height_in"]
        if robot.get("wheel_diameter_mm"):
            kw["wheel_diameter_mm"] = robot["wheel_diameter_mm"]
        if robot.get("axle_track_mm"):
            kw["axle_track_mm"] = robot["axle_track_mm"]
        fp = robot.get("footprint_in", {})
        if fp.get("width", 0) > 0 and fp.get("length", 0) > 0:
            kw["robot_width_in"] = fp["width"]
            kw["robot_front_in"] = fp["length"] / 2
            kw["robot_rear_in"] = fp["length"] / 2
        kw.update(overrides)
        return cls(SimConfig(**kw))

    def install(self) -> "SimWorld":
        """Make this the active world and expose fake pybricks modules."""
        SimWorld.current = self
        sys.modules.update(_FAKE_MODULES)
        return self

    def uninstall(self) -> None:
        if SimWorld.current is self:
            SimWorld.current = None
        for name in _FAKE_MODULES:
            sys.modules.pop(name, None)

    # -- state ---------------------------------------------------------------
    @property
    def pose(self) -> tuple[float, float, float]:
        """True (x, y, heading) with heading wrapped to (-180, 180]."""
        h = self.heading % 360
        return self.x, self.y, h - 360 if h > 180 else h

    def measured_heading(self) -> float:
        """What the gyro reports: truth + drift + noise (unwrapped, like Pybricks)."""
        c = self.cfg
        h = self.heading + c.gyro_drift_deg_per_min * self.t_ms / 60000.0 + self.imu_offset
        if c.gyro_noise_deg:
            h += self.rng.gauss(0, c.gyro_noise_deg)
        return h

    def log_event(self, kind: str, detail: Any = None) -> None:
        self.events.append((self.t_ms, kind, detail))

    # -- physics -------------------------------------------------------------
    def _penetration(self, x: float, y: float, h_deg: float) -> tuple:
        """How far (inches) the footprint sticks out past each edge (left, right, bottom, top)."""
        c = self.cfg
        h = math.radians(h_deg)
        sh, ch = math.sin(h), math.cos(h)
        hw = c.robot_width_in / 2
        l = r = b = t = 0.0
        for f in (c.robot_front_in, -c.robot_rear_in):
            for s in (hw, -hw):
                px = x + f * sh + s * ch
                py = y + f * ch - s * sh
                l, r = max(l, -px), max(r, px - c.mat_width_in)
                b, t = max(b, -py), max(t, py - c.mat_height_in)
        return (l, r, b, t)

    def _physics_tick(self) -> None:
        c = self.cfg
        dt = TICK_MS / 1000.0
        for hook in self.hooks:
            hook(dt)
        deltas = {m: m._integrate(dt) for m in self.motors.values()}
        left, right = self.motors.get(c.left_port), self.motors.get(c.right_port)
        self.blocked = False
        self.yaw_rate = 0.0
        if left and right:
            k = self._circ_in / 360.0
            dl, dr = deltas[left] * k, deltas[right] * k
            if c.slip_pct:
                dl *= 1 - c.slip_pct / 100.0 * self.rng.uniform(0, 2)
                dr *= 1 - c.slip_pct / 100.0 * self.rng.uniform(0, 2)
            dh = math.degrees((dl - dr) / self._track_in)  # right wheel faster = turn left
            hmid = math.radians(self.heading + dh / 2)
            d = (dl + dr) / 2
            nx, ny = self.x + d * math.sin(hmid), self.y + d * math.cos(hmid)
            nh = self.heading + dh
            self.yaw_rate = dh / dt
            old_pen = self._penetration(self.x, self.y, self.heading)
            new_pen = self._penetration(nx, ny, nh)
            if any(n > max(o, 1e-9) + 1e-9 for n, o in zip(new_pen, old_pen)):
                left._undo(deltas[left])  # wall: wheels stall
                right._undo(deltas[right])
                self.blocked = True
            else:
                self.x, self.y, self.heading = nx, ny, nh
        self.t_ms += TICK_MS
        if c.realtime:
            ahead = self.t_ms / 1000.0 - (time.monotonic() - self._wall_t0)
            if ahead > 0.002:
                time.sleep(ahead)

    def advance(self, ms: float) -> None:
        """Blocking: run physics for ms (used when code is not inside run_task)."""
        for _ in range(int(ms)):
            self._physics_tick()

    def advance_until(self, pred: Callable[[], bool]) -> None:
        start = self.t_ms
        while not pred():
            self._physics_tick()
            if self.t_ms - start > self.cfg.max_runtime_ms:
                raise TimeoutError("Simulation ran too long (stuck waiting?).")

    # -- cooperative scheduler (multitask / run_task) -------------------------
    def spawn(self, aw: Any) -> _Task:
        coro = aw if inspect.iscoroutine(aw) else _wrap(aw)
        task = _Task(coro, self.t_ms)
        self.tasks.append(task)
        return task

    def _resume(self, task: _Task) -> None:
        try:
            req = task.coro.send(None)
        except StopIteration as e:
            task.done, task.result = True, e.value
            return
        except BaseException as e:  # stored; re-raised by whoever awaits it
            task.done, task.exc = True, e
            return
        if isinstance(req, tuple) and req[0] == "sleep":
            task.wake = self.t_ms + max(0, int(req[1]))
        elif isinstance(req, tuple) and req[0] == "until":
            task.pred = req[1]
        else:
            task.wake = self.t_ms

    def _run_tasks(self) -> None:
        self.in_task = True
        try:
            for task in list(self.tasks):
                if task.done:
                    continue
                if task.pred is not None:
                    if not task.pred():
                        continue
                    task.pred = None
                elif task.wake > self.t_ms:
                    continue
                self._resume(task)
        finally:
            self.in_task = False
        self.tasks = [t for t in self.tasks if not t.done]

    def cancel(self, task: _Task) -> None:
        if not task.done:
            task.done = True
            task.coro.close()

    def run_task(self, main: Any, loop_callback: Optional[Callable[[], Any]] = None) -> Any:
        """Run main (and anything it multitasks) until main finishes."""
        task = self.spawn(main)
        start = self.t_ms
        while not task.done:
            self._run_tasks()
            if task.done:
                break
            self._physics_tick()
            if loop_callback is not None and loop_callback():
                break
            if self.t_ms - start > self.cfg.max_runtime_ms:
                for t in list(self.tasks):
                    self.cancel(t)
                raise TimeoutError("Simulation ran too long (endless loop in the mission?).")
        for t in list(self.tasks):
            self.cancel(t)
        self.tasks = []
        if task.exc is not None:
            raise task.exc
        return task.result


def _world() -> SimWorld:
    w = SimWorld.current
    if w is None:
        raise RuntimeError("No simulator running: call SimWorld().install() first.")
    return w


# --------------------------------------------------------------------------
# Awaitables (work both awaited inside run_task and called blocking outside)
# --------------------------------------------------------------------------
class _Ready:
    def __await__(self):
        return iter(())


class _Sleep:
    def __init__(self, ms: float) -> None:
        self.ms = ms

    def __await__(self):
        yield ("sleep", self.ms)


class _Until:
    def __init__(self, pred: Callable[[], bool]) -> None:
        self.pred = pred

    def __await__(self):
        yield ("until", self.pred)


def _wait_until(pred: Callable[[], bool]) -> Any:
    w = _world()
    if w.in_task:
        return _Until(pred)
    w.advance_until(pred)
    return _Ready()


def wait(time_ms: float) -> Any:
    """pybricks.tools.wait: blocks outside run_task, awaitable inside."""
    w = _world()
    if w.in_task:
        return _Sleep(time_ms)
    w.advance(time_ms)
    return _Ready()


class _Multi:
    def __init__(self, aws: tuple, race: bool) -> None:
        self.aws, self.race = aws, race

    def __await__(self):
        w = _world()
        tasks = [w.spawn(a) for a in self.aws]
        race = self.race

        def finished() -> bool:
            if any(t.exc is not None for t in tasks):
                return True
            return any(t.done for t in tasks) if race else all(t.done for t in tasks)

        yield ("until", finished)
        for t in tasks:
            w.cancel(t)
        for t in tasks:
            if t.exc is not None:
                raise t.exc
        return tuple(t.result for t in tasks)


def multitask(*awaitables: Any, race: bool = False) -> Any:
    """pybricks.tools.multitask: run awaitables at the same time."""
    return _Multi(awaitables, race)


def run_task(main: Any, loop_callback: Optional[Callable[[], Any]] = None) -> Any:
    """pybricks.tools.run_task."""
    return _world().run_task(main, loop_callback)


class StopWatch:
    """pybricks.tools.StopWatch on sim time."""

    def __init__(self) -> None:
        self._w = _world()
        self._acc = 0
        self._t0: Optional[int] = self._w.t_ms

    def time(self) -> int:
        return self._acc + (0 if self._t0 is None else self._w.t_ms - self._t0)

    def pause(self) -> None:
        if self._t0 is not None:
            self._acc += self._w.t_ms - self._t0
            self._t0 = None

    def resume(self) -> None:
        if self._t0 is None:
            self._t0 = self._w.t_ms

    def reset(self) -> None:
        self._acc = 0
        self._t0 = None if self._t0 is None else self._w.t_ms


# --------------------------------------------------------------------------
# pybricks.pupdevices.Motor
# --------------------------------------------------------------------------
class Motor:
    """Fake SPIKE motor. Angles in degrees, speeds in deg/s."""

    def __init__(self, port: Port, positive_direction: Direction = Direction.CLOCKWISE,
                 gears: Any = None, reset_angle: bool = True, profile: Any = None) -> None:
        self._w = _world()
        self.port = port
        self.direction = positive_direction
        self._w.motors[port] = self
        self._angle = 0.0
        self._speed = 0.0
        self._mode = "coast"  # coast brake hold run angle time track
        self._cmd = 0.0
        self._target = 0.0
        self._end_ms = 0
        self._then = Stop.HOLD

    # -- sim internals --
    def _integrate(self, dt: float) -> float:
        cfg = self._w.cfg
        old = self._angle
        mode = self._mode
        desired = 0.0
        if mode == "run":
            desired = self._cmd
        elif mode in ("angle", "track"):
            err = self._target - self._angle
            vmax = self._cmd if mode == "angle" else cfg.max_motor_speed_dps
            desired = math.copysign(min(vmax, math.sqrt(2 * cfg.motor_accel_dps2 * abs(err))), err)
            if mode == "angle" and abs(err) < 1.0 and abs(self._speed) < 50:
                self._angle, self._speed = self._target, 0.0
                self._finish()
                return self._angle - old
        elif mode == "time":
            desired = self._cmd
            if self._w.t_ms >= self._end_ms:
                self._finish()
        lim = cfg.max_motor_speed_dps
        desired = max(-lim, min(lim, desired))
        if self._mode == "hold":
            self._speed = 0.0
        else:
            self._speed += (desired - self._speed) * (1 - math.exp(-TICK_MS / cfg.motor_tau_ms))
            self._angle += self._speed * dt
        return self._angle - old

    def _undo(self, delta: float) -> None:
        self._angle -= delta
        self._speed = 0.0

    def _finish(self) -> None:
        then = self._then
        if then == Stop.HOLD:
            self._mode, self._target = "hold", self._angle
        elif then == Stop.BRAKE:
            self._mode = "brake"
        else:
            self._mode = "coast"

    def _maybe_wait(self, wait_: bool) -> Any:
        return _wait_until(self.done) if wait_ else _Ready()

    # -- Pybricks API --
    def angle(self) -> int:
        return int(round(self._angle))

    def speed(self, window: int = 100) -> int:
        return int(round(self._speed))

    def reset_angle(self, angle: Optional[float] = None) -> None:
        new = 0.0 if angle is None else float(angle)
        self._target += new - self._angle
        self._angle = new

    def run(self, speed: float) -> None:
        self._mode, self._cmd = "run", float(speed)

    def dc(self, duty: float) -> None:
        self._mode, self._cmd = "run", duty / 100.0 * self._w.cfg.max_motor_speed_dps

    def stop(self) -> None:
        self._mode = "coast"

    def brake(self) -> None:
        self._mode = "brake"

    def hold(self) -> None:
        self._mode, self._target = "hold", self._angle

    def run_angle(self, speed: float, rotation_angle: float, then: Stop = Stop.HOLD,
                  wait: bool = True) -> Any:
        self._target = self._angle + rotation_angle * (1 if speed >= 0 else -1)
        self._mode, self._cmd, self._then = "angle", abs(float(speed)), then
        return self._maybe_wait(wait)

    def run_target(self, speed: float, target_angle: float, then: Stop = Stop.HOLD,
                   wait: bool = True) -> Any:
        self._target = float(target_angle)
        self._mode, self._cmd, self._then = "angle", abs(float(speed)), then
        return self._maybe_wait(wait)

    def run_time(self, speed: float, time: float, then: Stop = Stop.HOLD, wait: bool = True) -> Any:
        self._mode, self._cmd, self._then = "time", float(speed), then
        self._end_ms = self._w.t_ms + int(time)
        return self._maybe_wait(wait)

    def track_target(self, target_angle: float) -> None:
        self._mode, self._target = "track", float(target_angle)

    def done(self) -> bool:
        return self._mode not in ("angle", "time")

    def stalled(self) -> bool:
        return False

    def load(self) -> int:
        """Rough percent load: how hard the motor is working to reach its command."""
        if self._mode in ("run", "time"):
            return int(min(100, abs(self._cmd - self._speed) * 0.05 + abs(self._speed) * 0.02))
        return 0


# --------------------------------------------------------------------------
# pybricks.robotics.DriveBase (with optional gyro)
# --------------------------------------------------------------------------
class DriveBase:
    """Fake DriveBase. Units: mm, mm/s, degrees, deg/s (like Pybricks).

    Mirrors the real thing: a controller tracks a moving distance/angle target.
    With ``use_gyro(True)`` the angle comes from the IMU, so wheel slip does not
    spoil turns or straight lines.
    """

    KP_DIST = 8.0  # (mm/s) per mm of error
    KP_ANGLE = 10.0  # (deg/s) per degree of error

    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter: float,
                 axle_track: float) -> None:
        self._w = _world()
        self.left, self.right = left_motor, right_motor
        self.wd, self.track = float(wheel_diameter), float(axle_track)
        self._gyro = False
        self._settings = [500.0, 1000.0, 200.0, 1000.0]  # v, a, turn rate, turn accel
        self._mode = "idle"  # idle drive straight turn hold
        self._cmd_v = self._cmd_w = 0.0
        self._v = self._wr = 0.0  # current profile speeds
        self._t_dist = self._t_ang = 0.0  # moving targets
        self._goal = 0.0
        self._then = Stop.HOLD
        self._dist0 = self._ang0 = 0.0  # reset offsets
        self._gyro0 = self._w.measured_heading()
        self._w.hooks.append(self._control)

    # -- measurements --
    def _raw_dist(self) -> float:
        return (self.left._angle + self.right._angle) / 2 * math.pi * self.wd / 360.0

    def _raw_angle(self) -> float:
        diff = (self.left._angle - self.right._angle) * math.pi * self.wd / 360.0
        return math.degrees(diff / self.track)

    def _meas_angle(self) -> float:
        if self._gyro:
            return self._w.measured_heading() - self._gyro0
        return self._raw_angle() - self._ang0

    def _meas_dist(self) -> float:
        return self._raw_dist() - self._dist0

    # -- control loop (once per tick) --
    def _control(self, dt: float) -> None:
        if self._mode == "idle":
            return
        a, ta = self._settings[1], self._settings[3]
        if self._mode == "drive":
            self._v, self._wr = self._cmd_v, self._cmd_w
        elif self._mode == "straight":
            rem = self._goal - self._t_dist
            vmax = min(self._settings[0], math.sqrt(2 * a * abs(rem)))
            want = math.copysign(vmax, rem)
            self._v += max(-a * dt, min(a * dt, want - self._v))
            self._wr = 0.0
            if abs(rem) < 0.05 and abs(self._v) < 5:
                self._t_dist, self._v = self._goal, 0.0
                self._mode = "settle"
        elif self._mode == "turn":
            rem = self._goal - self._t_ang
            wmax = min(self._settings[2], math.sqrt(2 * ta * abs(rem)))
            want = math.copysign(wmax, rem)
            self._wr += max(-ta * dt, min(ta * dt, want - self._wr))
            self._v = 0.0
            if abs(rem) < 0.05 and abs(self._wr) < 2:
                self._t_ang, self._wr = self._goal, 0.0
                self._mode = "settle"
        elif self._mode in ("hold", "settle"):
            self._v = self._wr = 0.0
            if self._mode == "settle" and abs(self._t_dist - self._meas_dist()) < 1.0 \
                    and abs(self._t_ang - self._meas_angle()) < 0.3 \
                    and abs(self.left._speed) + abs(self.right._speed) < 30:
                self._mode = "hold"
        self._t_dist += self._v * dt
        self._t_ang += self._wr * dt
        v = self._v + self.KP_DIST * (self._t_dist - self._meas_dist())
        w = self._wr + self.KP_ANGLE * (self._t_ang - self._meas_angle())
        half = math.radians(w) * self.track / 2  # clockwise: left wheel faster
        k = 360.0 / (math.pi * self.wd)
        self.left.run((v + half) * k)
        self.right.run((v - half) * k)

    def _start(self, mode: str) -> None:
        if self._mode == "idle":  # begin from where we are
            self._t_dist, self._t_ang = self._meas_dist(), self._meas_angle()
            self._v = self._wr = 0.0
        self._mode = mode

    # -- Pybricks API --
    def use_gyro(self, use_gyro: bool) -> None:
        a = self._meas_angle()  # keep angle() continuous when switching
        self._gyro = bool(use_gyro)
        if self._gyro:
            self._gyro0 = self._w.measured_heading() - a
        else:
            self._ang0 = self._raw_angle() - a

    def settings(self, straight_speed: Optional[float] = None,
                 straight_acceleration: Optional[float] = None,
                 turn_rate: Optional[float] = None,
                 turn_acceleration: Optional[float] = None) -> tuple:
        new = [straight_speed, straight_acceleration, turn_rate, turn_acceleration]
        for i, v in enumerate(new):
            if v is not None:
                self._settings[i] = float(v)
        return tuple(int(v) for v in self._settings)

    def drive(self, speed: float, turn_rate: float) -> None:
        self._start("drive")
        self._cmd_v, self._cmd_w = float(speed), float(turn_rate)

    def stop(self) -> None:
        self._mode = "idle"
        self.left.stop()
        self.right.stop()

    def brake(self) -> None:
        self._mode = "idle"
        self.left.brake()
        self.right.brake()

    def straight(self, distance: float, then: Stop = Stop.HOLD, wait: bool = True) -> Any:
        self._start("straight")
        self._goal = self._t_dist + float(distance)
        self._then = then
        return _wait_until(self.done) if wait else _Ready()

    def turn(self, angle: float, then: Stop = Stop.HOLD, wait: bool = True) -> Any:
        self._start("turn")
        self._goal = self._t_ang + float(angle)
        self._then = then
        return _wait_until(self.done) if wait else _Ready()

    def curve(self, radius: float, angle: float, then: Stop = Stop.HOLD, wait: bool = True) -> Any:
        raise NotImplementedError("curve() is not simulated yet; use straight()/turn()/drive().")

    def distance(self) -> int:
        return int(round(self._meas_dist()))

    def angle(self) -> int:
        return int(round(self._meas_angle()))

    def state(self) -> tuple:
        return (self.distance(), int(round(self._v)), self.angle(), int(round(self._wr)))

    def reset(self, distance: float = 0, angle: float = 0) -> None:
        self._dist0 = self._raw_dist() - distance
        self._ang0 = self._raw_angle() - angle
        self._gyro0 = self._w.measured_heading() - angle
        self._t_dist, self._t_ang = float(distance), float(angle)

    def done(self) -> bool:
        return self._mode not in ("straight", "turn", "settle")

    def stalled(self) -> bool:
        return self._w.blocked


# --------------------------------------------------------------------------
# pybricks.pupdevices.ColorSensor (demo floor: colored stripes every 12 in)
# --------------------------------------------------------------------------
_STRIPES = (Color.WHITE, Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.BLACK)
_HSV = {  # (hue, sat, val, reflection)
    Color.WHITE: (0, 0, 100, 90), Color.RED: (0, 100, 90, 40), Color.GREEN: (120, 100, 70, 30),
    Color.BLUE: (220, 100, 80, 20), Color.YELLOW: (55, 100, 95, 70), Color.BLACK: (0, 0, 5, 3),
}


class _HSV3:
    def __init__(self, h: int, s: int, v: int) -> None:
        self.h, self.s, self.v = h, s, v


class ColorSensor:
    """Fake color sensor under the robot: stripe color depends on x position."""

    def __init__(self, port: Port) -> None:
        self._w = _world()
        if port != self._w.cfg.color_port:
            raise OSError(19, "No such device")

    def _stripe(self) -> Color:
        return _STRIPES[int(self._w.x // 12) % len(_STRIPES)]

    def color(self, surface: bool = True) -> Color:
        return self._stripe()

    def reflection(self) -> int:
        return _HSV[self._stripe()][3]

    def ambient(self) -> int:
        return 12

    def hsv(self, surface: bool = True) -> _HSV3:
        return _HSV3(*_HSV[self._stripe()][:3])


# --------------------------------------------------------------------------
# pybricks.hubs.PrimeHub
# --------------------------------------------------------------------------
class _IMU:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def heading(self, *args: Any, **kwargs: Any) -> float:
        return self._w.measured_heading()

    def reset_heading(self, angle: float) -> None:
        w = self._w
        drift = w.cfg.gyro_drift_deg_per_min * w.t_ms / 60000.0
        w.imu_offset = angle - (w.heading + drift)

    def tilt(self) -> tuple:
        return (0, 0)

    def angular_velocity(self, axis: Any = None) -> float:
        return self._w.yaw_rate

    def ready(self) -> bool:
        return True

    def stationary(self) -> bool:
        return all(abs(m._speed) < 1 for m in self._w.motors.values())


class _Buttons:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def pressed(self) -> set:
        return set(self._w.pressed)


class _Light:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def on(self, color: Any) -> None:
        self._w.light = color

    def off(self) -> None:
        self._w.light = None

    def blink(self, color: Any, durations: Any) -> None:
        self._w.light = color


class _Display:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def number(self, n: Any) -> None:
        self._w.display_text = n

    def text(self, text: Any, on: int = 500, off: int = 50) -> Any:
        self._w.display_text = text
        return wait(0)

    def char(self, c: Any) -> None:
        self._w.display_text = c

    def off(self) -> None:
        self._w.display_text = None

    def icon(self, icon: Any) -> None:
        self._w.display_text = icon

    def pixel(self, row: int, column: int, brightness: int = 100) -> None:
        pass


class _Speaker:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def beep(self, frequency: int = 500, duration: int = 100) -> Any:
        self._w.log_event("BEEP", (frequency, duration))
        return wait(duration)

    def volume(self, volume: Optional[int] = None) -> int:
        return 100 if volume is None else volume


class _Battery:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def voltage(self) -> int:
        return self._w.cfg.battery_mv

    def current(self) -> int:
        return 0


class _System:
    def __init__(self, w: SimWorld) -> None:
        self._w = w

    def set_stop_button(self, button: Any) -> None:
        self._w.stop_button = button

    def shutdown(self) -> None:
        raise SystemExit


class PrimeHub:
    """Fake SPIKE Prime hub."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        w = _world()
        self.imu = _IMU(w)
        self.buttons = _Buttons(w)
        self.light = _Light(w)
        self.display = _Display(w)
        self.speaker = _Speaker(w)
        self.battery = _Battery(w)
        self.system = _System(w)


# --------------------------------------------------------------------------
# Fake module objects
# --------------------------------------------------------------------------
def _module(name: str, **attrs: Any) -> types.ModuleType:
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    return m


_FAKE_MODULES: dict[str, types.ModuleType] = {
    "pybricks": _module("pybricks"),
    "pybricks.hubs": _module("pybricks.hubs", PrimeHub=PrimeHub),
    "pybricks.pupdevices": _module("pybricks.pupdevices", Motor=Motor, ColorSensor=ColorSensor),
    "pybricks.parameters": _module("pybricks.parameters", Port=Port, Direction=Direction,
                                   Stop=Stop, Button=Button, Color=Color, Axis=Axis),
    "pybricks.robotics": _module("pybricks.robotics", DriveBase=DriveBase),
    "pybricks.tools": _module("pybricks.tools", wait=wait, StopWatch=StopWatch,
                              multitask=multitask, run_task=run_task),
}


def reset_robot_modules(prefixes: tuple = ("alliedwork", "missions")) -> None:
    """Forget imported hub modules so they rebind to a new SimWorld."""
    for name in list(sys.modules):
        if name.split(".")[0] in prefixes:
            del sys.modules[name]
