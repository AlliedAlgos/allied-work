import math
import os

import pytest

from pc.simulator import SimWorld

FIELD = os.path.join(os.path.dirname(__file__), "..", "field", "missions.json")
CIRC = math.pi * 56 / 25.4  # inches per wheel turn


def drive_motors():
    from pybricks.parameters import Port
    from pybricks.pupdevices import Motor
    return Motor(Port.F), Motor(Port.B)


def test_straight_one_wheel_turn(make_world):
    w = make_world()
    left, right = drive_motors()
    left.run_angle(500, 360, wait=False)
    right.run_angle(500, 360)
    w.advance(100)
    x, y, h = w.pose
    assert y == pytest.approx(CIRC, abs=0.05)
    assert x == pytest.approx(0, abs=0.01) and h == pytest.approx(0, abs=0.1)
    assert left.angle() == 360


def test_spin_in_place_is_clockwise_and_correct(make_world):
    w = make_world(start_x_in=40, start_y_in=20)
    from pybricks.hubs import PrimeHub
    hub = PrimeHub()
    left, right = drive_motors()
    left.run_angle(500, 360, wait=False)      # left forward, right backward
    right.run_angle(500, -360)
    w.advance(100)
    assert w.pose[2] == pytest.approx(180, abs=1.0)  # 2*CIRC / track = pi rad
    assert hub.imu.heading() == pytest.approx(180, abs=1.0)
    assert w.pose[:2] == pytest.approx((40, 20), abs=0.1)


def test_quarter_turn_direction(make_world):
    w = make_world(start_x_in=40, start_y_in=20)
    left, right = drive_motors()
    left.run_angle(300, 180, wait=False)
    right.run_angle(300, -180)
    w.advance(100)
    assert w.pose[2] == pytest.approx(90, abs=0.7)  # +90 = clockwise = facing +x


def test_wall_stops_robot(make_world):
    w = make_world(start_x_in=10, start_y_in=10)
    from pybricks.tools import run_task, wait
    left, right = drive_motors()

    async def main():
        left.run(500)
        right.run(500)
        await wait(8000)

    run_task(main())
    assert w.blocked
    assert w.y == pytest.approx(45 - 5, abs=0.05)  # front edge 5 in ahead of axle
    assert abs(left.speed()) == 0


def test_backing_into_wall(make_world):
    w = make_world(start_x_in=10, start_y_in=20, start_heading_deg=90)
    from pybricks.tools import run_task, wait
    left, right = drive_motors()

    async def main():
        left.run(-500)
        right.run(-500)
        await wait(6000)

    run_task(main())
    assert w.x == pytest.approx(3, abs=0.05)  # rear edge 3 in behind axle


def test_gyro_drift_and_reset(make_world):
    w = make_world(gyro_drift_deg_per_min=6)
    from pybricks.hubs import PrimeHub
    from pybricks.tools import wait
    hub = PrimeHub()
    assert hub.imu.heading() == pytest.approx(0)
    wait(60_000)
    assert hub.imu.heading() == pytest.approx(6, abs=0.01)
    hub.imu.reset_heading(0)
    assert hub.imu.heading() == pytest.approx(0, abs=1e-9)


def test_imu_starts_at_zero_with_start_heading(make_world):
    make_world(start_heading_deg=90)
    from pybricks.hubs import PrimeHub
    assert PrimeHub().imu.heading() == pytest.approx(0)


def test_slip_loses_distance(make_world):
    w = make_world(slip_pct=5, start_x_in=10, start_y_in=10)
    left, right = drive_motors()
    left.run_angle(500, 1080, wait=False)
    right.run_angle(500, 1080)
    w.advance(100)
    assert 0.9 * 3 * CIRC < w.y - 10 < 0.99 * 3 * CIRC


def test_blocking_wait_and_stopwatch(make_world):
    w = make_world()
    from pybricks.tools import StopWatch, wait
    sw = StopWatch()
    wait(250)
    assert w.t_ms == 250 and sw.time() == 250


def test_multitask_all_and_race(make_world):
    w = make_world()
    from pybricks.tools import multitask, run_task, wait
    out = []

    async def a():
        await wait(100)
        out.append("a")

    async def b():
        await wait(300)
        out.append("b")

    async def forever():
        while True:
            await wait(10)

    async def main():
        await multitask(a(), b())
        out.append(w.t_ms)
        await multitask(forever(), wait(50), race=True)
        out.append(w.t_ms)

    run_task(main())
    assert out[:2] == ["a", "b"]
    assert out[2] == pytest.approx(300, abs=2)
    assert out[3] == pytest.approx(350, abs=10)


def test_exception_in_task_propagates(make_world):
    make_world()
    from pybricks.tools import multitask, run_task, wait

    async def boom():
        await wait(5)
        raise ValueError("kaboom")

    async def main():
        await multitask(boom(), wait(1000))

    with pytest.raises(ValueError, match="kaboom"):
        run_task(main())


def test_endless_loop_times_out(make_world):
    make_world(max_runtime_ms=500)
    from pybricks.tools import run_task, wait

    async def main():
        while True:
            await wait(10)

    with pytest.raises(TimeoutError):
        run_task(main())


def test_from_field_reads_mat_and_wheel():
    w = SimWorld.from_field(FIELD)
    assert (w.cfg.mat_width_in, w.cfg.mat_height_in) == (93, 45)
    assert w.cfg.wheel_diameter_mm == 56 and w.cfg.axle_track_mm == 112
    assert w.cfg.robot_width_in > 0  # footprint 0 in file -> placeholder used


def test_alliedwork_imports_in_sim(make_world):
    make_world()
    import alliedwork
    assert alliedwork.WHEEL_DIAMETER_MM == 56


def make_base(use_gyro=True):
    from pybricks.parameters import Port
    from pybricks.pupdevices import Motor
    from pybricks.robotics import DriveBase
    db = DriveBase(Motor(Port.F), Motor(Port.B), 56, 112)
    if use_gyro:
        db.use_gyro(True)
    return db


def test_drivebase_straight_and_turn(make_world):
    w = make_world(start_x_in=40, start_y_in=10)
    db = make_base()
    db.straight(254)  # 10 in
    assert w.y == pytest.approx(20, abs=0.1)
    assert db.distance() == pytest.approx(254, abs=2)
    db.turn(90)
    assert w.pose[2] == pytest.approx(90, abs=1.0)
    db.straight(-127)
    assert w.x == pytest.approx(35, abs=0.15)


def test_drivebase_drive_runs_until_stopped(make_world):
    w = make_world(start_x_in=40, start_y_in=10)
    from pybricks.tools import run_task, wait
    db = make_base()

    async def main():
        db.drive(100, 0)
        await wait(1000)
        db.stop()

    run_task(main())
    assert w.y - 10 == pytest.approx(100 / 25.4, abs=0.4)


def test_gyro_makes_turns_accurate_despite_slip(make_world):
    w = make_world(start_x_in=40, start_y_in=20, slip_pct=8, seed=3)
    db = make_base(use_gyro=True)
    db.turn(90)
    assert w.pose[2] == pytest.approx(90, abs=1.5)


def test_drivebase_stops_at_wall(make_world):
    w = make_world(start_x_in=40, start_y_in=30)
    db = make_base()
    db.straight(1000, wait=False)
    from pybricks.tools import wait
    wait(4000)
    assert w.blocked and db.stalled()
    assert w.y == pytest.approx(40, abs=0.1)
