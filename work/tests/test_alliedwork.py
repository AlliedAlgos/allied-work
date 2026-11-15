"""alliedwork.py running on the simulated robot."""
import random

import pytest


def start(make_world, x=40, y=20, h=0, **cfg):
    w = make_world(start_x_in=x, start_y_in=y, start_heading_deg=h, **cfg)
    import alliedwork as aw
    aw.TELEMETRY_MS = 0
    aw.set_pose(x, y, h)
    return w, aw


def test_square_returns_to_start(make_world):
    w, aw = start(make_world, x=30, y=8)
    for _ in range(4):
        assert aw.drive(24, 8)
        assert aw.turn(90)
    x, y, h = w.pose
    assert (x, y) == pytest.approx((30, 8), abs=0.25)
    assert h == pytest.approx(0, abs=1.0)


def test_drive_backward_and_estimate_matches_truth(make_world):
    w, aw = start(make_world)
    aw.drive(-10, 6)
    assert w.y == pytest.approx(10, abs=0.25)
    ex, ey, eh = aw.pose()
    assert (ex, ey) == pytest.approx((w.x, w.y), abs=0.3)


def test_drive_to_random_points(make_world):
    rng = random.Random(7)
    w, aw = start(make_world, x=46, y=22)
    for _ in range(5):
        tx, ty = rng.uniform(10, 80), rng.uniform(10, 35)
        assert aw.drive_to(tx, ty, 10)
        assert (w.x, w.y) == pytest.approx((tx, ty), abs=0.4)


def test_turn_to_absolute_heading(make_world):
    w, aw = start(make_world)
    aw.turn_to(-90)
    assert w.pose[2] == pytest.approx(-90, abs=1.0)
    aw.turn_to(170)
    assert w.pose[2] == pytest.approx(170, abs=1.0)


def test_gyro_keeps_heading_despite_slip(make_world):
    w, aw = start(make_world, x=30, y=8, slip_pct=6, seed=5)
    aw.drive(30, 8)
    aw.turn(90)
    assert w.pose[2] == pytest.approx(90, abs=1.5)


def test_stalled_when_pushing_into_wall(make_world):
    w, aw = start(make_world, y=30)
    assert aw.drive(40, 8) is False  # wall is ~10 in away
    assert w.y == pytest.approx(40, abs=0.2)


def test_square_to_wall_fixes_pose_after_drift(make_world):
    w, aw = start(make_world, x=30, y=12, gyro_drift_deg_per_min=30)
    aw.square_to_wall(heading=0, y=3, speed=3, back=True)
    x, y, h = aw.pose()
    assert y == pytest.approx(3, abs=0.05) and h == pytest.approx(0, abs=0.5)
    assert w.y == pytest.approx(3, abs=0.3)  # rear 3 in behind the axle touches y=0


def test_attachment_motor(make_world):
    w, aw = start(make_world)
    aw.attachment("A", 90)
    aw.attachment_to("A", 0)
    assert aw._attach["A"].angle() == 0
    with pytest.raises(ValueError, match="no motor on port B"):
        aw.attachment("B", 10)


def test_clear_error_messages(make_world):
    w, aw = start(make_world)
    with pytest.raises(ValueError, match=r"drive\(\): speed must be > 0"):
        aw.drive(10, 0)
    with pytest.raises(ValueError, match="too fast"):
        aw.drive(10, 99)


def test_log_and_mission_lines(make_world, capsys):
    w, aw = start(make_world)
    aw.mission("M01")
    aw.log("hello")
    out = capsys.readouterr().out
    assert ",MISSION,M01" in out and ",hello" in out


def test_color_sensor_readings(make_world):
    w, aw = start(make_world, x=14)   # stripe 1 = RED
    assert aw.color() == "RED" and aw.reflection() == 40


def test_run_py_sim_runs_test_mission():
    import subprocess, sys
    r = subprocess.run([sys.executable, "run.py", "test", "--sim"], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and "Simulator finished" in r.stdout
    assert "x=20." in r.stdout and "y=10." in r.stdout
