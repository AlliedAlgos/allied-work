import math
import random

import pytest

@pytest.fixture(autouse=True)
def _load_alliedwork(make_world):
    """alliedwork sets up the (fake) hub when imported, so start a sim first."""
    global distance, heading_to_point, move_point, waypoint_step, wrap_angle, clamp, sign
    make_world()
    import alliedwork as a
    distance, heading_to_point, move_point = a.distance, a.heading_to_point, a.move_point
    waypoint_step, wrap_angle, clamp, sign = a.waypoint_step, a.wrap_angle, a.clamp, a.sign


def angle_diff(target, current):
    return wrap_angle(target - current)


@pytest.mark.parametrize("a,expected", [
    (0, 0), (90, 90), (180, 180), (-180, 180), (181, -179), (-181, 179),
    (360, 0), (-360, 0), (540, 180), (-540, 180), (725, 5), (-725, -5),
])
def test_wrap_angle(a, expected):
    assert wrap_angle(a) == pytest.approx(expected)


def test_wrap_angle_always_in_range():
    rng = random.Random(0)
    for _ in range(1000):
        w = wrap_angle(rng.uniform(-5000, 5000))
        assert -180 < w <= 180


def test_angle_diff_takes_short_way():
    assert angle_diff(10, 350) == pytest.approx(20)
    assert angle_diff(350, 10) == pytest.approx(-20)
    assert angle_diff(-170, 170) == pytest.approx(20)


@pytest.mark.parametrize("dx,dy,expected", [
    (0, 1, 0), (1, 0, 90), (0, -1, 180), (-1, 0, -90),   # axes
    (1, 1, 45), (1, -1, 135), (-1, -1, -135), (-1, 1, -45),  # four quadrants
])
def test_heading_to_point(dx, dy, expected):
    h = heading_to_point(10, 10, 10 + dx, 10 + dy)
    assert wrap_angle(h - expected) == pytest.approx(0, abs=1e-9)


def test_heading_to_same_point_is_zero():
    assert heading_to_point(5, 5, 5, 5) == 0


@pytest.mark.parametrize("h,dx,dy", [
    (0, 0, 1), (90, 1, 0), (180, 0, -1), (-90, -1, 0), (45, math.sqrt(.5), math.sqrt(.5)),
])
def test_move_point(h, dx, dy):
    x, y = move_point(3, 4, h, 1)
    assert (x - 3, y - 4) == pytest.approx((dx, dy), abs=1e-9)


def test_move_then_heading_round_trip_all_quadrants():
    rng = random.Random(1)
    for _ in range(200):
        h, d = rng.uniform(-180, 180), rng.uniform(0.5, 30)
        x, y = move_point(20, 20, h, d)
        assert wrap_angle(heading_to_point(20, 20, x, y) - h) == pytest.approx(0, abs=1e-7)
        assert distance(20, 20, x, y) == pytest.approx(d)


def test_waypoint_step():
    # facing +y at origin, point is to the right (+x) -> turn +90, drive 10
    turn, d = waypoint_step(0, 0, 0, 10, 0)
    assert (turn, d) == pytest.approx((90, 10))
    # facing +x (90), target behind-left -> shortest turn
    turn, d = waypoint_step(0, 0, 90, -3, -4)
    assert d == pytest.approx(5)
    assert turn == pytest.approx(wrap_angle(heading_to_point(0, 0, -3, -4) - 90))
    assert waypoint_step(1, 1, 30, 1, 1) == (0.0, 0.0)


def test_clamp_sign():
    assert clamp(5, 0, 3) == 3 and clamp(-1, 0, 3) == 0 and clamp(2, 0, 3) == 2
    assert (sign(-2), sign(0), sign(7)) == (-1, 0, 1)
