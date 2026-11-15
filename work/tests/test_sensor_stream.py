"""Run the real hub program in the simulator and parse what it prints."""
import contextlib
import io
import os

import pytest

from work.pc.protocol import parse_line
from work.pc.sim_runner import STREAM_FILE
from work.pc.simulator import SimWorld


class Done(BaseException):
    pass


def run_stream(make_world, **cfg):
    w = make_world(**cfg)
    buf = io.StringIO()

    def stopper(dt):
        if w.t_ms > 1500:
            raise Done()
    w.hooks.append(stopper)
    with open(STREAM_FILE) as f, contextlib.redirect_stdout(buf):
        try:
            exec(compile(f.read(), STREAM_FILE, "exec"), {"__name__": "__main__"})
        except Done:
            pass
    return [parse_line(l) for l in buf.getvalue().splitlines()]


def test_stream_program_output_parses(make_world):
    rows = run_stream(make_world)
    assert len(rows) >= 14 and all(rows)
    r = rows[-1]
    assert set(r["motors"]) == {"A", "B", "E", "F"} and all(r["motors"].values())
    assert r["color"]["name"] == "WHITE" and r["battery_mv"] == 8000
    assert r["gyro"]["heading"] == pytest.approx(0, abs=0.1)


def test_stream_without_color_sensor(make_world):
    rows = run_stream(make_world, color_port=None)
    assert rows[-1]["color"] is None
