"""Runs hardware/sensor_stream.py unchanged inside the simulator (no robot needed).

The sim robot drives a slow wavy path so every reading moves.
"""
from __future__ import annotations

import contextlib
import math
import os
import threading
from typing import Callable

from work.pc.simulator import SimConfig, SimWorld

STREAM_FILE = os.path.join(os.path.dirname(__file__), "..", "..", "hardware", "sensor_stream.py")


class _Stop(BaseException):
    pass


class _LineWriter:
    def __init__(self, on_line: Callable[[str], None]) -> None:
        self.on_line, self.buf = on_line, ""

    def write(self, text: str) -> int:
        self.buf += text
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.on_line(line)
        return len(text)

    def flush(self) -> None:
        pass


class SimRunner:
    """Background thread running the stream program against a fake hub."""

    def __init__(self, on_line: Callable[[str], None]) -> None:
        self.on_line = on_line
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        world = SimWorld(SimConfig(realtime=True, start_x_in=10, start_y_in=22,
                                   start_heading_deg=90, gyro_drift_deg_per_min=0.5)).install()

        def demo_driver(dt: float) -> None:  # wiggles the drive + attachment motors
            if self._stop.is_set():
                raise _Stop()
            t = world.t_ms / 1000.0
            m = world.motors
            if not m:
                return
            left, right = m.get(world.cfg.left_port), m.get(world.cfg.right_port)
            if left and right:
                left.run(250 + 120 * math.sin(t / 2.0))
                right.run(250 - 120 * math.sin(t / 2.0))
            for port, motor in m.items():
                if motor not in (left, right):
                    motor.run(200 * math.sin(t / 3.0 + ord(port.value)))

        world.hooks.append(demo_driver)
        try:
            with open(STREAM_FILE) as f:
                code = compile(f.read(), STREAM_FILE, "exec")
            with contextlib.redirect_stdout(_LineWriter(self.on_line)):  # type: ignore[type-var]
                exec(code, {"__name__": "__main__"})
        except _Stop:
            pass
        finally:
            world.uninstall()
