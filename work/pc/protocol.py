"""Parse the hub's sensor lines (format: work/docs/PROTOCOL.md)."""
from __future__ import annotations

from typing import Any, Optional

MOTOR_PORTS = ("A", "B", "E", "F")
EXPECTED_FIELDS = 7 + 3 * len(MOTOR_PORTS) + 6  # = 25


def _num(text: str) -> Optional[float]:
    """'-' (device missing) -> None, otherwise a number."""
    return None if text == "-" else float(text)


def parse_line(line: str) -> Optional[dict[str, Any]]:
    """Turn one 'S,...' line into a dict, or None if it isn't a good sensor line.

    Never raises: other print() output and half-received lines are just skipped.
    """
    f = line.strip().split(",")
    if f[0] != "S" or len(f) != EXPECTED_FIELDS:
        return None
    try:
        motors = {}
        for i, port in enumerate(MOTOR_PORTS):
            a, s, l = f[7 + 3 * i: 10 + 3 * i]
            motors[port] = None if a == "-" else {
                "angle": _num(a), "speed": _num(s), "load": _num(l)}
        color = None if f[19] == "-" else {
            "name": f[19], "reflection": _num(f[20]), "ambient": _num(f[21]),
            "h": _num(f[22]), "s": _num(f[23]), "v": _num(f[24])}
        return {
            "ms": int(f[1]),
            "gyro": {"heading": _num(f[2]), "pitch": _num(f[3]),
                     "roll": _num(f[4]), "yaw_rate": _num(f[5])},
            "battery_mv": int(f[6]),
            "motors": motors,
            "color": color,
        }
    except ValueError:
        return None
