# Architecture

## Two halves
```
 COMPUTER (work/, run.py)                         HUB (hardware/)
 ─────────────────────────                        ───────────────────────────
 run.py ── bundles + uploads ───Bluetooth───►     main.py (generated) → run_menu
                                                  alliedwork.py  (settings, PID, commands)
 work/pc/server.py ◄── stdout lines ◄──────────   hardware/missions/runN.py
 work/dashboard/index.html ◄─WebSocket─ server    hardware/sensor_stream.py
 work/pc/simulator.py   fake hub for tests/--sim
```

## File map
```
CLAUDE.md                    context for Claude Code / coding helpers (keep accurate)
README.md  requirements.txt  pytest.ini  pyrightconfig.json
run.py                       upload / run / simulate
hardware/                    EVERYTHING that runs on the hub
  alliedwork.py              settings, math, PID, pose, motion, attachments, sensors, menu
  sensor_stream.py           streams sensor readings (for the dashboard)
  missions/runN.py           one run per file (def run())
  missions/test.py           24 in square test (not in the menu)
  missions/_template.py      copy me
work/                        EVERYTHING that runs on the computer
  pc/simulator.py            fake pybricks + physics
  pc/server.py               dashboard server (aiohttp)
  pc/protocol.py             parse hub lines
  pc/sim_runner.py           runs sensor_stream in the simulator
  dashboard/index.html       the page
  field/missions.json        map of the mat (see FIELD_MODEL)
  tests/                     pytest (72 tests)
  docs/                      this documentation
```

## How a command works (e.g. `drive(24, 8)`)
Every 10 ms inside the command:
1. **Pose**: read `DriveBase.distance()` and the gyro; update `x, y, heading` (midpoint heading for each step).
2. **Errors**: distance left along the path; cross-track offset from the straight line; heading error.
3. **PID**: distance PID → speed (limited by a ramp-up and a "brake in time" curve `√(2·a·d)`); heading PID → turn rate.
4. **Output**: `DriveBase.drive(speed, turn_rate)`. The DriveBase (gyro on) drives the wheels.
5. **Done / stuck**: within tolerance for 100 ms = done; no progress for 1.5 s = `STALLED`.

## Conventions (never break these)
Inches, degrees, in/s. Origin bottom-left of the mat; +x right, +y up. Heading 0 = +y, clockwise
positive, wrapped to (−180, 180]. Moving `d` at heading `h`: `dx = d·sin h`, `dy = d·cos h`.
Millimetres only where Pybricks asks (`DriveBase`, wheel/axle settings).

## Rules for contributors
- Hub code stays small (MicroPython memory): no big libraries, no dataclasses/type hints in `hardware/`.
- Only `pybricks.*` and built-in modules on the hub.
- Same `alliedwork.py` must run on the hub and in the simulator unchanged.
- Add tests for PC code and for new commands (`work/tests/`), run `python -m pytest`.
- Update `CLAUDE.md`, the docs, and the [CHANGELOG](CHANGELOG.md) in the same change; bump `VERSION` on release.
