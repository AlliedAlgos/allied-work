# Changelog

Format: [Keep a Changelog](https://keepachangelog.com). Versions are `MAJOR.MINOR.PATCH`
written with a two-digit patch (`0.2.00`). The number lives in `hardware/alliedwork.py`
(`VERSION`); check it with `python run.py --version`. The hub log prints it when the menu starts.

When you change something: add a line under **Unreleased**, and when you release, move
the lines under a new version heading and bump `VERSION`.

## Unreleased
- `python run.py work` starts the dashboard (`--sim` for a fake robot, `-n NAME` to pick a hub).
- **Documentation website** (MkDocs + Material theme): left navigation, search, dark/light mode, works offline. `python run.py docs` serves it at http://localhost:8001, `python run.py changelog` opens it on this page, `python run.py docs --build` writes a static copy to `site/`. Config: `mkdocs.yml`; `mkdocs-material` added to `requirements.txt`.

## 0.2.00: 2026-11-14
The "students can actually use it" release.

### Added
- **`hardware/alliedwork.py`**: one file with the settings, PID, pose tracking and every mission command (`set_pose`, `drive`, `turn`, `turn_to`, `drive_to`, `square_to_wall`, `attachment`, `attachment_to`, `wait`, `beep`, `log`, `mission`, `stop`, `pose`, `heading`, `color`, `reflection`).
- **Hub menu** (`run_menu`): light matrix shows the run number, LEFT/RIGHT choose, CENTER starts, CENTER during a run stops it, a crashing run returns to the menu.
- **`run.py`**: `python run.py` uploads all runs + menu; `run.py run1` runs one; `--sim` fake robot; `--no-start`; `-n NAME`; `list`; `--version`. Bundles `alliedwork.py` + missions + generated `main.py` (checked against pybricksdev's compiler).
- **Sensor dashboard** (`python -m work.pc.server`, `--sim`): live gyro compass, motors A/B/E/F (angle, speed, load), color sensor (name, reflection, ambient, HSV), battery, 10-second graphs, Freeze button, auto-reconnect. Works offline.
- **`hardware/sensor_stream.py`** hub program and **`work/pc/protocol.py`** parser (never crashes on bad lines).
- **Simulator additions**: fake `DriveBase` (with `use_gyro`), `ColorSensor`, `Axis`, hub `system`/`imu.tilt`/`angular_velocity`, motor load; `SimRunner` for the dashboard.
- Real `pybricks` package in `requirements.txt` (editor autocomplete, any OS); `pyrightconfig.json`.
- Example runs `run1.py`, `run2.py`, `test.py`, `_template.py`.
- Telemetry lines `T,ms,x,y,heading` (every 200 ms), events `E,...`, logs `L,...`.
- This documentation set.

### Changed
- Steering now uses Pybricks **`DriveBase` with the gyro on** (`use_gyro(True)`) instead of driving the two motors directly.
- Each command blocks and runs its own 10 ms PID loop (no background controller task).
- Folder layout: **`hardware/`** = runs on the hub, **`work/`** = runs on the computer.
- Missions are `runN.py` files that define `def run():`.
- `claude.md` renamed `CLAUDE.md` (Claude Code only loads the uppercase name); `bridge/` corrected to `work/pc/`.
- Tests moved to `work/tests/`; `pytest.ini` sets the import paths.

### Removed
- `hardware/robot/` package (geometry, config) folded into `alliedwork.py`; mm/inch helper functions dropped (`DriveBase` works in mm itself).
- Empty Node stubs (`server.js`, `public/index.html`, `package.json`, `package-lock.json`): the dashboard is plain HTML + Python.
- Broken `.zed/tasks.json` (replaced by `python run.py`).

### Known limitations
- **Not tested on a real robot yet.** Hub-side calls `imu.tilt()`, `angular_velocity()`, `hsv()`, `hub.system.set_stop_button()` and `DriveBase.reset()` follow the current Pybricks docs but are unverified on hardware.
- Robot size, wheel/axle measurements, mat size, launch areas and mission polygons in `work/field/missions.json` are placeholders.
- Simulator: `DriveBase.curve()` not simulated; `then=` ignored; footprint is a 6 × 8 in placeholder.
- The hub can run the sensor stream **or** a mission, not both at once.

## 0.1.00: 2026-11-14
Foundation.

### Added
- `robot/geometry.py` and `robot/config.py` (angle wrap, heading-to-point, waypoint math, ports, gains, limits).
- **Simulator** (`work/pc/simulator.py`): fake `pybricks` modules on a virtual 1 ms clock; differential-drive physics, wall collisions (per-edge, wheels stall), wheel slip, gyro drift/noise, `multitask`/`run_task`, real-time or fast-forward.
- Field model `work/field/missions.json`, `CLAUDE.md`, build instructions.
- Tests for geometry and the simulator.

## Roadmap (not built yet)
- **0.3**: formal telemetry (T/E/L) shown live; field-map view with the robot's position and path on the 1 in grid; mat background image.
- **0.4**: field editor (click-to-plot polygons and waypoints), field-file validation, **code generation** from waypoints.
- **0.5**: live PID gain tuning from the dashboard; run recording and playback.
- Real-robot calibration pass (wheel diameter, axle track, PID gains) once measurements exist.
