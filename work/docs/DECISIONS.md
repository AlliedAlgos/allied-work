# Decisions

One short entry each.

## Folder layout
- `hardware/` = everything uploaded to the hub.
- `work/` = everything on the PC (`pc/`, `dashboard/`, `field/`, `tests/`, `docs/`). `work/` already had an unrelated Node app (`server.js`, `public/`); left untouched.
- `CLAUDE.md` renamed from `claude.md` (Claude Code only loads the uppercase name). `bridge/` in it corrected to `work/pc/`.

## Drive control: Pybricks `DriveBase` with gyro (user request)
- The controller drives the robot through `DriveBase` with `use_gyro(True)`, not by running the two motors directly. The gyro keeps turns and straight lines accurate despite wheel slip.
- Our own PID still decides target speed / turn rate (`DriveBase.drive(speed, turn_rate)`); the DriveBase handles wheel-level control.
- The simulator has a fake `DriveBase` (straight, turn, drive, stop, state, reset, settings, use_gyro). `curve()` is not simulated yet. `then=` is ignored (always holds).

## Units at the Pybricks layer
- Our code is inches / degrees. `DriveBase` wants mm, mm/s: convert only at the controller edge (`geometry.in_to_mm`).

## Simulator
- Virtual 1 ms clock; same code runs in sim and on hub. Wall hits reject the step and stall the wheels (like a real wall). Starting with the footprint partly off the mat is allowed (per-edge check).
- Motor ports from the team's `main.py`: left F (CCW), right B (CW), attachments A and E.
- Sim footprint is a placeholder (6 x 8 in) until `robot.footprint_in` in `missions.json` is measured.

## Sensor dashboard (built on request, before Phase 2)
- `hardware/sensor_stream.py` prints one CSV line / 100 ms (format: `work/docs/PROTOCOL.md`); color sensor auto-detected on port C or D (no sensor in the original `main.py`, so port was unknown).
- `work/pc/server.py` runs `pybricksdev run ble` on it (reads stdout, auto-retries) and pushes JSON to the page over WebSocket; `--sim` runs the same program in the simulator.
- Deleted the empty Node stubs (`server.js`, `public/index.html`, `package*.json`): dashboard is plain HTML + aiohttp, no npm.

## One big library file: `hardware/alliedwork.py` (user request)
- Replaces the planned `robot/` package (geometry, config, pose, controller, api). Missions only `from alliedwork import *`. Settings are the first section of the file.
- Commands are blocking and each runs its own 10 ms PID loop (no background controller task): simpler for kids, no multitask needed. Steering goes through `DriveBase.drive(speed, turn_rate)` with `use_gyro(True)`.
- `square_to_wall` takes the wall's heading / coordinate as arguments instead of reading the field file (the hub can't read JSON); codegen will fill them in.
- `run.py` copies `alliedwork.py` + the mission (as `main.py`) into a temp folder and uploads that, because pybricksdev bundles imports found next to the script. 
- Removed `.zed/tasks.json` (broken); `python run.py <mission>` replaces it. Telemetry (`T,ms,x,y,h`) prints every 200 ms from the control loop; formal protocol is Phase 3.

## Hub menu (user request)
- Missions are `runN.py` files each defining `def run()` (a menu needs a function to call). `run.py` generates `main.py` that imports every `runN` and calls `run_menu([...])`; the bundle (`alliedwork.py`, all missions, `main.py`) is built in a temp folder and uploaded (verified that pybricksdev's `compile_multi_file` picks up every import).
- Menu: light matrix shows N, LEFT/RIGHT cycle (wraps), CENTER starts. While in the menu the stop button is moved to the Bluetooth button so CENTER is usable; during a run CENTER is the stop button again (emergency stop ends the program; CENTER restarts it). A crash in one run is caught, shown (red light + beep) and the menu returns.
- `python run.py --no-start` uploads without running so the program can be started from the hub with no PC.
- Numbers show 1-99 (`hub.display.number`).

## Documentation website: MkDocs + Material (user request)
- Docs stay plain markdown in `work/docs/` (readable in any editor and on GitHub); `mkdocs.yml` (repo root, `docs_dir: work/docs`) turns them into a site. `use_directory_urls: false` so the built `site/` also works opened straight from disk. The built-in `search` plugin is offline.
- Chosen over a hand-written generator for navigation/search/dark mode with no code to maintain. Note: Material for MkDocs announces end of life on 2027-05-05 (fixes continue until then); its maintained successor is Zensical, which reads the same `mkdocs.yml`: plan a switch then. `run.py docs` sets `NO_MKDOCS_2_WARNING=1` to keep the output quiet.
- Docs server uses port 8001 so it can run next to the dashboard (8000).
