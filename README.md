# FLL Robot Sensor Dashboard

Live page showing the gyro, all 4 motors (angle / speed / load), the color sensor and the battery.
Works offline (no internet needed). Needs Python 3.10+ (Windows, Mac or Linux).

## Setup (once, after forking)

**Mac / Linux**
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Windows (PowerShell)**
```
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

That installs the `pybricks` library too, so your editor can autocomplete and check the
hub code in `hardware/` right away (`pyrightconfig.json` tells the editor where our modules are).

## Run
(with the venv activated)
- With the robot (hub on, Pybricks firmware): `python run.py work`
  (add `-n "Allied"` to pick a hub by Bluetooth name)
- Without a robot (simulator): `python run.py work --sim`

(Long form: `python -m work.pc.server [--sim] [--hub NAME]`.)

The browser opens at http://localhost:8000. If the hub disconnects the page says so and
reconnects by itself. **Freeze** holds the numbers so you can read them.

## Write a run
1. Copy `hardware/missions/_template.py` to `hardware/missions/run3.py`. **The number in the file name is the number shown on the robot.**
2. Code inside `def run():` with the simple commands (`drive`, `turn`, `drive_to`, `attachment`, ...). The first line of the file is always `from alliedwork import *`.
3. Try it without the robot: `python run.py run3 --sim`

## Put runs on the robot (competition flow)
1. Hub on. Run `python run.py` : it uploads **all** runs plus the menu and starts it.
   (Use `python run.py --no-start` to upload only, then press the hub's center button to start.)
2. The light matrix shows a run number. **Left / right arrow buttons** change it, the **center button** starts it.
3. When the run ends the robot beeps and the menu comes back. Pressing **center during a run stops the robot**; press center again to get the menu back.

Other commands: `python run.py run1` (upload and run just one), `python run.py list`, `-n NAME` to pick a hub.
All robot settings (ports, wheel size, PID gains) are at the top of `hardware/alliedwork.py`.

## Documentation
Full docs (version 0.2.00) with a changelog: [`work/docs/README.md`](work/docs/README.md).
As a website: `python run.py docs` (opens http://localhost:8001); changelog: `python run.py changelog`; static copy for offline/sharing: `python run.py docs --build` (open `site/index.html`).

## Layout
- `hardware/` runs on the hub: `alliedwork.py` (the library + menu), `missions/` (your runs: run1.py, run2.py...), `sensor_stream.py` (dashboard readings).
- `run.py` bundles and uploads the runs to the hub.
- `work/` runs on the computer (`pc/` server, `dashboard/` page, `field/`, `docs/`, `tests/`).

Tests: `python -m pytest`
