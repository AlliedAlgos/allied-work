# Getting started

## 1. One-time setup

**Hub firmware.** The SPIKE Prime hub must run Pybricks firmware (latest).
Install it at https://code.pybricks.com ("Install Pybricks Firmware", Chrome). Once per hub.

**Computer** (Python 3.10+, Windows / Mac / Linux):

```
# Mac / Linux
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
```
# Windows (PowerShell)
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
This installs `pybricksdev` (the uploader), `pybricks` (editor autocomplete),
`aiohttp` (dashboard server), `jsonschema`, `pytest`. Activate the venv each time
you open a new terminal.

## 2. Check that it works without a robot
```
python -m pytest                    # all tests should pass
python run.py test --sim            # a 24 in square on the fake robot
python -m work.pc.server --sim      # dashboard with a fake robot
```

## 3. Fill in your robot (coaches, once)
Do the measuring tasks (see [TUNING](TUNING.md) and [FIELD_MODEL](FIELD_MODEL.md)):
wheel diameter, axle track, robot size, ports, mat size, mission coordinates.
Put wheel/axle/ports at the top of `hardware/alliedwork.py`.

## 4. Your first run on the robot
1. Hub on (press the center button). Robot on the mat, ~30 in of clear space ahead.
2. `python run.py test` : uploads and runs the 24 in square test.
3. Measure how far it really went. See [TUNING](TUNING.md) if it's not 24 in.

## 5. Write your own run
Copy `hardware/missions/_template.py` to `hardware/missions/run3.py`, then see
[WRITING_MISSIONS](WRITING_MISSIONS.md). Upload everything with `python run.py`.

## Read these docs as a website
`python run.py docs` opens this documentation as a site (search, navigation, dark mode) at http://localhost:8001.
