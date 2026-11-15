# The hub menu

The menu runs **on the robot**: no computer needed once it's uploaded.

## Controls
| Button | In the menu | During a run |
|---|---|---|
| LEFT / RIGHT arrows | previous / next run number (wraps around) | : |
| CENTER | **start** the shown run | **emergency stop** (ends the program) |
| Bluetooth button | stops the program | : |

Light: **green** = ready, **orange** = running, **red** = the run crashed (menu returns after 1.5 s).
Beeps: high = started, higher = finished OK, low long = crash.

The light matrix shows the run's number (1–99): the **N in `runN.py`**.

## Competition-day flow
1. At home (or at the pit), hub on: `python run.py --no-start` (uploads, doesn't start).
2. At the table, no computer: press **CENTER** on the hub → the menu appears.
3. Pick the run with the arrows, press **CENTER** to go.
4. After the run (or after an emergency stop → press CENTER again) the menu is back.

`python run.py` (without `--no-start`) uploads and starts the menu immediately.
Uploading **replaces** whatever program was on the hub.

## How it's built
`run.py` generates `main.py`:
```python
from alliedwork import run_menu
import run1
import run2
run_menu([(1, "run1", run1.run), (2, "run2", run2.run)])
```
and uploads it together with `alliedwork.py` and every mission file. Only `runN.py`
files appear in the menu; `test.py` and `_template.py` do not.

While the menu is waiting, the hub's stop button is moved to the Bluetooth button (so CENTER
is free to start runs); while a run is going, CENTER is the stop button again.
(`hub.system.set_stop_button`: verify on your firmware: see [TROUBLESHOOTING](TROUBLESHOOTING.md).)

## Limits
- Run numbers above 99 can't be shown on the 5×5 display.
- A mission that finishes normally returns to the menu; a mission stopped with CENTER ends the
  whole program.
