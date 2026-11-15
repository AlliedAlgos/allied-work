# Troubleshooting

## Uploading / connecting
| Problem | Try |
|---|---|
| `python run.py` can't find the hub | hub on (press center), close the Pybricks Code website (it holds the Bluetooth link), try `-n "HubName"` |
| `ModuleNotFoundError: pybricksdev` | activate the venv, `pip install -r requirements.txt` |
| Dashboard says "Looking for the hub..." forever | same as above; the server's terminal shows pybricksdev's messages |
| Hub firmware errors on upload | the hub needs Pybricks firmware (https://code.pybricks.com) |
| Dashboard works but numbers lag | raise `PERIOD_MS` in `hardware/sensor_stream.py` (default 100) |
| Dashboard replaced my menu | `python run.py --no-start` puts it back |

## On the robot
| Problem | Try |
|---|---|
| Menu number doesn't change / center does nothing | check Pybricks version (latest); `hub.system.set_stop_button` must exist: send the error text |
| Robot drives the wrong distance | calibrate `WHEEL_DIAMETER_MM`: see [TUNING](TUNING.md) |
| A wheel runs backward | flip `LEFT_DIR` / `RIGHT_DIR` in `alliedwork.py` |
| `STALLED` in the log | the robot was blocked or pushed; check the path, or the wheels slipping |
| Turns very slow | raise `HEAD_KP` |
| `attachment(): no motor on port X` | check the cable and the `ATTACHMENTS` setting |
| Color shows `None` | sensor not on C/D; add its port to `COLOR_PORTS` |
| Run crashes (red light) | the log shows `E,...,ERROR,runN: message`; run it with `--sim` to see the Python error |
| Center during a run stopped everything | that's the emergency stop; press center again for the menu |

## On the computer
| Problem | Try |
|---|---|
| Tests fail with `No module named alliedwork` | run `python -m pytest` from the repo root (`pytest.ini` sets paths) |
| `import alliedwork` fails in a script | it needs a hub or a simulator: use `run.py --sim` or the `make_world` test fixture |
| `run.py --sim` robot ends somewhere odd | the sim starts at your first `set_pose`; check wall hits (`STALLED`) |
| Port 8000 in use | `python -m work.pc.server --port 8001` |

## Still stuck
Copy the **exact error text** and the command you ran. Check [CHANGELOG](CHANGELOG.md) → "Known limitations".
