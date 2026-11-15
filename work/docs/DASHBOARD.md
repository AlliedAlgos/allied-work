# The sensor dashboard

A web page on your computer that shows what the robot's sensors read, live. Works offline.

```
python run.py work                  # shortcut: real hub (turn it on first)
python run.py work --sim            # shortcut: fake robot

python -m work.pc.server            # the long form: real hub
python -m work.pc.server --sim      # fake robot, no hub needed
python -m work.pc.server --hub "Allied"     # pick a hub by Bluetooth name
python -m work.pc.server --port 8001 --no-browser
```
Opens http://localhost:8000. The server only listens on `127.0.0.1` (this computer).

## What you see
- **Header**: connection status (green = connected), battery voltage (red under 7.2 V, amber under 7.7 V), **Freeze** button (holds the numbers so you can read them), "NO DATA" and a dimmed page if nothing arrives for 1.5 s.
- **Gyro**: compass needle (0 = up, clockwise), heading, turn rate, pitch/roll.
- **Motors F, B, A, E** (left drive, right drive, left attachment, right attachment): angle, speed with a bar, load with a bar (red at ≥ 80 %). "Not connected" if a port is empty.
- **Color sensor**: swatch of the color, name, reflection, ambient, hue/saturation/value. "Not detected" if none on C/D.
- **Graphs**: motor speeds and heading for the last 10 seconds.

Use it to check wiring: spin a wheel by hand and watch its angle change; hold the color sensor over the mat.

## How the data gets there
```
hub: hardware/sensor_stream.py prints a line every 100 ms
 → Bluetooth → pybricksdev (server runs it as a subprocess, unbuffered)
 → work/pc/protocol.py parses the line (bad lines are skipped)
 → WebSocket /ws (JSON) → the page
```
If the hub disconnects the server retries every 3 s and the page shows the status.
Format details: [PROTOCOL](PROTOCOL.md).

## Important
The dashboard uploads `sensor_stream.py` to the hub, which **replaces the menu program**.
Run `python run.py --no-start` again afterwards to put the menu back. Streaming and running
missions at the same time is not supported yet (planned: see the roadmap).

## Files
`work/pc/server.py` (aiohttp server) · `work/pc/protocol.py` · `work/pc/sim_runner.py`
(runs the stream program in the simulator for `--sim`) · `work/dashboard/index.html`
(one file, no libraries) · `hardware/sensor_stream.py`.
