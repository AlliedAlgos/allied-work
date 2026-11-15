# Sensor stream protocol

The hub program `hardware/sensor_stream.py` prints one line every 100 ms:

```
S,ms,heading,pitch,roll,yaw_rate,batt_mv,
  A_angle,A_speed,A_load, B_..., E_..., F_...,
  color,reflection,ambient,hue,sat,val
```

- 25 comma-separated fields. `-` means that device isn't plugged in.
- Heading: degrees, 0 at program start, clockwise positive (Pybricks). Pitch/roll: degrees. Yaw rate: deg/s.
- Motor angle: degrees, speed: deg/s, load: percent.
- Color: name (RED, WHITE, ... or NONE), reflection/ambient: 0-100, hue 0-359, sat/val 0-100.
- Any other line (normal `print()`, partial line) is ignored by `work/pc/protocol.py`.

## Mission program lines (`alliedwork.py`)
These are printed by mission programs (not by `sensor_stream.py`), one per line:

| Line | When | Example |
|---|---|---|
| `T,<ms>,<x>,<y>,<heading>` | every 200 ms while a command runs (`TELEMETRY_MS`) | `T,4400,30.00,8.08,0.0` |
| `E,<ms>,<event>,<detail>` | events: `MISSION`, `RUN_START`, `RUN_END`, `STALLED`, `TIMEOUT`, `SQUARED`, `ERROR` | `E,4130,STALLED,drive` |
| `L,<ms>,<text>` | `log(text)`; the menu logs `alliedwork <version>` at start | `L,22380,square done` |

`work/pc/protocol.py` currently parses only the `S` sensor lines; showing T/E/L live is on the roadmap.
