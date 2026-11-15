# Allied Work: Documentation

**Version 0.2.00** · FLL SPIKE Prime · Pybricks · Python 3.10+

Allied Work is a toolkit for FLL teams. Students write runs as short Python files
(`drive(24)`, `turn(90)`, ...); a hub menu lets them pick a run with the arrow buttons
and start it with the center button; a live dashboard shows every sensor; and a
simulator lets everything be tried without a robot.

> You are reading the docs website. Start it any time with `python run.py docs`. Everything is also plain markdown in `work/docs/`.

## Read in this order

| Doc | For | What's in it |
|---|---|---|
| [GETTING_STARTED](GETTING_STARTED.md) | everyone | install, first run, first mission |
| [WRITING_MISSIONS](WRITING_MISSIONS.md) | students | how to write a run, tips, examples |
| [API](API.md) | students | every command, with arguments and what it returns |
| [HUB_MENU](HUB_MENU.md) | students, coaches | the on-robot menu and the competition-day flow |
| [DASHBOARD](DASHBOARD.md) | everyone | the live sensor page |
| [TUNING](TUNING.md) | coaches, coders | measuring the robot, PID gains, limits |
| [SIMULATOR](SIMULATOR.md) | coders | the fake robot and how to test |
| [ARCHITECTURE](ARCHITECTURE.md) | coders | how the pieces fit, the file map |
| [FIELD_MODEL](FIELD_MODEL.md) | everyone | `missions.json`: mat, missions, coordinates |
| [PROTOCOL](PROTOCOL.md) | coders | the lines the hub prints |
| [TROUBLESHOOTING](TROUBLESHOOTING.md) | everyone | when something doesn't work |
| [DECISIONS](DECISIONS.md) | coders | why things are built the way they are |
| [CHANGELOG](CHANGELOG.md) | everyone | what changed in each version |

## The one-minute picture

```
hardware/alliedwork.py       the toolbox: settings, PID, commands, menu   (runs on the hub)
hardware/missions/runN.py    your runs, one file each                     (runs on the hub)
run.py                       bundles + uploads them to the hub            (runs on the computer)
work/                        dashboard, simulator, tests, docs            (runs on the computer)
```

## Status

Works and is tested in the simulator (72 tests): the command library, the hub menu,
the upload bundling, the sensor dashboard. **Not yet checked on a real robot**: see
[TUNING](TUNING.md) for the first-day checklist. Planned but not built: the
field-map view, click-to-plot editor and code generation (see the roadmap in the
[CHANGELOG](CHANGELOG.md)).
