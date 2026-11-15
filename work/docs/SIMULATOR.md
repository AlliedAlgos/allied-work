# The simulator

A fake robot and fake `pybricks` library on the computer. The **same** `alliedwork.py` and
mission files run on it unchanged.

```
python run.py run1 --sim            # run a mission, prints the robot's true final position
python -m work.pc.server --sim      # dashboard with a fake robot
python -m pytest                    # 72 tests, all on the simulator
```
For `run.py --sim` the fake robot starts at the first `set_pose(...)` found in the mission file
(so the robot's belief and the truth start equal), or (12, 12, 0) if none.

## What is simulated
- **Clock**: virtual 1 ms ticks. Tests run as fast as possible; `realtime=True` sleeps to match the wall clock.
- **Drive**: two-wheel differential drive from motor angles, wheel diameter, axle track.
- **Walls**: the robot footprint can't push deeper into a mat edge; the wheels stall (so `STALLED` and `square_to_wall` work). A robot may start with the footprint partly off the mat.
- **Noise** (all off by default): `slip_pct` (wheel travel lost), `gyro_drift_deg_per_min`, `gyro_noise_deg`.
- **Devices**: `Motor`, `DriveBase` (with `use_gyro`), `PrimeHub` (imu heading/tilt/angular velocity, buttons, light, display, speaker, battery, system), `ColorSensor` (demo floor: stripes every 12 in), `StopWatch`, `wait`, `multitask`, `run_task`, `Port/Direction/Stop/Button/Color/Axis`.

## Not simulated / differences from the real hub
- `DriveBase.curve()` raises `NotImplementedError`; `then=` (stop mode) is ignored.
- Motor `load()` is a rough number; motors never stall except against a wall.
- The footprint is a placeholder (6 in wide, 5 in ahead of / 3 in behind the axle). `SimWorld.from_field(path)` reads mat size, wheel and axle from `work/field/missions.json` and uses the real footprint once it's filled in.
- Everything is ideal compared with a real mat, so **use the simulator to test logic, never to tune gains**.

## Using it in a test
```python
def test_mine(make_world):                 # fixture in work/tests/conftest.py
    w = make_world(start_x_in=30, start_y_in=8, slip_pct=5)   # any SimConfig field
    import alliedwork as aw                # import AFTER the world exists
    aw.drive(10)
    assert w.y == pytest.approx(18, abs=0.5)    # w.x, w.y, w.pose = the TRUTH
```
- `import alliedwork` builds the (fake) hub, so a world must exist first.
- Press buttons by setting `w.pressed = {Button.CENTER}`; add per-tick code with `w.hooks.append(fn)`.
- `w.events`, `w.display_text`, `w.light`, `w.blocked` expose what the fake hub did.

File: `work/pc/simulator.py` (config: `SimConfig`, engine: `SimWorld`).
