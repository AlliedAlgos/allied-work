"""Streams every sensor reading to the dashboard (runs ON the hub).

Upload with the dashboard server (python -m work.pc.server) or by hand:
    pybricksdev run ble hardware/sensor_stream.py

One CSV line every PERIOD_MS, format in work/docs/PROTOCOL.md:
S,ms,heading,pitch,roll,yaw_rate,batt_mv, then angle,speed,load for motors
A,B,E,F, then color,reflection,ambient,hue,sat,val. Missing device = "-".
Motors are never driven here, so you can spin them by hand and watch.
"""
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Color, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.tools import StopWatch, wait

PERIOD_MS = 100  # 10 readings per second
MOTOR_PORTS = (Port.A, Port.B, Port.E, Port.F)  # team robot: A/E attachments, B/F drive
SENSOR_PORTS = (Port.C, Port.D)  # where the color sensor may be plugged in
COLORS = (
    (Color.RED, "RED"), (Color.ORANGE, "ORANGE"), (Color.YELLOW, "YELLOW"),
    (Color.GREEN, "GREEN"), (Color.CYAN, "CYAN"), (Color.BLUE, "BLUE"),
    (Color.VIOLET, "VIOLET"), (Color.MAGENTA, "MAGENTA"), (Color.WHITE, "WHITE"),
    (Color.GRAY, "GRAY"), (Color.BLACK, "BLACK"),
)

hub = PrimeHub()
hub.imu.reset_heading(0)


def find_color_sensor():
    for port in SENSOR_PORTS:
        try:
            return ColorSensor(port)
        except OSError:
            pass
    return None


motors = []
for p in MOTOR_PORTS:
    try:
        motors.append(Motor(p))
    except OSError:
        motors.append(None)
sensor = find_color_sensor()
clock = StopWatch()


def color_name(c):
    for known, name in COLORS:
        if c == known:
            return name
    return "NONE"


while True:
    try:
        pitch, roll = hub.imu.tilt()
        rate = hub.imu.angular_velocity(Axis.Z)
    except Exception:
        pitch, roll, rate = "-", "-", "-"
    parts = ["S", clock.time(), round(hub.imu.heading(), 1), pitch, roll,
             round(rate, 1) if rate != "-" else "-", hub.battery.voltage()]
    for m in motors:
        if m is None:
            parts += ["-", "-", "-"]
        else:
            parts += [m.angle(), m.speed(), m.load()]
    if sensor is None:
        parts += ["-"] * 6
    else:
        h = sensor.hsv()
        parts += [color_name(sensor.color()), sensor.reflection(), sensor.ambient(),
                  h.h, h.s, h.v]
    print(",".join(str(v) for v in parts))
    wait(PERIOD_MS)
