"""SPIKE Prime project converted to synchronous Pybricks Python.

Distances: cm. Speeds: motor degrees/second. Wheel diameter: 5.6 cm.
Assumes F is left drive, B is right drive, and a flat, upright hub.
Positive turn angles turn LEFT, preserving the original steering intent.
Only the E attachment test is enabled in main(), as in the source.
Movement must be calibrated on the physical robot before mission use.
"""
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch
from math import pi

hub = PrimeHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
right_attachment_motor = Motor(Port.E)
left_attachment_motor = Motor(Port.A)

WHEEL_DIAMETER = 5.6
cm_per_rotation = WHEEL_DIAMETER * pi
cm_per_degree = cm_per_rotation / 360
# New direct motor-speed controller: tune these on your robot.
STRAIGHT_KP = 5
TURN_KP = 4
LOOP_MS = 20


def to_degrees(distance):
   //return int(distance / cm_per_rotation * 360)


def stop_drive():
    left_motor.brake()
    right_motor.brake()


def drive_distance(cm, speed, direction):
    if cm < 0 or speed <= 0:
        raise ValueError('Use a nonnegative distance and a positive speed.')
    if cm == 0:
        stop_drive()
        return
    start_left = left_motor.angle()
    start_right = right_motor.angle()
    target_heading = hub.imu.heading()
    target_rotation = cm / cm_per_degree
    timer = StopWatch()
    timeout = max(5000, target_rotation / speed * 3000 + 2000)
    try:
        while True:
            traveled = direction * ((left_motor.angle() - start_left) +
                                    (right_motor.angle() - start_right)) / 2
            if traveled >= target_rotation:
                break
            if timer.time() > timeout:
                raise RuntimeError('Drive timed out: check wheels and motor directions.')
            # Positive heading is clockwise. Slow the left wheel to correct it.
            error = hub.imu.heading() - target_heading
            correction = max(-speed * 0.5, min(speed * 0.5, error * STRAIGHT_KP))
            left_motor.run(direction * speed - correction)
            right_motor.run(direction * speed + correction)
            wait(LOOP_MS)
    finally:
        stop_drive()


def go_forward(cm, speed):#js use speed for idrection
    drive_distance(cm, speed, 1)


def go_backward(cm, speed):
    drive_distance(cm, speed, -1)


def turn(degrees, speed):
    if speed <= 0:
        raise ValueError('Turn speed must be positive.')
    target_heading = hub.imu.heading() - degrees
    timer = StopWatch()
    # Adjustable timeout: stops a blocked or incorrectly configured robot.
    timeout = max(10000, abs(degrees) * 100)
    try:
        while True:
            error = target_heading - hub.imu.heading()
            if abs(error) < 2:
                break
            if timer.time() > timeout:
                raise RuntimeError('Turn timed out: check hub orientation and motors.')
            turn_speed = min(speed, max(min(60, speed), abs(error) * TURN_KP))
            direction = 1 if error > 0 else -1
            left_motor.run(direction * turn_speed)
            right_motor.run(-direction * turn_speed)
            wait(LOOP_MS)
    finally:
        stop_drive()


def right_attachment(degrees, speed):
    right_attachment_motor.run_angle(speed, degrees, then=Stop.HOLD)


def left_attachment(degrees, speed):
    left_attachment_motor.run_angle(speed, degrees, then=Stop.HOLD)


def main():
    # These calls block until complete; no async, await, or runloop is needed.
    # left_attachment(-460, 500)
    right_attachment(400, 500)
    # go_forward(68, 500)
    # go_backward(28, 500)
    # turn(26, 200)


try:
    main()
finally:
    stop_drive()
