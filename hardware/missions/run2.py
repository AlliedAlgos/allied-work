"""RUN 2 (example: replace with your real run). Turns right, drives, comes back."""
from alliedwork import *


def run():
    set_pose(x=30, y=8, heading=0)
    mission("M02")
    turn(90)
    drive(10, 8)
    drive_to(30, 8, 8)
    turn_to(0)
