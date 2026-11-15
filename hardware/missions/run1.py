"""RUN 1 (example: replace with your real run). Drives 12 in out and back."""
from alliedwork import *


def run():
    set_pose(x=30, y=8, heading=0)      # the launch area's start pose
    mission("M01")
    drive(12, 8)
    drive(-12, 8)
