"""Copy this to run3.py, run4.py ...  The number in the file name is the number
shown on the hub. Then:  python run.py   (uploads all runs to the hub)"""
from alliedwork import *


def run():
    set_pose(x=0, y=0, heading=0)       # the launch area's start pose

    # M01_example: drive to the approach pose, then push the model
    mission("M01")
    drive_to(43, 16, 8)
    turn_to(0)
    drive(4, 4)                         # slow near models
    drive(-4, 4)
