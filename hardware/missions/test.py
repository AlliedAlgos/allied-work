"""Test: drives a 24 in square.  python run.py test"""
from alliedwork import *


def run():
    set_pose(x=20, y=10, heading=0)     # room to drive up and right
    for _ in range(4):
        drive(24, 8)                    # 24 inches forward
        turn(90)                        # quarter turn right
    log("square done")
