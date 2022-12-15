#!/usr/bin/env python3
import rospy
import sys
from manipulate import GripperControl

if __name__ == "__main__":
    args = sys.argv[1:]
    rospy.init_node('sawyerdemo', anonymous=True)
    
    gripper = GripperControl()
    if 'open' in args:
        gripper.gripper_open()
    elif 'close' in args:
        gripper.gripper_close()

