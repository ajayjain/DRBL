#!/usr/bin/env python

#Basic imports
from ctypes import *
import sys
import rospy
from std_msgs.msg import Bool
import random
import phidgets # phidgets.py


def shoot():
    phidgets.interfaceKit.setOutputState(0, 1)
    rospy.sleep(1)
    phidgets.interfaceKit.setOutputState(0, 0)

def on_message(data):
    if data.data:
        shoot()

def main():
    rospy.init_node("shooter_driver")

    phidgets.phidgetsLauncher()
    rospy.Subscriber("status/fire", Bool, on_message)
    rospy.spin()
    
    print("Closing...")
    phidgets.closePhidget()
    print("Done.")


if __name__ == "__main__":
    main()