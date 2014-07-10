#!/usr/bin/env python
'''
Ajay Jain
July 8, 2014
ARSENL Lab, Naval Postgraduate School
'''


#Basic imports
from ctypes import *
import sys
import rospy
import random
import phidgets # phidgets.py


def shoot():
    phidgets.interfaceKit.setOutputState(0, 1)
    sleep(1)
    phidgets.interfaceKit.setOutputState(0, 0)


def main():
    rospy.init_node("shooter")


    phidgets.phidgetsLauncher()

    rospy.spin()

    print("Closing...")

    try:
        interfaceKit.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Done.")
    exit(0)


if __name__ == "__main__":
    main()