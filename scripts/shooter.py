#!/usr/bin/env python

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
    rospy.init_node("leap_control")


    phidgets.phidgetsLauncher()





    rospy.spin()

    # # Keep this process running until Enter is pressed
    # print "Press Enter to quit..."
    # try:
    #   sys.stdin.readline()
    # except KeyboardInterrupt:
    #   pass
    # finally:
    #   # Remove the sample listener when done
    #   controller.remove_listener(listener)

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