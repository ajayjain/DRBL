#!/usr/bin/env python
'''
Author: Ajay Jain
Created: 16 June 2014
'''

#Basic imports
from ctypes import *
import sys
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
import random
import phidgets # phidgets.py

from pprint import pprint

health_pub = None
health_msg = UInt8()
health_msg.data = 6
lives = []

def shoot():
	phidgets.interfaceKit.setOutputState(1, 1)
	rospy.sleep(0.1)
	phidgets.interfaceKit.setOutputState(1, 0)

def publish_health():
	if health_pub is not None:
		health_pub.publish(health_msg)

def on_message(data):
	if data.data:
		shoot()

def update_health(e):
	rospy.loginfo("phidget_driver InterfaceKit source %i: Input %i: %s" % (e.device.getSerialNum(), e.index, e.state))
	if e.index < len(lives):
		lives[e.index] = not e.state
	rospy.loginfo(lives)
	health_msg.data = lives.count(True)
	publish_health()

def main():
	global health_pub, lives

	rospy.init_node("phidget_driver")

	phidgets.phidgetsLauncher(input_changed_callback=update_health)
	lives = [True for _ in range(6)]
	# phidgets.interfaceKit.setOnInputChangeHandler(update_health)
	
	rospy.Subscriber("status/fire", Bool, on_message)
	health_pub = rospy.Publisher("status/health", UInt8, latch=True)

	rospy.spin()
	
	rospy.loginfo("Closing...")
	phidgets.closePhidget()
	rospy.loginfo("Done.")


if __name__ == "__main__":
	main()
