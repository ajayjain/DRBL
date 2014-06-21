#!/usr/bin/env python
'''
Author: Ajay Jain
Created: 21 June 2014
'''

from __future__ import print_function

import sys
import rospy
from std_msgs.msg import String

mode_pub = target_pub = None
mode = String("idle") # pursuit | evasion | patrol | idle
target = String("B")

def main():
	global mode_pub, target_pub

	rospy.init_node("state_publisher")

	print("Creating status/mode and status/target publishers.")
	mode_pub = rospy.Publisher("status/mode", String)
	target_pub = rospy.Publisher("status/target", String)

	print("Publishing.")
	r = rospy.Rate(10) # 10 Hz
	while not rospy.is_shutdown():
		mode_pub.publish(mode)
		target_pub.publish(target)
		# print("Mode: {}, Target: {}".format(mode.data, target.data))
		r.sleep()

if __name__ == "__main__":
	main()