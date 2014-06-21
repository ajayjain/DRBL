#!/usr/bin/env python
'''
Author: Ajay Jain
Created: 21 June 2014
'''

import sys
import rospy
from std_msgs.msg import String

mode_pub = target_pub = None
mode = String("idle") # pursuit | evasion | patrol | idle

def main():
	global mode_pub, target_pub

	rospy.init_node("state_publisher")

	mode_pub = rospy.Publisher("status/mode", String)
	target_pub = rospy.Publisher("status/target", String)

	r = rospy.rate(1)
	while not rospy.is_shutdown():
		target_pub.publish
		r.sleep()

if __name__ == "__main__":
	main()