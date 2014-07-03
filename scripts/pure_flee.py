#!/usr/bin/env python
'''
Ajay Jain
July 3, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
import geometry_msgs.msg

from utils import *

MAX_LIN = 0.8
MAX_ANG = math.pi/2

def get_params():
	global MAX_LIN, MAX_ANG
	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)


def main():
	rospy.init_node("flee")
	
	listener = tf.TransformListener()

	vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			listener.waitForTransform("/robot_0/base_footprint", "/robot_1/base_footprint", rospy.Time(0), rospy.Duration(3.0))
			(trans,rot) = listener.lookupTransform('/robot_0/base_footprint', '/robot_1/base_footprint', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("exception!")
			continue

		get_params()
		trans = mult(trans, -1)
		cmd = seek(trans, MAX_LIN, MAX_ANG)

		rospy.loginfo('linear vel: %f', cmd.linear.x)
		rospy.loginfo('angular vel: %f', cmd.angular.z)
		vel.publish(cmd)

		rate.sleep()


if __name__ == "__main__":
	main()