#!/usr/bin/env python
'''
Ajay Jain
July 3, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
import geometry_msgs.msg

from utils import *
from pure_seek import seek

FRAME1_PREFIX = '/robot_0'
FRAME2_PREFIX = '/robot_1'

frame1 = None
frame2 = None

MAX_LIN = 0.8
MAX_ANG = math.pi/2

def get_params():
	global MAX_LIN, MAX_ANG, FRAME1_PREFIX, FRAME2_PREFIX, frame1, frame2
	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)
	FRAME1_PREFIX = rospy.get_param('~own_tf_prefix', FRAME1_PREFIX)
	FRAME2_PREFIX = rospy.get_param('~target_tf_prefix', FRAME2_PREFIX)
	frame1 = FRAME1_PREFIX + '/base_footprint'
	frame2 = FRAME2_PREFIX + '/base_footprint'

def flee(translation, maxlin, maxang):
	inverted = mult(translation, -1)
	return seek(inverted, maxlin, maxang)

def main():
	rospy.init_node("flee")
	
	listener = tf.TransformListener()

	vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		get_params()

		try:
			listener.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(3.0))
			(trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("exception!")
			continue

		cmd = flee(trans, MAX_LIN, MAX_ANG)

		rospy.loginfo('linear vel: %f', cmd.linear.x)
		rospy.loginfo('angular vel: %f', cmd.angular.z)
		vel.publish(cmd)

		rate.sleep()

if __name__ == "__main__":
	main()