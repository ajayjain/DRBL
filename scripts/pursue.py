#!/usr/bin/env python
'''
Ajay Jain
July 7, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
import geometry_msgs.msg
import nav_msgs.msg

from utils import *
from pure_seek import seek

MAX_LIN = 0.8
MAX_ANG = math.pi/2

OWN_PREFIX = '/robot_0'
TARGET_PREFIX = '/robot_1'

own_frame = None
target_frame = None

CONSTANT_TIME_EXTRAPOLATION = True
EXTRAPOLATION_TIME = 3 # seconds

target_vel = geometry_msgs.msg.Twist()
target_pose = geometry_msgs.msg.Pose()
own_pose = geometry_msgs.msg.Pose()

def get_params():
	global MAX_LIN, MAX_ANG
	global OWN_PREFIX, own_frame, TARGET_PREFIX, target_frame
	global CONSTANT_TIME_EXTRAPOLATION, EXTRAPOLATION_TIME

	OWN_PREFIX = rospy.get_param('~own_tf_prefix', OWN_PREFIX)
	TARGET_PREFIX = rospy.get_param('~target_tf_prefix', TARGET_PREFIX)

	own_frame = OWN_PREFIX + '/base_footprint'
	target_frame = TARGET_PREFIX + '/base_footprint'

	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)

	CONSTANT_TIME_EXTRAPOLATION = rospy.get_param('~extrapolate_with_constant_time', CONSTANT_TIME_EXTRAPOLATION)
	EXTRAPOLATION_TIME = rospy.get_param('~extrapolation_time', EXTRAPOLATION_TIME)

def pursue(own_pose, target_pose, target_vel, extrapolation_time, maxlin, maxang):
	future_target_pose = extrapolate(target_pose, target_vel, extrapolation_time)
	translation = pose_translation(own_pose, future_target_pose)
	print "own_pose", own_pose
	print "target_pose", target_pose
	print "target_vel", target_vel
	print "future_target_pose", future_target_pose
	print "translation", translation
	return seek(translation, maxlin, maxang)

def on_own_odom(odom):
	global own_pose
	own_pose = odom.pose.pose

def on_target_odom(odom):
	global target_vel, target_pose
	target_vel = odom.twist.twist
	target_pose = odom.pose.pose

def main():
	rospy.init_node("pursue")
	
	listener = tf.TransformListener()

	vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)
	rospy.Subscriber('/target_odom', nav_msgs.msg.Odometry, on_target_odom)
	rospy.Subscriber('/own_odom', nav_msgs.msg.Odometry, on_own_odom)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		get_params()

		try:
			listener.waitForTransform('/world', own_frame, rospy.Time(0), rospy.Duration(3.0))
			own_pose = listener.lookupTransform('/world', own_frame, rospy.Time(0))

			listener.waitForTransform('/world', target_frame, rospy.Time(0), rospy.Duration(3.0))
			target_pose = listener.lookupTransform('/world', target_frame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("tf exception!")
			continue

		cmd = pursue(own_pose, target_pose, target_vel, EXTRAPOLATION_TIME, MAX_LIN, MAX_ANG)
		print "cmd_vel", cmd
		print '\n'
		# rospy.loginfo('linear vel: %f', cmd.linear.x)
		# rospy.loginfo('angular vel: %f', cmd.angular.z)
		vel.publish(cmd)

		rate.sleep()

if __name__ == "__main__":
	main()