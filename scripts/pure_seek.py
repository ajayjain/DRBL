#!/usr/bin/env python
'''
Ajay Jain
July 3, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest('husky_pursuit')
import rospy, tf, math
import geometry_msgs.msg
from husky_pursuit.msg import RelativePosition

from utils import *

MAX_LIN = 0.8
MAX_ANG = math.pi/2

vel_pub = None

def get_params():
	global MAX_LIN, MAX_ANG
	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)

def seek_rtheta(rtheta, maxlin, maxang):
	cmd = geometry_msgs.msg.Twist()

	cmd.linear.x = truncate(rtheta[0], maxlin)
	cmd.angular.z = truncate(rtheta[1], maxang)

	return cmd

def seek(translation, maxlin, maxang):
	trans = mult(normalize(translation), maxlin)

	angular = math.atan2(trans[1], trans[0]) # tan inverse(y, x) -> radians from positive x axis, [-pi, pi]
	angular = truncate(angular, maxang)

	linear  = math.hypot(trans[0], trans[1]) # sqrt(x^2 + y^2)
	# linear  = math.sqrt(trans[0] ** 2 + trans[1] ** 2) # sqrt(x^2 + y^2)
	linear  = truncate(linear, maxlin)

	cmd = geometry_msgs.msg.Twist()
	cmd.linear.x = linear
	cmd.angular.z = angular

	return cmd

def on_relative(rel_pos):
	get_params()
	rtheta = [rel_pos.range, rel_pos.bearing]
	cmd = seek(rtheta, MAX_LIN, MAX_ANG)
	vel_pub.publish(cmd)

	rospy.loginfo('linear vel: %f', cmd.linear.x)
	rospy.loginfo('angular vel: %f', cmd.angular.z)

# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~linear_vel_max'), rospy.get_param('~linear_vel_max', 0.8))
# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('linear_vel_max'), rospy.get_param('linear_vel_max', 0.8))
# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('/linear_vel_max'), rospy.get_param('/linear_vel_max', 0.8))

def main():
	rospy.init_node("seek")
	
	listener = tf.TransformListener()

	global vel_pub
	vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist) # remap this
	rospy.Subscriber('/target_relative', RelativePosition, on_relative) # remap this in the launch file

	rospy.spin()


if __name__ == "__main__":
	main()