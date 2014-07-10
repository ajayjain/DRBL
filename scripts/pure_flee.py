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
from pure_seek import seek, seek_rtheta

MAX_LIN = 0.8
MAX_ANG = math.pi/2

vel_pub = None

def get_params():
	global MAX_LIN, MAX_ANG, FRAME1_PREFIX, FRAME2_PREFIX, frame1, frame2
	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)

def flee_rtheta(rtheta, maxlin, maxang):
	inverted = [rtheta[0] * -1, rtheta[1]]
	return seek_rtheta(inverted, maxlin, maxang)

def flee(translation, maxlin, maxang):
	inverted = mult(translation, -1)
	return seek(inverted, maxlin, maxang)

def on_relative(rel_pos):
	get_params()

	rtheta = [rel_pos.range, rel_pos.bearing]
	cmd = flee_rtheta(rtheta, MAX_LIN, MAX_ANG)
	vel_pub.publish(cmd)

	rospy.loginfo('linear vel: %f', cmd.linear.x)
	rospy.loginfo('angular vel: %f', cmd.angular.z)

def main():
	global vel_pub

	rospy.init_node("flee")
	
	listener = tf.TransformListener()

	rospy.Subscriber('/target_relative', RelativePosition, on_relative) # remap this in the launch file
	vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)

	rospy.spin()

if __name__ == "__main__":
	main()