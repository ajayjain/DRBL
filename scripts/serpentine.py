#!/usr/bin/env python
'''
Ajay Jain
July 2, 2014
ARSENL Lab, Naval Postgraduate School
'''

import math, random

import rospy, tf
import geometry_msgs.msg
from seek import seek

MAX_ANGULAR = math.pi/2
MAX_LINEAR  = 0.6

CMD_TOPIC = rospy.get_param('~cmd_topic', '/cmd_vel')
CMD_FREQ  = 10.0

SECONDS_PER_DIR = 2
SWITCH_ON_ITER = SECONDS_PER_DIR * CMD_FREQ

def get_params():
	global MAX_LINEAR, MAX_ANGULAR

	MAX_LINEAR = rospy.get_param('~linear_vel_max',  MAX_LINEAR)
	MAX_ANGULAR = rospy.get_param('~angular_vel_max', MAX_ANGULAR)


def main():
	rospy.init_node("serpentine")

	get_params()

	vel_pub = rospy.Publisher(CMD_TOPIC, geometry_msgs.msg.Twist)
	cmd_vel = geometry_msgs.msg.Twist()
	cmd_vel.linear.x = MAX_LINEAR
	cmd_vel.angular.z = MAX_ANGULAR

	rate = rospy.Rate(CMD_FREQ)
	count = SWITCH_ON_ITER
	trans = None
	delta = 2 * MAX_ANGULAR / SWITCH_ON_ITER

	while not rospy.is_shutdown():

		if count == SWITCH_ON_ITER:
			delta = delta * -1
			count = 0
		cmd_vel.angular.z += delta
		count += 1

		print cmd_vel.linear.x, '\t', cmd_vel.angular.z, '\t', delta
		vel_pub.publish(cmd_vel)
		
		rate.sleep()


if __name__ == "__main__":
	main()