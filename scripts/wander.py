#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School
'''

import math, random

import rospy, tf
import geometry_msgs.msg
from pure_seek import seek

CMD_TOPIC = rospy.get_param('~cmd_topic', '/cmd_vel')
CMD_FREQ  = 10.0
SECONDS_PER_TRANSLATION = 2
LOOP_ON_ITER = SECONDS_PER_TRANSLATION * CMD_FREQ

MAX_LIN = 0.8
MAX_ANG = math.pi/2

def get_params():
	global MAX_LIN, MAX_ANG
	MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)

def random_translation():
	x = random.uniform(-10, 10)
	y = random.uniform(-10, 10)
	return (x, y, 0)

def main():
	rospy.init_node("wander_node")

	vel_pub = rospy.Publisher(CMD_TOPIC, geometry_msgs.msg.Twist)

	rate = rospy.Rate(CMD_FREQ)
	count = LOOP_ON_ITER
	trans = None
	while not rospy.is_shutdown():
		if count == LOOP_ON_ITER:
			trans = random_translation()
			print "Seeking trans", trans
			count = 0
		count += 1

		get_params()

		cmd = seek(trans, MAX_LIN, MAX_ANG)
		rospy.loginfo('linear vel: %f', cmd.linear.x)
		rospy.loginfo('angular vel: %f', cmd.angular.z)

		vel_pub.publish(cmd)

		rate.sleep()


if __name__ == "__main__":
	main()