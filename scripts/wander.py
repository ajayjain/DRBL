#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School
'''

import math, random

import rospy, tf
import geometry_msgs.msg
from seek import seek

CMD_TOPIC = rospy.get_param('~cmd_topic', '/wander_cmd_vel')
CMD_FREQ  = 10.0
SECONDS_PER_TRANSLATION = 2
LOOP_ON_ITER = SECONDS_PER_TRANSLATION * CMD_FREQ

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

		cmd_vel = seek(trans)
		print cmd_vel.linear.x, '\t', cmd_vel.angular.z
		# vel_pub.publish(cmd_vel)

		twist = geometry_msgs.msg.Twist()
		twist.linear.x = 0.6
		twist.angular.z = 0.2
		vel_pub.publish(twist)
		
		rate.sleep()


if __name__ == "__main__":
	main()