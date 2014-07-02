#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
import geometry_msgs.msg
from seek import seek

CMD_TOPIC = rospy.get_param('~cmd_topic', '/wander_cmd_vel')

def main():
	rospy.init_node("wander_node")

	vel_pub = rospy.Publisher(CMD_TOPIC, geometry_msgs.msg.Twist)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		cmd_vel = seek(trans)

		print cmd_vel
		print '\n'
		vel_pub.publish(cmd_vel)
		# vel2.publish(cmd2)

		rate.sleep()


if __name__ == "__main__":
	main()