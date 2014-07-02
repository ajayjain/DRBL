#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School

PARAMETERS:
private:
	[angular_vel_factor]
	[linear_vel_factor]
	[max_linear_velocity]

	[cmd_topic]
'''

import rospy, tf, math
import geometry_msgs.msg

TURN_FACTOR = rospy.get_param('~angular_vel_factor', 4)
LIN_FACTOR	= rospy.get_param('~linear_vel_factor', .8)
MAX_LIN = rospy.get_param('~max_linear_velocity', 1)

CMD_TOPIC = rospy.get_param('~cmd_topic', '/seek_cmd_vel')

TRANSFORM_PARENT = rospy.get_param('~frame_id', '/robot1_tf/base_footprint')
TRANSFORM_CHILD = rospy.get_param('~child_frame_id', '/robot2_tf/base_footprint')
# MAX_ANG = rospy.get_param('~max_angular_velocity', 1)

# Given an x,y,z (z = 0) translation, return a linear and angular Twist velocity to reach target
# Translation needs to be a list or tuple of format (x, y, ...) or [x, y, ...]
def seek(translation):
	angular = TURN_FACTOR * math.atan2(trans[1], trans[0])

	linear  = LIN_FACTOR  * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
	linear  = MAX_LIN if linear > MAX_LIN else linear

	cmd = geometry_msgs.msg.Twist()
	cmd.linear.x = linear
	cmd.angular.z = angular

	return cmd


def main():
	rospy.init_node("seek_node")
	
	listener = tf.TransformListener()

	vel_pub = rospy.Publisher(CMD_TOPIC, geometry_msgs.msg.Twist)
	# vel2 = rospy.Publisher('/robot2/husky/cmd_vel', geometry_msgs.msg.Twist)
	# cmd2 = geometry_msgs.msg.Twist()
	# cmd2.linear.x = 1
	# cmd2.angular.z = 0.2

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			listener.waitForTransform(TRANSFORM_PARENT, TRANSFORM_CHILD, rospy.Time(0), rospy.Duration(3.0))
			(trans,rot) = listener.lookupTransform(TRANSFORM_PARENT, TRANSFORM_CHILD, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "exception!"
			continue

		cmd_vel= seek(trans)

		print cmd_vel, '\n'
		vel_pub.publish(cmd_vel)
		# vel2.publish(cmd2)

		rate.sleep()


if __name__ == "__main__":
	main()