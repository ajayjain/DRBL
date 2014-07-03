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

import get_data

# TURN_FACTOR = rospy.get_param('~angular_vel_factor', 4)

LIN_FACTOR	= rospy.get_param('~linear_vel_factor', .2)
MAX_LIN = rospy.get_param('~max_linear_velocity', 1)
MAX_ANG = rospy.get_param('~max_angular_velocity', math.pi/2)

CMD_TOPIC = rospy.get_param('~cmd_topic', '/cmd_vel')

USE_TRANSFORMS = rospy.get_param('~use_transforms', False)

# Given an x,y,z (z = 0) translation, return a linear and angular Twist velocity to reach target
# Translation needs to be a list or tuple of format (x, y, ...) or [x, y, ...]
def seek(translation):
	translation = get_data.normalize(translation)
	translation = get_data.scalar_mult(translation, MAX_LIN)
	translation[2] = 0

	angular = math.atan2(translation[1], translation[0])
	# angular = MAX_ANG * math.atan2(translation[1], translation[0]) / math.pi

	print "Normalized: ", translation
	# linear  = math.sqrt(translation[0] ** 2 + translation[1] ** 2)
	# linear  = MAX_LIN if linear > MAX_LIN else linear
	linear = 0

	cmd = geometry_msgs.msg.Twist()
	cmd.linear.x = linear
	cmd.angular.z = angular

	return cmd

def main():
	rospy.init_node("seek_node")
	
	# listener = tf.TransformListener()

	vel_pub = rospy.Publisher(CMD_TOPIC, geometry_msgs.msg.Twist)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		# trans = get_data.transforms_translation(listener)
		trans = get_data.gazebo_translation()

		if trans is not None:
			cmd_vel = seek(trans)
			print "seeker translation: ", trans
			# print "seeker cmd_vel    : ", cmd_vel

			print cmd_vel.linear.x, '\t', cmd_vel.angular.z
			vel_pub.publish(cmd_vel)

		rate.sleep()


if __name__ == "__main__":
	main()