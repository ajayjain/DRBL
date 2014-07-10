#!/usr/bin/env python
'''
Ajay Jain
July 10, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest('husky_pursuit')
import rospy, tf, math
from tf.transformations import euler_from_quaternion
from husky_pursuit.msg import RelativePosition

from utils import xy_to_rtheta

OWN_PREFIX = '/robot_0'
TARGET_PREFIX = '/robot_1'

own_frame = None
target_frame = None

def get_params():
	global OWN_PREFIX, own_frame, TARGET_PREFIX, target_frame

	OWN_PREFIX = rospy.get_param('~own_tf_prefix', OWN_PREFIX)
	TARGET_PREFIX = rospy.get_param('~target_tf_prefix', TARGET_PREFIX)

	own_frame = OWN_PREFIX + '/base_footprint'
	target_frame = TARGET_PREFIX + '/base_footprint'

def main():
	rospy.init_node("tf_to_relative")
	
	listener = tf.TransformListener()

	pub = rospy.Publisher('/target_relative', RelativePosition)
	message = RelativePosition()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		get_params()

		try:
			listener.waitForTransform(own_frame, target_frame, rospy.Time(0), rospy.Duration(3.0))
			(trans, rot) = listener.lookupTransform(own_frame, target_frame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("tf exception!")
			continue

		(r, theta) = xy_to_rtheta(trans[:2])
		message.range = r
		message.bearing = theta

		(message.roll, message.pitch, message.yaw) = euler_from_quaternion(rot)

		print "rot", tf.transformations.euler_from_quaternion(rot)
		print "trans", trans
		print "range", message.range
		print "bearing", math.degrees(message.bearing)
		pub.publish(message)

		rate.sleep()

if __name__ == "__main__":
	main()