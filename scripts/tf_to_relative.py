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

def main():
	global OWN_PREFIX, own_frame, TARGET_PREFIX, target_frame

	rospy.init_node("tf_to_relative")

	OWN_PREFIX = rospy.get_param('~own_tf_prefix', OWN_PREFIX)
	TARGET_PREFIX = rospy.get_param('~target_tf_prefix', TARGET_PREFIX)

	own_frame = OWN_PREFIX + '/base_footprint'
	target_frame = TARGET_PREFIX + '/base_footprint'

	relative_topic = rospy.get_param('~relative_topic', 'target_relative')

	silent = rospy.get_param('~silent', True) # silence node logs by default

	listener = tf.TransformListener()
	pub = rospy.Publisher(relative_topic, RelativePosition)
	message = RelativePosition()

	rate = rospy.Rate(10.0)
	# listener.waitForTransform(own_frame, target_frame, rospy.Time.now(), rospy.Duration(5.0))

	trans = rot = None

	while not rospy.is_shutdown():
		try:
			# now = rospy.Time.now()
			# listener.waitForTransform(own_frame, target_frame, now, rospy.Duration(5.0))
			# (trans, rot) = listener.lookupTransform(own_frame, target_frame, now)
			(trans, rot) = listener.lookupTransform(own_frame, target_frame, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			if not silent:
				rospy.loginfo("tf exception: %s", e)
			# Allow usage of old data if data was recieved at some point
			if trans == None or rot == None:
				continue

		print own_frame, target_frame, trans
		(x, y) = trans[:2]
		(r, theta) = xy_to_rtheta((x, y))
		# (r, theta) = xy_to_rtheta((x, y * -1))
		theta = theta % (2 * math.pi)
		message.range = r
		message.bearing = theta

		(message.roll, message.pitch, message.yaw) = map(lambda x: x % (2 * math.pi), euler_from_quaternion(rot))

		# if not silent:	
		rospy.loginfo("To %s:\nrot %s\ntrans %s\nrtheta %s\nrange %s\nbearing %s",
			relative_topic,
			tf.transformations.euler_from_quaternion(rot),
			trans,
			str((r, math.degrees(theta))),
			message.range,
			math.degrees(message.bearing))

		pub.publish(message)

		rate.sleep()

if __name__ == "__main__":
	main()