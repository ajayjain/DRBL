#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
import geometry_msgs.msg
# from geometry_msgs.msg import Twist, Vector3
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point

def main():
	rospy.init_node("seek")
	
	listener = tf.TransformListener()

	vel = rospy.Publisher('/robot1/husky/cmd_vel', geometry_msgs.msg.Twist)
	vel2 = rospy.Publisher('/robot2/husky/cmd_vel', geometry_msgs.msg.Twist)
	cmd2 = geometry_msgs.msg.Twist()
	cmd2.linear.x = 1
	# cmd2.angular.z = 0.5

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			listener.waitForTransform("/robot1_tf/base_footprint", "/robot2_tf/base_footprint", rospy.Time(0), rospy.Duration(3.0))
			(trans,rot) = listener.lookupTransform('/robot1_tf/base_footprint', '/robot2_tf/base_footprint', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "exception!"
			continue

		angular = 4 * math.atan2(trans[1], trans[0])
		linear = .8 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
		cmd = geometry_msgs.msg.Twist()
		cmd.linear.x = linear
		cmd.angular.z = angular
		print cmd
		print '\n'
		vel.publish(cmd)
		vel2.publish(cmd2)

		rate.sleep()


if __name__ == "__main__":
	main()