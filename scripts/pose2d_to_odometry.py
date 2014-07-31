#!/usr/bin/env python
'''
Ajay Jain
July 30, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest('husky_pursuit')
import rospy, math
# from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan

odom_pub = None

def on_pose(pose):
	odom = Odometry()
	odom.pose.pose.position.x = pose.x
	odom.pose.pose.position.y = pose.y
	odom.pose.pose.orientation.w = math.cos(pose.theta / 2)

	o = 10**-9 # zero

	odom.pose.covariance = [1, 0, 0, 0, 0, 0,
							0, 1, 0, 0, 0, 0,
							0, 0, o, 0, 0, 0, # z doesn't change
							0, 0, 0, o, 0, 0, # constant roll
							0, 0, 0, 0, o, 0, # constant pitch
							0, 0, 0, 0, 0, 0.3] # guess - .3 radians rotation cov

	odom.twist.covariance = [99999, 0, 0, 0, 0, 0,
							 0, 99999, 0, 0, 0, 0,
							 0, 0, 99999, 0, 0, 0, # large covariances - no data
							 0, 0, 0, 99999, 0, 0,
							 0, 0, 0, 0, 99999, 0,
							 0, 0, 0, 0, 0, 99999]

	odom_pub.publish(odom)

def main():
	rospy.init_node("pose2d_to_odometry")
	
	global odom_pub
	odom_topic = rospy.get_param('~odometry_publish_topic', 'vo')
	pose2D_topic = rospy.get_param('~pose2D_topic', 'pose2D')
	odom_pub = rospy.Publisher(odom_topic, Odometry)
	rospy.Subscriber(pose2D_topic, Pose2D, on_pose)

	rospy.spin()

if __name__ == "__main__":
	main()