#!/usr/bin/env python
'''
Ajay Jain
July 10, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest('husky_pursuit')
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from husky_pursuit.msg import RelativePosition

# from utils import *

STOP_THRESHOLD = 1
emergency = False
stop_vel = Twist()

def get_params():
	pass
	# global MAX_LIN, MAX_ANG
	# MAX_LIN = rospy.get_param('~linear_vel_max',  MAX_LIN)
	# MAX_ANG = rospy.get_param('~angular_vel_max', MAX_ANG)


def on_scan(scan):
	global emergency

	num_ranges = len(scan.ranges)

	angle = scan.angle_min
	start_index = 0
	while angle < math.radians(-10) and start_index < num_ranges:
		angle += scan.angle_increment
		start_index += 1

	end_index = start_index
	while angle < math.radians(10) and end_index < num_ranges:
		angle += scan.angle_increment
		end_index += 1

	sub_ranges = scan.ranges[start_index:end_index]
	sub_intensities = scan.intensities[start_index:end_index]
	emergency = False
	for dist, intensity in zip(sub_ranges, sub_intensities):
		print dist, intensity
		if intensity == 1 and dist <= STOP_THRESHOLD:
			emergency = True
			break

	print emergency


# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~linear_vel_max'), rospy.get_param('~linear_vel_max', 0.8))
# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('linear_vel_max'), rospy.get_param('linear_vel_max', 0.8))
# rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('/linear_vel_max'), rospy.get_param('/linear_vel_max', 0.8))

def main():
	global vel_pub

	rospy.init_node("seek")
	
	vel_pub = rospy.Publisher('/robot_1/cmd_vel', Twist) # remap this
	rospy.Subscriber('/robot_1/base_scan', LaserScan, on_scan) # remap this in the launch file

	rate = rospy.Rate(500.0)
	while not rospy.is_shutdown():
		if emergency:
			vel_pub.publish(stop_vel)
		rate.sleep()


if __name__ == "__main__":
	main()