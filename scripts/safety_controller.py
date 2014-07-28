#!/usr/bin/env python
'''
Ajay Jain
July 10, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest('husky_pursuit')
import rospy, math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from husky_pursuit.msg import RelativePosition

from utils import rtheta_to_xy

STOP_THRESHOLD = 1
emergency = False
stop_vel = Twist()

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

	# sub_ranges = scan.ranges[start_index:end_index]
	# sub_intensities = scan.intensities[start_index:end_index]
	# emergency = False
	# for dist, intensity in zip(sub_ranges, sub_intensities):
	# 	print dist, intensity
	# 	if intensity == 1 and dist <= STOP_THRESHOLD:
	# 		emergency = True
	# 		break

	sub_ranges = scan.ranges[start_index:end_index]
	emergency = False
	for dist in sub_ranges:
		if dist >= scan.range_min and dist <= STOP_THRESHOLD:
			rospy.loginfo("Emergency: distance: %f", dist)
			emergency = True
			break

	# rospy.loginfo("emergency status: %s", str(emergency))

def main():
	global vel_pub

	rospy.init_node("seek")

	# cmd_topic = rospy.get_param('~cmd_topic', 'husky/cmd_vel')
	obstacle_topic = rospy.get_param('~obstacle_topic', 'status/obstacle')
	scan_topic = rospy.get_param('~scan_topic', 'scan')
	
	# vel_pub = rospy.Publisher(cmd_topic, Twist) # remap this
	obstacle_pub = rospy.Publisher(obstacle_topic, Bool) # remap this
	rospy.Subscriber(scan_topic, LaserScan, on_scan) # remap this in the launch file

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		# if emergency:
			# rospy.loginfo("EMERGENCY: PUBLISHING TO %s, SUBSCRIBED TO %s", cmd_topic, scan_topic)
			# vel_pub.publish(stop_vel)
		obstacle_topic.publish(Bool(emergency))
		rate.sleep()


if __name__ == "__main__":
	main()