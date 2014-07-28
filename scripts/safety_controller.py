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
HALF_ROBOT_WIDTH = .7 / 2 # meters, actually, it's 670 / 2 mm
emergency = False
vel_pub = None

def on_scan(scan):
	global emergency

	theta = scan.angle_min
	angles = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
	emergency = False
	for polar in zip(scan.ranges, angles):
		if math.radians(-30) < polar[1] < math.radians(30):
			x, y = rtheta_to_xy(polar)
			if abs(y) <= HALF_ROBOT_WIDTH and x <= STOP_THRESHOLD:
				rospy.loginfo("Emergency: (%f m, %f deg), (%f m, %f m)", polar[0], math.degrees(polar[1]), x, y)
				emergency = True

# Only relay velocities when there isn't an emergency
def on_vel(vel):
	rospy.loginfo("Got vel: (%f, %f), emergency status: %s", vel.linear.x, vel.angular.z, str(emergency))
	vel_pub.publish(Twist() if emergency else vel)

def main():
	global vel_pub

	rospy.init_node("seek")

	behavior_cmd_topic = rospy.get_param('~behavior_cmd_topic', 'behavior_vel')
	robot_cmd_topic = rospy.get_param('~robot_cmd_topic', 'husky/cmd_vel')
	obstacle_topic = rospy.get_param('~obstacle_topic', 'status/obstacle')
	scan_topic = rospy.get_param('~scan_topic', 'scan')
	
	vel_pub = rospy.Publisher(robot_cmd_topic, Twist) # global
	obstacle_pub = rospy.Publisher(obstacle_topic, Bool)
	rospy.Subscriber(behavior_cmd_topic, Twist, on_vel)
	rospy.Subscriber(scan_topic, LaserScan, on_scan)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		obstacle_pub.publish(Bool(emergency))
		rate.sleep()


if __name__ == "__main__":
	main()