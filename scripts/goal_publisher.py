#!/usr/bin/env python
'''
Author: Ajay Jain
Created: 21 June 2014
'''

from __future__ import print_function

import sys
import rospy
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Pose

goal_pub = None
goal = generate_goal()

def generate_goal():
	pose_stamped = PoseStamped(None)

def on_pose(pose):
	target_pose = pose
	goal.goal.target_pose.pose = target_pose
	goal_pub.publish(goal)

def main():
	global goal_pub, goal

	rospy.init_node("goal_publisher")

	print("Creating move_base/goal publisher")
	goal_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
	rospy.Subscriber
	rospy.spin()

if __name__ == "__main__":
	main()