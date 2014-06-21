#!/usr/bin/env python
'''
Author: Ajay Jain
Created: 21 June 2014
'''

# TODO: Monitor goal progress

from __future__ import print_function

import sys
import rospy
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Pose

goal_pub = None
goal = None

# Scaffold empty MoveBaseActionGoal goal
def generate_goal():
	global goal

	pose_stamped = PoseStamped()
	pose_stamped.pose = None # no goal at start

	move_base_goal = MoveBaseGoal()
	move_base_goal.target_pose = pose_stamped

	goal = MoveBaseActionGoal()
	goal.goal = move_base_goal

def on_pose(pose):
	print("Received pose", pose)
	goal.goal.target_pose.pose = pose  # TODO: modify pose? Perhaps some distance behind target
	goal_pub.publish(goal)
	print("Published", goal)

def main():
	global goal_pub

	rospy.init_node("goal_publisher")

	generate_goal()

	print("Creating move_base/goal publisher.")
	goal_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
	print("Creating amcl_pose/pose Subscriber.")
	rospy.Subscriber("amcl_pose/pose", on_pose, Pose) # TODO: enemy pose, not my _own_ pose
	print("Listening and publishing.")
	rospy.spin()

if __name__ == "__main__":
	main()