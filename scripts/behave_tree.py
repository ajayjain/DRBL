#!/usr/bin/env python

"""
	behave_tree.py - Version 1.0 2015-02-07
"""

import rospy
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *

class Behave():
	def __init__(self):
		BEHAVE = Sequence("behave")

		IS_TARGET_REMAINING = IsTargetRemaining("is_target_remaining", 1) # destroy 1 target

		# Run the tree
		while True:
			status = BEHAVE.run()
			if status == TaskStatus.SUCCESS:
				print "Finished running tree. All targets destroyed."
				break

class IsTargetRemaining(Task):
	def __init__(self, name, num_to_destroy, *args, **kwargs):
		super(IsTargetRemaining, self).__init__(name, *args, **kwargs)

		self.name = name
		self.num_destroyed = 0
		self.num_to_destroy = num_to_destroy
		
		print "Creating task IsTargetRemaining, num_to_destroy=", self.num_to_destroy

	def run(self):
		#TODO: Actually check if we have destroyed anything, and update the counter
		if self.num_destroyed < num_to_destroy:
			return TaskStatus.SUCCESS

		return TaskStatus.FAILURE

