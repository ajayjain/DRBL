#!/usr/bin/env python

"""
	behave_tree.py - Version 1.0 2015-02-07

	validate_and_publish(twist vel)
		# end_point = current_pos + vel*time
		if will hit obstacle
			if angular = 0
				publish rotate in arbitrary direction
			else
				publish rotate in correct vel.angular direction
		else
			publish vel
	Behave while target remains:
		ParallelOne:
			Sequence: RespondToTarget
				Task: WanderWhileTargetLocationUnknown	(RUNNING while loc unknown
														 SUCCESS when location is known)
					while location unknown:
						validate_and_publish(Wander)
					return SUCCESS (when location known)
				Task: PursueWhileAdvisable	(RUNNING while advisable,
											 SUCCESS when unadvisable)
					while pursuit is advisable:
						validate_and_publish(Pursue)
					return SUCCESS (when pursuit is unadvisable)
				Task: AvoidTargetWhileAdvisable
					while avoiding target is advisable:
						if damage is imminent
							validate_and_publish(Serpentine)
						else
							validate_and_publish(Flee)
					return SUCCESS (no longer need to avoid target, restart Behave loop)
			Task: Shooter/Shoot	(RUNNING until a shot is fired, then SUCCESS restarts Behave loop)
"""

import rospy, math
import geometry_msgs.msg
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *

class MotionValidator():
	@classmethod
	def setup(cls):
		cls.MAX_LIN = rospy.get_param('~linear_vel_max',  0.8)
		cls.MAX_ANG = rospy.get_param('~angular_vel_max', math.pi/2)
		cls.vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, latch=True) # remap this

	@classmethod
	def will_hit_obstacle(cls, vel):
		return False

	@classmethod
	def validate_and_publish(cls, vel):
		if cls.will_hit_obstacle(vel):
			new_vel = geometry_msgs.msg.Twist()
			new_vel.angular = math.copysign(cls.MAX_ANG, vel.angular) # if angular is near 0, the resulting turn is ~arbitrary
		else
			vel_pub.publish(vel)

class BehaviorCoordinator():
	def __init__(self):
		BEHAVE = Sequence("behave")

		TARGET_COUNTER = TargetCounter(1) # destroy 1 target
		# PARALLEL_MOVE_SHOOT = Parallel

		# Run the tree
		while TARGET_COUNTER.is_target_remaining():
			BEHAVE.announce()
			status = BEHAVE.run()
			if status == TaskStatus.SUCCESS:
				print "Finished running tree. All targets destroyed."
				break

class TargetCounter():
	def __init__(self, num_to_destroy):
		self.num_destroyed = 0
		self.num_to_destroy = num_to_destroy
		print "Creating TargetCounter, num_to_destroy=", self.num_to_destroy

	def is_target_remaining(self):
		#TODO: Actually check if we have destroyed anything, and update the counter
		if self.num_destroyed < num_to_destroy:
			return True
		return False

class IsTargetLocationKnown(MonitorTask):
	def __init__(self, name, num_to_destroy, *args, **kwargs):
		super(IsTargetRemaining, self).__init__(name, *args, **kwargs)
	

	def run(self):

if __name__ == "__main__":
	rospy.init_node("behave_tree")
	
	MotionValidator.setup()
	BehaviorCoordinator()
	# rospy.Subscriber('/target_relative', RelativePosition, on_relative) # remap this in the launch file

	rospy.spin()