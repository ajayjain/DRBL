#!/usr/bin/env python

"""
	behave_tree.py - Version 1.0 2015-02-07

	Complete tree:
	Sequence: Behave
		Task: IsTargetRemaining
		Parallel
			Task: Move
				Parallel
					Loop: Avoid Obstacles
						Sequence
							Task: IsCollisionImminent
							Task: Rotate
					Sequence: RespondToTarget
						Selector: FindTarget
							Task: IsTargetLocationKnown
							Task: Wander			(for time)
						Sequence: ChaseTarget
							Task: IsChaseAdvisable
							Task: Pursue			(for time)
						Selector: AvoidTarget
							Sequence: AvoidBeingShot
								Task: IsDamageImminent
								Task: Serpentine	(for time)
							Task: Flee				(for time)
			Task: Shooter/Shoot	(RUNNING until a shot is fired, then SUCCESS)
		Inverter: IsTargetDestroyed		(SUCCESS when all targets are destroyed, so behavior is done.
										 FAILURE when a target remains and Behave should restart from left)
			Task: IsTargetRemaining


	Simple start:
	Behave while target remains:
		Parallel:
			Loop: Avoid Obstacles
				Sequence
					Task: IsCollisionImminent
					Task: Rotate
			Sequence: RespondToTarget
				Task: WanderWhileTargetLocationUnknown	(RUNNING while loc unknown
														 SUCCESS when location is known)
				Task: PursueWhileAdvisable	(RUNNING while advisable,
											 SUCCESS when unadvisable)
				Task: SerpentineWhileDamageImminent	(RUNNING while at risk,
													 SUCCESS when not at risk)
				Task: FleeWhileAdvisable	(RUNNING while advisable,
											 SUCCESS when unadvisable)
			Task: Shooter/Shoot	(RUNNING until a shot is fired, then SUCCESS)


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

import rospy
from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *

class Behave():
	def __init__(self):
		BEHAVE = Sequence("behave")

		IS_TARGET_REMAINING = IsTargetRemaining("is_target_remaining", 1) # destroy 1 target
		PARALLEL_MOVE_SHOOT = Parallel


		# Run the tree
		while True:
			BEHAVE.announce()
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

class IsTargetLocationKnown(MonitorTask):
	def __init__(self, name, num_to_destroy, *args, **kwargs):
		super(IsTargetRemaining, self).__init__(name, *args, **kwargs)
	

	def run(self):
