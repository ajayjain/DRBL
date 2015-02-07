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
				Task: ChaseWhileAdvisable	(RUNNING while advisable,
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

import rospy, math, random
import geometry_msgs.msg

from husky_pursuit.msg import RelativePosition
import pure_seek
import utils

from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *

# Parameters for the robot itself
class Me():
	MAX_LIN = rospy.get_param('~linear_vel_max',  0.8)
	MAX_ANG = rospy.get_param('~angular_vel_max', math.pi/2)
	CMD_FREQ  = 10.0

	WANDER_SECONDS_PER_TRANSLATION = 3
	WANDER_LOOP_ON_ITER = CMD_FREQ * WANDER_SECONDS_PER_TRANSLATION

class MotionValidator():
	@classmethod
	def setup(cls):
		cls.vel_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, latch=True) # remap this

	@classmethod
	def will_hit_obstacle(cls, vel):
		#TODO Forecast trajectory from velocity and compare to depth data or vision
		return False

	@classmethod
	def validate_and_publish(cls, vel):
		print "MotionValidator got velocity ", vel
		if cls.will_hit_obstacle(vel):
			new_vel = geometry_msgs.msg.Twist()
			new_vel.angular = math.copysign(Me.MAX_ANG, vel.angular) # if angular is near 0, the resulting turn is ~arbitrary
			cls.vel_pub.publish(new_vel)
		else:
			print "Publishing velocity"
			cls.vel_pub.publish(vel)
		rospy.sleep(1.0/Me.CMD_FREQ)

class BehaviorCoordinator():
	def __init__(self):
		# The root node
		BEHAVE = Sequence("behave")
		# Track 1 target
		TARGET_TRACKER = TargetTracker(1)

		# Composite task: returns SUCCESS as soon as any subtask returns SUCCESS
		PARALLEL_MOVE_SHOOT = ParallelOne("Moving and Shooting in Parallel")

		RESPOND_TO_TARGET = Sequence("respond to target")

		# Behavior task instances
		WANDER = WanderWhileTargetLocationUnknown("wander to find target", TARGET_TRACKER)
		CHASE_TARGET = ChaseWhileAdvisable("chase target", TARGET_TRACKER)
		# AVOID_TARGET = AvoidTargetWhileAdvisable("avoid target", TARGET_TRACKER)

		RESPOND_TO_TARGET.add_child(WANDER)
		RESPOND_TO_TARGET.add_child(CHASE_TARGET)
		# RESPOND_TO_TARGET.add_child(AVOID_TARGET)

		PARALLEL_MOVE_SHOOT.add_child(RESPOND_TO_TARGET)

		BEHAVE.add_child(PARALLEL_MOVE_SHOOT)

		# Print a simple representation of the tree
		print "Behavior Tree Structure"
		print_tree(BEHAVE)

		# Run the tree
		while TARGET_TRACKER.is_target_remaining():
			BEHAVE.announce()
			status = BEHAVE.run()
			if status == TaskStatus.SUCCESS:
				print "Finished running tree. Resetting and allowing to continue until no targets remain."
				BEHAVE.reset()


class TargetTracker():
	def __init__(self, num_to_destroy):
		self.num_destroyed = 0
		self.targets = [Target() for i in xrange(num_to_destroy)]
		print "Created TargetTracker, num=%d, targets=" % num_to_destroy, self.targets

	def is_target_remaining(self):
		#TODO: Actually check if we have destroyed anything, and update the counter
		if self.num_destroyed < len(self.targets):
			return True
		return False

	def choose_first_target(self):
		if len(self.targets) > 0:
			for target in self.targets:
				#TODO: Choose eg closest target
				if target.is_location_known():
					return target # chooses the first target with a known location
		return None

class Target():
	def __init__(self, health=2, topic='/target_relative', timeout=2): # 2 second timeout
		self.health = health
		self.time_last_found = None
		self.timeout = timeout
		self.rtheta = None
		rospy.Subscriber(topic, RelativePosition, self.on_relative) # remap this in the launch file

	def on_relative(self, rel_pos):
		self.time_last_found = rospy.get_time()
		self.rtheta = [rel_pos.range, rel_pos.bearing]

	def is_location_known(self):
		now = rospy.get_time()
		if (self.rtheta != None) and (self.time_last_found + self.timeout >= now):
			return True
		return False

class WanderWhileTargetLocationUnknown(Task):
	def __init__(self, name, target_tracker, *args, **kwargs):
		super(WanderWhileTargetLocationUnknown, self).__init__(name, *args, **kwargs)
		
		self.name = name
		self.target_tracker = target_tracker

		self.count = Me.WANDER_LOOP_ON_ITER
		self.trans = None

		print "Created task WanderWhileTargetLocationUnknown"

	def random_translation(self):
		x = random.uniform(-1, 1)
		y = random.uniform(-10, 10)
		return (x, y, 0)

	def run(self):
		self.announce()
		while self.target_tracker.choose_first_target() == None:
			if self.count == Me.WANDER_LOOP_ON_ITER:
				trans = self.random_translation()
				self.count = 0
			self.count += 1

			# Seek the random translation and send off
			print "Seeking trans", trans
			vel = pure_seek.seek(trans, Me.MAX_LIN, Me.MAX_ANG)
			vel.angular.z = utils.truncate(vel.angular.z, vel.linear.x/2) #cap rotation speed
			MotionValidator.validate_and_publish(vel)

		return TaskStatus.SUCCESS

class ChaseWhileAdvisable(Task):
	def __init__(self, name, target_tracker, *args, **kwargs):
		super(ChaseWhileAdvisable, self).__init__(name, *args, **kwargs)
		
		self.name = name
		self.target_tracker = target_tracker

		print "Created task ChaseWhileAdvisable"

	def is_chase_advisable(self, target):
		return True
		second_quadrant = 0 <= target.rtheta <= math.pi/2
		first_quadrant = 1.5*math.pi <= target.rtheta <= 2*math.pi
		return second_quadrant or first_quadrant

	def run(self):
		self.announce()
		target = self.target_tracker.choose_first_target()
		while self.is_chase_advisable(target):
			print "Chase advisable, seeking rtheta=", target.rtheta
			vel = pure_seek.seek_rtheta(target.rtheta, Me.MAX_LIN, Me.MAX_ANG)
			MotionValidator.validate_and_publish(vel)

			# If a position estimate is lost, get out (go back to Wander)
			if not target.is_location_known():
				print "Target location lost. Chase failure."
				return TaskStatus.FAILURE

		return TaskStatus.SUCCESS

if __name__ == "__main__":
	rospy.init_node("behave_tree")
	
	MotionValidator.setup()
	BehaviorCoordinator()

	rospy.spin()