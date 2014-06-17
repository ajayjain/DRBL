#!/usr/bin/env python
'''
Author: Ajay Jain
'''

import Leap, sys, thread, time, math

import rospy
from geometry_msgs.msg import Twist, Vector3

MAX_LINEAR 		= 0.3 # m/s
MAX_RAD 		= math.pi / 4
THRESHOLD_PITCH = math.pi / 8
THRESHOLD_ROLL  = math.pi / 8

class SampleListener(Leap.Listener):
	finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
	bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
	state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

	linear = Vector3(0, 0, 0)
	angular = Vector3(0, 0, 0) # radians per second
	twist = Twist(linear, angular)

	pub = rospy.Publisher('cmd_vel', Twist)

	def compute_twist(self, roll, pitch, yaw):
		lin = abs(pitch) >= THRESHOLD_PITCH
		ang = abs(roll)  >= THRESHOLD_ROLL
		self.linear.x  = MAX_LINEAR * (-1 * pitch / 2 * math.pi) if lin else 0
		self.angular.z = MAX_RAD    * (-1 * roll / 2 * math.pi)  if ang else 0

	def publish_twist(self):
		self.pub.publish(self.twist)

	def on_init(self, controller):
		print "Initialized"

	def on_connect(self, controller):
		print "Connected"

	def on_disconnect(self, controller):
		# Note: not dispatched when running in a debugger.
		print "Disconnected"

	def on_exit(self, controller):
		print "Exited"

	def on_frame(self, controller):
		# Get the most recent frame and report some basic information
		frame = controller.frame()

		print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
			  frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

		if not frame.hands.is_empty:
			hand = frame.hands.rightmost

			# Get the hand's normal vector and direction
			normal = hand.palm_normal
			direction = hand.direction

			# pitch = direction.pitch * Leap.RAD_TO_DEG
			# roll = normal.roll * Leap.RAD_TO_DEG
			# yaw = direction.yaw * Leap.RAD_TO_DEG
			# self.compute_twist(pitch, roll, yaw)

			# RPY in radians
			roll, pitch, yaw = direction.pitch, normal.roll, direction.yaw
			self.compute_twist(pitch, roll, yaw)
			self.publish_twist()

			# Calculate the hand's pitch, roll, and yaw angles
			print "  pitch: %f rad, roll: %f rad, yaw: %f rad" % (pitch, roll, yaw)

		print ""

	def state_string(self, state):
		if state == Leap.Gesture.STATE_START:
			return "STATE_START"

		if state == Leap.Gesture.STATE_UPDATE:
			return "STATE_UPDATE"

		if state == Leap.Gesture.STATE_STOP:
			return "STATE_STOP"

		if state == Leap.Gesture.STATE_INVALID:
			return "STATE_INVALID"

def main():
	rospy.init_node("leap_control")

	# Create a sample listener and controller
	listener = SampleListener()
	controller = Leap.Controller()

	# Have the sample listener receive events from the controller
	controller.add_listener(listener)

	rospy.spin()

	# # Keep this process running until Enter is pressed
	# print "Press Enter to quit..."
	# try:
	# 	sys.stdin.readline()
	# except KeyboardInterrupt:
	# 	pass
	# finally:
	# 	# Remove the sample listener when done
	# 	controller.remove_listener(listener)


if __name__ == "__main__":
	main()