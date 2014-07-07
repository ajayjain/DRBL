#!/usr/bin/env python
'''
Ajay Jain
July 3, 2014
ARSENL Lab, Naval Postgraduate School
'''

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ARRAY "VECTOR" OPERATIONS

def normalize(vec):
	mag = magnitude(vec)
	try:
		return [comp / mag for comp in vec]
	except ZeroDivisionError:
		print "ZeroDivisionError: magnitude of vector is 0 in normalize(vec)"
		return vec

def magnitude(vec):
	added = sum([comp ** 2 for comp in vec])
	return math.sqrt(added)

def mult(vec, const):
	return [const * comp for comp in vec]

def add(vec1, vec2):
	# the return array length is equal to the minimum of the two inputs
	return [e1 + e2 for (e1, e2) in zip(vec1, vec2)]

def pose_translation(own_pose, target_pose):
	dx = target_pose.position.x - own_pose.position.x
	dy = target_pose.position.y - own_pose.position.y
	dz = target_pose.position.z - own_pose.position.z
	return [dx, dy, dz]

def extrapolate(pose, vel, secs):
	dist = vel.linear.x * secs
	dtheta = vel.angular.z * sec

	(w, x, y, z) = pose.orientation
	euler = euler_from_quaternion([w, x, y, z])
	theta_f = dtheta + euler[2]
	euler[2] = theta_f

	return extend_pose(pose, dist, euler)

def extend_pose(pose, dist, euler_orientation_f):
	theta_f = euler_orientation_f[2]
	pose.position.x += dist * math.cos(theta_f)
	pose.position.y += dist * math.sin(theta_f)

	quat = quaternion_from_euler(euler_orientation_f)
	pose.orientation.w = quat[0]
	pose.orientation.w = quat[1]
	pose.orientation.w = quat[2]
	pose.orientation.w = quat[3]

	return pose # original object was modified, but let's return it anyway


# OTHER UTILS

def truncate(val, max_val):
	if val >= 0:
		return max_val if val > max_val else val
	else:
		return -max_val if val < -max_val else val