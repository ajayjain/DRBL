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

# vec is [x, y...] (remainder preserved), radians is the positive or negative angle
def rotate(vec, radians):
	x, y = vec[0], vec[1]
	r = math.hypot(x, y)
	theta_0 = math.atan2(y, x)
	theta_f = theta_0 - radians
	xf = r * math.cos(theta_f)
	yf = r * math.sin(theta_f)
	return [xf, yf] + vec[2:]

# ETRAPOLATION, POSE, TWIST METHODS

def pose_translation(own_pose, target_pose):
	return [tp - op for (tp, op) in zip(target_pose[0], own_pose[0])]

def extrapolate(pose, vel, secs):
	dist = vel.linear.x * secs
	dtheta = vel.angular.z * secs

	quat = pose[1]
	euler = euler_from_quaternion(quat)
	theta_f = dtheta + euler[2]
	euler = (euler[0], euler[1], theta_f)

	return extend_pose(pose, dist, euler)

def extend_pose(pose, dist, euler_orientation_f):
	theta_f = euler_orientation_f[2]
	x = pose[0][0] + dist * math.cos(theta_f)
	y = pose[0][1] + dist * math.sin(theta_f)
	z = pose[0][2]

	quat = quaternion_from_euler(euler_orientation_f[0], euler_orientation_f[1], theta_f)

	return ((x, y, z), tuple(quat))

def get_yaw(pose):
	quat = pose[1]
	euler = euler_from_quaternion(quat)
	return euler[2]

# OTHER UTILS

def truncate(val, max_val):
	if val >= 0:
		return max_val if val > max_val else val
	else:
		return -max_val if val < -max_val else val