#!/usr/bin/env python
'''
Ajay Jain
July 3, 2014
ARSENL Lab, Naval Postgraduate School
'''

import math

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

def extrapolate_twist(vel, secs):
	dist  = vel.linear.x * secs
	angle = vel.angular.z * sec

# OTHER UTILS

def truncate(val, max_val):
	if val >= 0:
		return max_val if val > max_val else val
	else:
		return -max_val if val < -max_val else val