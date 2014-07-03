#!/usr/bin/env python
'''
Ajay Jain
July 1, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy, tf, math
from gazebo_msgs.srv import GetModelState

def get_model_state_client(model_name):
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        resp = get_model_state(model_name, '')
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Returns (x, y, z)
def gazebo_translation():
	MODEL1 = rospy.get_param('~robot1_model_name', 'Robot1')
	MODEL2 = rospy.get_param('~robot2_model_name', 'Robot2')

	pos1 = get_model_state_client(MODEL1).pose.position
	pos2 = get_model_state_client(MODEL2).pose.position

	x = round(pos2.x - pos1.x, 9)
	y = round(pos2.y - pos1.y, 9)
	z = round(pos2.z - pos1.z, 9)

	# return ((x, y, z), (0, 0, 0, 1)) # TODO: Actual rotation difference
	return (x, y, z)

def transforms_translation(listener):
	parent = rospy.get_param('~robot1_frame_id', '/robot1_tf/base_footprint')
	child  = rospy.get_param('~robot2_frame_id', '/robot2_tf/base_footprint')

	try:
		listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(3.0))
		(trans, rot) = listener.lookupTransform(parent, child, rospy.Time(0))
		return trans
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
		print "transforms exception: %s"%e
		return None


if __name__=="__main__":
	rospy.init_node('get_data')

	print get_model_state_client('Robot1')
	print '\n'
	print get_model_state_client('Robot2')
	print '\n'
	print gazebo_translation()
	print '\n'
	listener = tf.TransformListener()
	print transforms_translation(listener)

	rospy.spin()
