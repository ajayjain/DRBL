#!/usr/bin/env python
'''
Ajay Jain
Created June 28, 2014
ARSENL Lab, Naval Postgraduate School
'''

import rospy
from nav_msgs.msg import Odometry

pub = None
frame_prefix = 'robot_0/'

def on_odom(odom):
    odom.header.frame_id = frame_prefix + odom.header.frame_id
    odom.child_frame_id = frame_prefix + odom.child_frame_id
    pub.publish(odom)

def main():
    global pub, frame_prefix

    rospy.init_node("encoder_frame_remapper")

    frame_prefix = rospy.get_param('~frame_prefix', frame_prefix)
    pub = rospy.Publisher("odom", Odometry)
    rospy.Subscriber("encoder", Odometry, on_odom)

    rospy.spin()

if __name__ == "__main__":
    main()