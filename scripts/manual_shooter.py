#!/usr/bin/env python
'''
Ajay Jain
Created July 15, 2014
ARSENL Lab, Naval Postgraduate School
'''

import roslib; roslib.load_manifest("husky_pursuit")
import rospy, math
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

fire_pub = rospy.Publisher("status/fire", Bool)
fire_msg = Bool(True)

fire_button = rospy.get_param('~fire_button', 0)
semiauto = rospy.get_param('~semiauto', True)

released = True

def on_joy(joy):
	global released

	button_state = joy.buttons[fire_button]
	clear_to_fire = (semiauto and released) or not semiauto

	print "button_state, clear_to_fire, semiauto, released"
	print button_state, clear_to_fire, semiauto, released

	if button_state == 1 and clear_to_fire:
		print "FIRE"
		fire_pub.publish(fire_msg)

	released = True if button_state == 0 else False

def main():
	rospy.init_node("manual_shooter")
	rospy.Subscriber("joy", Joy, on_joy)
	rospy.spin()

if __name__ == "__main__":
	main()