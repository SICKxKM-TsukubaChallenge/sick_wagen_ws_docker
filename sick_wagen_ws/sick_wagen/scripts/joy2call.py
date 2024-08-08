#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

def joy_callback(data, restart_service):
	B_btn = data.buttons[2]
	if B_btn == 1:
		rospy.loginfo("Button B pressed")
		restart_service()

def joy_listener():
	rospy.init_node('joy2call', anonymous=True)
	rospy.wait_for_service('restart')
	restart_service = rospy.ServiceProxy('restart', Empty)

	rospy.Subscriber('joy', Joy, lambda data: joy_callback(data, restart_service))
	rospy.spin()

if __name__ == '__main__':
	joy_listener()
