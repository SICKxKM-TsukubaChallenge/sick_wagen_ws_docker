#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse

def handle_empty(req):
	rospy.loginfo("Empty service called")
	return EmptyResponse() 

def empty_service_server():
	rospy.init_node('joy2call_testserver')
	service = rospy.Service('restart', Empty, handle_empty)
	rospy.loginfo("Empty service is ready to receive requests")
	rospy.spin()

if __name__ == "__main__":
	empty_service_server()