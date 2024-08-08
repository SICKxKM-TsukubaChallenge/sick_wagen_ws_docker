#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Joy
import csv
import os
from datetime import datetime
from visualization_msgs.msg import Marker

ROS_WAYPOINT_DIR = os.getenv('ROS_WAYPOINT_DIR')
if ROS_WAYPOINT_DIR is None:
    print(f"WARNING Environment variable 'ROS_WAYPOINT_DIR' not found")
    rospy.signal_shutdown()

current_time_str = datetime.now().strftime('%Y%m%d_%H%M%S')
filename = f"{current_time_str}.csv"
csvfile = os.path.join(ROS_WAYPOINT_DIR, filename)

def joy_callback(msg):
	#(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	if msg.buttons[2]:
		print("[{0},{1},0.0,0.0,0.0,{2},{3}, 0],".format(trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3]))
		with open(csvfile, 'a') as f:
			writer = csv.writer(f)
			writer.writerow([trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3], 0])
		print("Set continue pose")
	elif msg.buttons[3]:
		print("[{0},{1},0.0,0.0,0.0,{2},{3}, 1],".format(trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3]))
		with open(csvfile, 'a') as f:
			writer = csv.writer(f)
			writer.writerow([trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3], 1])
		print("Set temporary stop pose")

if __name__ == '__main__':
	with open(csvfile, 'w') as fc:
		writer = csv.writer(fc)
	fc.close

	rospy.init_node('waypoint_saver')

	listener = tf.TransformListener()
	rate = rospy.Rate(10)
	rospy.Subscriber("/joy", Joy, joy_callback)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		#rate.sleep()
