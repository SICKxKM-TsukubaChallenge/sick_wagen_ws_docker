#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import PointCloud2

class Node():
	def __init__(self, subscriber_name, publisher_name, target_frame):
		self.sub1 = rospy.Subscriber(subscriber_name, PointCloud2, self.subscriber_pointcloud) #subscriber1
		self.pub = rospy.Publisher(publisher_name, PointCloud2, queue_size=3) #publisher
		self.target_frame = target_frame
		self.point_step = 16

	def subscriber_pointcloud(self, msg):
		msg.fields[3].name = "distances"
		
		new_msg = PointCloud2(
			header = msg.header, height = msg.height, width = msg.width,
			fields = msg.fields, is_bigendian = msg.is_bigendian, point_step = msg.point_step,
			row_step = msg.row_step, data = msg.data, is_dense = msg.is_dense
		)

		new_msg.header.frame_id = "multiscan_link"
		self.pub.publish(new_msg)
	
if __name__ == '__main__':
	subscriber_name = "/pointcloud"
	publisher_name = "/pointcloud_out"
	target_frame = "multiscan_link"

	rospy.init_node('node1')
	node1 = Node(subscriber_name, publisher_name, target_frame)

	while not rospy.is_shutdown():
		rospy.sleep(0.001)
