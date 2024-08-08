#!/usr/bin/env python3.8

import rospy
import csv
from visualization_msgs.msg import Marker
import os

class WaypointVisualizer:
	def __init__(self, csv_file_path):
		self.csv_file_path = csv_file_path
		self.publisher = rospy.Publisher('waypoint_marker', Marker, queue_size=10)
		self.counter = 0

	def create_marker(self, waypoint, index):
		pose_marker = Marker()
		pose_marker.header.frame_id = "map"
		pose_marker.ns = "basic_shapes"
		pose_marker.type = 0
		# pose_marker.id = str(id) + '_pose'
		pose_marker.id = self.counter
		pose_marker.lifetime = rospy.Duration()
		pose_marker.action = Marker.ADD
		pose_marker.scale.x = 1.2
		pose_marker.scale.y = 0.5
		pose_marker.scale.z = 0.5
		if waypoint[4] == 0:
			pose_marker.color.a = 1.0
			pose_marker.color.r = 1.0
			pose_marker.color.g = 0.0
			pose_marker.color.b = 0.0
		elif waypoint[4] == 1:
			pose_marker.color.a = 1.0
			pose_marker.color.r = 1.0
			pose_marker.color.g = 1.0
			pose_marker.color.b = 0.0
		else:
			pose_marker.color.a = 1.0
			pose_marker.color.r = 1.0
			pose_marker.color.g = 1.0
			pose_marker.color.b = 1.0

		pose_marker.pose.position.x = waypoint[0]
		pose_marker.pose.position.y = waypoint[1]
		pose_marker.pose.position.z = 0.0
		pose_marker.pose.orientation.x = 0.0
		pose_marker.pose.orientation.y = 0.0
		pose_marker.pose.orientation.z = waypoint[2]
		pose_marker.pose.orientation.w = waypoint[3]
		self.counter += 1
		id_marker = Marker()
		id_marker.header.frame_id = "map"
		id_marker.ns = "basic_shapes"
		id_marker.type = Marker.TEXT_VIEW_FACING
		id_marker.text = str(index)
		id_marker.id = self.counter
		id_marker.lifetime = rospy.Duration()
		id_marker.action = Marker.ADD
		id_marker.scale.x = 3
		id_marker.scale.y = 3
		id_marker.scale.z = 3
		id_marker.color.a = 0.6
		id_marker.color.r = 1.0
		id_marker.color.g = 1.0
		id_marker.color.b = 1.0
		id_marker.pose.position.x = waypoint[0] + 1.0
		id_marker.pose.position.y = waypoint[1] + 1.0
		id_marker.pose.position.z = 0.0
		id_marker.pose.orientation.x = 0.0
		id_marker.pose.orientation.y = 0.0
		id_marker.pose.orientation.z = waypoint[2]
		id_marker.pose.orientation.w = waypoint[3]
		self.counter += 1

		return pose_marker, id_marker

	def read_waypoints_from_csv(self, file_path):
		with open(file_path, 'r') as file:
			reader = csv.reader(file)
			self.counter = 0
			for index, row in enumerate(reader):
				x, y = float(row[0]), float(row[1])
				oz, ow = float(row[5]), float(row[6])
				waypointType = int(row[7])
				pose_marker, id_marker = self.create_marker([x, y, oz, ow, waypointType], index)
				print(str(index) + ' ' + str(x) + ', ' + str(y))
				self.publisher.publish(pose_marker)
				self.publisher.publish(id_marker)

	def run(self):
			rate = rospy.Rate(1)  # 10Hz
			while not rospy.is_shutdown():
				self.read_waypoints_from_csv(self.csv_file_path )
				rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('waypoint_viewer', anonymous=True)
		ROS_WAYPOINT_FILE = os.getenv('ROS_WAYPOINT_FILE')
		if ROS_WAYPOINT_FILE is None:
			print(f"WARNING Environment variable 'ROS_WAYPOINT_FILE' not found")
			rospy.signal_shutdown()

		print('waypoint file : ' + ROS_WAYPOINT_FILE)
		visualizer = WaypointVisualizer(ROS_WAYPOINT_FILE)
		visualizer.run()
	except rospy.ROSInterruptException:
		pass
