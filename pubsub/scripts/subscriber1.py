#!/usr/bin/env python3.8
import struct, math
import util
import message_filters
import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from rosgraph_msgs.msg import Clock
import sensor_msgs.point_cloud2 as pc2

class Node():
    def __init__(self, subscriber_name, publisher_name):
        self.sub1 = rospy.Subscriber(subscriber_name, PointCloud2, self.subscriber_pointcloud) #subscriber1
        self.pub = rospy.Publisher(publisher_name, PointCloud2, queue_size=3) #publisher
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

        """
        iter = int(len(msg.data)/self.point_step) #32400 points/scan
        array_message_data = np.array(list(msg.data))
        array_split = np.array(np.array_split(array_message_data, iter), dtype=np.uint8)

        # get x, y, z and intensity values(4Byte list), shape = (32400, 4)
        x_raw = array_split[:, 0:4]
        y_raw = array_split[:, 4:8]
        z_raw = array_split[:, 8:12]

        # get x, y, z and intensity values(float32, list), shape = (32400, 1)
        x = np.apply_along_axis(util.binary2float_array, axis=1, arr = x_raw)
        y = np.apply_along_axis(util.binary2float_array, axis=1, arr = y_raw)
        z = np.apply_along_axis(util.binary2float_array, axis=1, arr = z_raw)

        # generate pointcloud array
        tmp = np.column_stack((x, y, z))
        pointcloud_data_2d = tmp.tolist()
        clock_data = msg.header.stamp
        self.publish(pointcloud_data_2d, clock_data)
        """
    def publish(self, pointcloud, timestamp):
        HEADER = self.header
        HEADER.stamp = timestamp

        POINTS = pointcloud
        point_cloud_publish = pc2.create_cloud(HEADER, self.fields, POINTS)
        point_cloud_publish.is_dense = True
        self.pub.publish(point_cloud_publish)
    
if __name__ == '__main__':

    subscriber_name = "/pointcloud"
    publisher_name = "/pointcloud_out"

    rospy.init_node('node1')
    node1 = Node(subscriber_name, publisher_name)

    while not rospy.is_shutdown():
        rospy.sleep(0.001)
