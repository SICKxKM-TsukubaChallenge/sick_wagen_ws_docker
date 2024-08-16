#!/usr/bin/env/python3.8
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
    def __init__(self, subscribe_topic, publish_topic):
        self.sub1 = rospy.Subscriber(subscribe_topic, PointCloud2, self.subscriber_pointcloud) #subscriber1
        self.pub = rospy.Publisher(publish_topic, PointCloud2, queue_size=1) #publisher

    def subscriber_pointcloud(self, msg):
        global pointcloud_data_2d, iter
        iter = int(len(msg.data)/16) #32400 points/scan
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
        self.data_process()

    def subscriber_clock(self, msg):
        global clock_data
        clock_data = msg.clock
        self.data_process()

    def data_process(self):
        global pointcloud_data_2d, clock_data
        if pointcloud_data_2d is not None and clock_data is not None:
            self.publish(pointcloud_data_2d, clock_data)

    def publish(self, pointcloud, timestamp):
        if len(pointcloud) == iter:
            HEADER = Header(frame_id='/point_frame')
            HEADER.stamp = timestamp
            
            FIELDS = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

            POINTS = pointcloud
            point_cloud_publish = pc2.create_cloud(HEADER, FIELDS, POINTS)
            point_cloud_publish.is_dense = True
            self.pub.publish(point_cloud_publish)
        else:
            print("size of pointcloud is invalid")
    
if __name__ == '__main__':
    pointcloud_data_2d = None
    clock_data = None

    subscribe_topic_name1 = "/cloud_360"
    publish_topic_name1 = "/multiscan_xyz"
    #subscribe_topic_name2 = "/merged_cloud"
    #publish_topic_name2 = "/tim_xyz"

    node_name1 = "node1"
    #node_name2 = "node2"

    rospy.init_node(node_name1)
    #rospy.init_node(node_name2)

    node1 = Node(subscribe_topic_name1, publish_topic_name1)
    #node2 = Node(subscribe_topic_name2, publish_topic_name2)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)


