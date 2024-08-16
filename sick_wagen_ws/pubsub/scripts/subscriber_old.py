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
        self.sub2 = rospy.Subscriber('/clock', Clock, self.subscriber_clock) #subscriber2
        self.pub = rospy.Publisher(publisher_name, PointCloud2, queue_size=1) #publisher
        self.point_step = 16
        #self.calring_array = np.vectorize(util.calring)
        #self.LID_array = np.vectorize(util.angle2LID)

    def subscriber_pointcloud(self, msg):
        global pointcloud_data_2d, iter
        #print("time_header : {}".format(msg.header.stamp))

        # timestampは点群データのHeaderから出力して大丈夫そう

        iter = int(len(msg.data)/self.point_step) #32400 points/scan
        array_message_data = np.array(list(msg.data))
        array_split = np.array(np.array_split(array_message_data, iter), dtype=np.uint8)

        # get x, y, z and intensity values(4Byte list), shape = (32400, 4)
        x_raw = array_split[:, 0:4]
        y_raw = array_split[:, 4:8]
        z_raw = array_split[:, 8:12]
        #intensity_raw = array_split[:, 12:16]

        # get x, y, z and intensity values(float32, list), shape = (32400, 1)
        x = np.apply_along_axis(util.binary2float_array, axis=1, arr = x_raw)
        y = np.apply_along_axis(util.binary2float_array, axis=1, arr = y_raw)
        z = np.apply_along_axis(util.binary2float_array, axis=1, arr = z_raw)
        #intensity = np.apply_along_axis(util.binary2float_array, axis=1, arr = intensity_raw)

        # calculate angle and Laser ID
        #angle_array = self.calring_array(x, y, z)
        #ring = self.LID_array(angle_array)

        # generate pointcloud array
        #tmp = np.column_stack((x, y, z, intensity//256, ring))
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
            #for i in range(len(pointcloud_data_2d)):
                #pointcloud_data_2d[i][4] = int(pointcloud_data_2d[i][4]) # format(ring) transformation (from float to int)
            self.publish(pointcloud_data_2d, clock_data)

    def publish(self, pointcloud, timestamp):
        if len(pointcloud) == iter:
            HEADER = Header(frame_id='base_link')
            HEADER.stamp = timestamp
            
            """
            FIELDS = [ PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1) ]
            """

            FIELDS = [ PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
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

    subscriber_name = "/multiscan/cloud_360"
    publisher_name = "/ring_cloud"

    rospy.init_node('test_node')
    node = Node(subscriber_name, publisher_name)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)
