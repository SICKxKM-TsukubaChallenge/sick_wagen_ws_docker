#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg):
    rospy.loginfo("Received pose estimate from RViz:\nPosition(x,y,z): (%.2f, %.2f, %.2f) Orientation(x,y,z,w): (%.2f, %.2f, %.2f, %.2f)",
                  msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                  msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    # トピックにポーズ情報をパブリッシュ
    pose_pub.publish(msg)

rospy.init_node('ekf_setpose_relay')
pose_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=10)
rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, pose_callback)
rospy.spin()