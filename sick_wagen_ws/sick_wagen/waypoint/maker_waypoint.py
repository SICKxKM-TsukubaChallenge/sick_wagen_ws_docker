#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# waypointを2D-NAV-GOALで逐次指定してCSVに保存し、waypointとするスクリプト。csvは毎回rm -rfで削除してください。しないと前回のwaypointが残ってしまいます。

import signal
import rospy
import csv
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

csvfile = "test1007.csv"
cont = True

#def handler(signal, grame):
    #global cont
    #cont = False

def callback(data):
    pos = data.goal.target_pose.pose
    print("[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))
    with open(csvfile, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([pos.position.x, pos.position.y, 0.0, 0.0, 0.0, pos.orientation.z, pos.orientation.w])

def listener():
    # global cont
    rospy.init_node('goal_sub', disable_signals=True)
    # rate = rospy.Rate(0.2) 
    #while cont:
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)
        # rate.sleep()
    # rospy.signal_shutdown("finish")
    rospy.spin()

if __name__ == '__main__':
    #signal.signal(signal.SIGINT, handler)
    listener()
