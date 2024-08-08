#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# csvに保存されているwaypointに沿ってmove_baseにgoalを指定するスクリプト。
#!/usr/bin/env python2

import rospy
import actionlib
import tf
import csv
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#waypoints = [
#        [(1.502096,1.190246,0.0),(0.0,0.0,0.168102,0.995941)],
#        [(2.355508,-0.00325,0.0),(0.0,0.0,0.935034,0.354558)],
#        [(-4.607642,-1.421862,0.0),(0.0,0.0,0.997357,-0.072662)],
#        [(-5.442663,1.507219,0.0),(0.0,0.0,0.090007,0.995941)]
#        ]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':

    waypoints = []
    with open('test20230905.csv', 'r') as f:
        reader = csv.reader(f)
        for one_line in reader:
            #one_line = one_line.strip()
            #one_line = one_line.split(' ')
            print(one_line)
            forward = (float(one_line[0]), float(one_line[1]), float(one_line[2]))
            back = (float(one_line[3]), float(one_line[4]), float(one_line[5]), float(one_line[6]))
            #forward = (map(float, one_line)[0], map(float, one_line)[1], map(float, one_line)[2])
            #back = (map(float, one_line)[3], map(float, one_line)[4], map(float, one_line)[5], map(float, one_line)[6])
            waypoints.append([forward, back])

    rospy.init_node('patrol')
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
    #listener.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(4.0))

    while True:
        for pose in waypoints:
            goal = goal_pose(pose)
            client.send_goal(goal)
            while True:
                now = rospy.Time.now()
                listener.waitForTransform("/map", "/base_link", now, rospy.Duration(0.1))
                position, quaternion = listener.lookupTransform("/map", "/base_link", now)
                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                    print("next goal : {0} {1} 0.0 0.0 0.0 {2} {3}".format(pose[0][0],pose[0][1],pose[1][2],pose[1][3]))
                    print("next!!!")
                    break

                else:
                    rospy.sleep(0.5)
        break
