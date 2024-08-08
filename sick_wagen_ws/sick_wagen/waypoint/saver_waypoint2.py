#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 走行中に５秒おきに座標（正確にはmapからbase_linkのtf）を取得しcsvに保存するスクリプト。頻度はrospy.Rateで変えられます。起動する前に既存のcsvをrm -rfしておく必要がある。


import roslib
# roslib.load_manifest('sick_wagen')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import csv

csvfile = "2tsukuba1120.csv"

if __name__ == '__main__':

    with open("sick10_demo1125_1.csv", 'w') as fc:
        writer = csv.writer(fc)
    fc.close

    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        print "[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3])

        with open(csvfile, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3]])


        rate.sleep()
