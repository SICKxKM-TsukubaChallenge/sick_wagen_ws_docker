#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 走行中に５秒おきに座標（正確にはmapからbase_linkのtf）を取得しcsvに保存するスクリプト。頻度はrospy.Rateで変えられます。起動する前に既存のcsvをrm -rfしておく必要がある。


import roslib
# roslib.load_manifest('sick_wagen')
import rospy
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Joy
import turtlesim.srv
import csv

csvfile = "test20230905.csv"

def joy_callback(msg):
    save_signal = msg.buttons[2]
    
    if save_signal:
            print("[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3]))
            with open(csvfile, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([trans[0], trans[1], 0.0, 0.0, 0.0, rot[2], rot[3]])
            save_signal = 0
            former_save =rospy.Time.now()


if __name__ == '__main__':

    with open(csvfile, 'w') as fc:
        writer = csv.writer(fc)
    fc.close

    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    rospy.Subscriber("/joy", Joy, joy_callback)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
    rospy.Subscriber("/joy", Joy, joy_callback)
    rate.sleep()
