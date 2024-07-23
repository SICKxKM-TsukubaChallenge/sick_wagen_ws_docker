#!/usr/bin/env python3.8
import rospy
from nav_msgs.msg import Odometry

class Odom_Node:
    def __init__(self) :
        rospy.init_node("odom_node", anonymous=True)
        self.sub = rospy.Subscriber("/whill/odom", Odometry, self.callback)
        self.pub = rospy.Publisher("/odom_fixed", Odometry, queue_size = 8)
        self.rate = rospy.Rate(30)
    
    def callback(self, data):
        self.pub.publish(data)
    
    def run(self):
        while not rospy.is_shutdown():
            #self.rate.sleep()
            rospy.sleep(0.05)

if __name__ == "__main__":
    try:
        node = Odom_Node()
        node.run()

    except rospy.ROSInterruptException:
        pass
