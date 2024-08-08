#!/usr/bin/env python

import signal
import rospy

cont = True

def handler(signal, frame):
    global cont
    cont = False

def the_node():
    global cont

    rospy.init_node('the_node', disable_signals = True)

    rate = rospy.Rate(40)
    while cont:
        ~~~
        rate.sleep()

    rospy.signal_shutdown('finish')
    rospy.spin()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    the_node()

