#! /usr/bin/env pyhton

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print(len(msg.ranges))
    print("Front: " + str(msg.ranges[0]))
    print("Left: " + str(msg.ranges[60]))
    print("Behind: " + str(msg.ranges[179]))
    print("Right: " + str(msg.ranges[269]))


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
