#! /usr/bin/env pyhton

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node("mover", anonymous=True)

move_cmd = Twist()
move_cmd.linear.x = 0.0
move_cmd.angular.z = 0.0

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(move_cmd)
    rate.sleep()
