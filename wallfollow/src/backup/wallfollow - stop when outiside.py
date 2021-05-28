#!/usr/bin/env python
import rospy, time
import datetime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# # Control Variables Initialization
command = Twist()


def scan_callback(msg):
    # Variables
    global range_front
    global range_right
    global range_left
    global ranges
    global xrange
    global min_front, i_front, min_right, i_right, min_left, i_left
    # get all the values from ranges
    ranges = msg.ranges
    xrange = range
    # in front of the robot (between 5 to -5 degrees)
    range_front[:5] = msg.ranges[5:0:-1]
    range_front[5:] = msg.ranges[-1:-5:-1]

    # to the right (between 300 to 345 degrees)
    range_right = msg.ranges[300:345]

    # to the left (between 15 to 60 degrees)
    range_left = msg.ranges[60:15:-1]

    # get the minimum values of each range
    # minimum value means the shortest obstacle from the robot
    min_range, i_range = min((ranges[i_range], i_range) for i_range in xrange(len(ranges)))
    min_front, i_front = min((range_front[i_front], i_front) for i_front in xrange(len(range_front)))
    min_right, i_right = min((range_right[i_right], i_right) for i_right in xrange(len(range_right)))
    min_left, i_left = min((range_left[i_left], i_left) for i_left in xrange(len(range_left)))


def initialization_of_robot():
    command.linear.x = 0.0
    command.angular.z = 0.0

    time.sleep(1)  # wait for node to initialize


def rotate_robot():
    # Turn the robot right at the start
    # to avoid the 'looping wall'
    print("Turning...")
    command.angular.z = -0.5
    command.linear.x = 0.1
    cmd_vel_pub.publish(command)
    time.sleep(2)


def stop_robot(command, cmd_vel_pub):
    print("Stopping...")
    while 1 and not rospy.is_shutdown():
        command.angular.z = 0.00
        command.linear.x = 0.00
        cmd_vel_pub.publish(command)


def forward(command, cmd_vel_pub):
    command.angular.z = 0.00
    command.linear.x = 0.175
    cmd_vel_pub.publish(command)


def forward_slow(command, cmd_vel_pub):
    command.angular.z = 0.00
    command.linear.x = 0.120
    cmd_vel_pub.publish(command)


def check_in_or_out(ranges):
    print("Forwarding...")
    inf_count = 0
    z = 'inf'
    for x in range(50):
        y = str(ranges[x])

        if y == z:
            inf_count = inf_count + 1
            print("1")

    for x in range(308, 358):
        y = str(ranges[x])
        if y == z:
            inf_count = inf_count + 1
            print("2")

    if inf_count > 90:
        inf_count = 0
        return 0
    else:
        inf_count = 0
        return 1


def inf_forward(ranges, command, cmd_vel_pub):
    check = check_in_or_out(ranges)
    really_out = 0
    if check == 0:
        print("seelipng")
        forward(command, cmd_vel_pub)
        time.sleep(5)
        print("Awaken")
    check = check_in_or_out(ranges)
    if check == 0:
        print("Outside Maze")
        return 0
    else:
        print("Inside Maze")
        return 1


def count_doors(ranges, command, cmd_vel_pub):
    y = str(ranges[269])
    z = 'inf'
    door_count = 0
    future_time = datetime.datetime.now() + datetime.timedelta(minutes=1)

    while datetime.datetime.now() == future_time:
        if y == z:
            print("Door detected")
            door_count += 1
            while y != z:
                print("Move Forward till door passes")
                forward_slow(command, cmd_vel_pub)
            print("Door Passed")
    print("2 Meters Completed")
    print("Door Count: " + door_count)
    stop_robot(command, cmd_vel_pub)


# Main Code
# Initialization
# Create the node
# Initialize all variables
ranges = []
range_front = []
range_right = []
range_left = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0
count = 0

inside_maze = 1
distance_out = 0
near_wall = 0  # start with 0, when we get to a wall, change to 1
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # to move the robot
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)  # to read the laser scanner
rospy.init_node('dmkk_node')
initialization_of_robot()
# rotate_robot()
rate = rospy.Rate(10)  # Rospy Rate

while not rospy.is_shutdown():
    # # Check if robot inside maze or not
    # inside_maze = inf_forward(ranges, command, cmd_vel_pub)
    # if inside_maze == 0:
    #     stop_robot(command, cmd_vel_pub)
    # else:
    #    rotate_robot()
    while near_wall == 0 and not rospy.is_shutdown():  # 1
        # Check if robot inside maze or not
        forward(command, cmd_vel_pub)
        inside_maze = inf_forward(ranges, command, cmd_vel_pub)
        if inside_maze == 0:
            stop_robot(command, cmd_vel_pub)
        print("Moving towards a wall.")
        if min_front > 0.3 and min_right > 0.3 and min_left > 0.3:
            command.angular.z = -0.1  # if nothing near, go forward
            command.linear.x = 0.15
        elif min_left < 0.3:  # if wall on left, start tracking
            near_wall = 1
        else:
            command.angular.z = -0.25  # if not on left, turn right
            command.linear.x = 0.0
        print("near wall =" + str(near_wall))
        cmd_vel_pub.publish(command)

    else:  # left wall following
        # Check if robot inside maze or not
        inside_maze = inf_forward(ranges, command, cmd_vel_pub)
        if inside_maze == 0:
            stop_robot(command, cmd_vel_pub)
        if min_front > 0.3:  # 2
            if min_left < 0.12:  # 3
                print("Range: {:.2f}m is too close, Going backward".format(min_left))
                command.angular.z = -1.2
                command.linear.x = -0.1
            elif min_left > 0.20:  # 4
                print("Range: {:.2f}m  follwing the wall, turn left.".format(min_left))
                command.angular.z = 0.75
                command.linear.x = 0.20
            else:
                print("Range: {:.2f}m follwing the wall, turn right.".format(min_left))
                command.angular.z = -0.75
                command.linear.x = 0.15

        else:  # 5
            print("Front obstacle detected. Turning away.")
            command.angular.z = -1.0
            command.linear.x = 0.0
            cmd_vel_pub.publish(command)
            print("ranges =" + str(ranges))
            # while min_front < 0.3 and not rospy.is_shutdown():
            #     print("ranges ="+str(ranges))
            #     pass
        # publish command
        cmd_vel_pub.publish(command)
        # wait for the loop
    rate.sleep()
