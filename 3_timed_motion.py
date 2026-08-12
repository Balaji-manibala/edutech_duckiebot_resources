#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped

rospy.init_node("timed_motion_sequence")

pub = rospy.Publisher(
    "/duck4/wheels_driver_node/wheels_cmd",
    WheelsCmdStamped,
    queue_size=1
)

rate = rospy.Rate(10)


def move(left, right, duration):

    msg = WheelsCmdStamped()
    msg.vel_left = left
    msg.vel_right = right

    start = rospy.Time.now()

    while rospy.Time.now() - start < rospy.Duration(duration):
        pub.publish(msg)
        rate.sleep()


def stop():

    msg = WheelsCmdStamped()
    msg.vel_left = 0.0
    msg.vel_right = 0.0

    for i in range(10):
        pub.publish(msg)
        rate.sleep()


try:

    # Forward
    move(0.3, 0.3, 3)
    stop()

    # Reverse
    move(-0.3, -0.3, 3)
    stop()

    # Turn Left
    move(0.1, 0.3, 2)
    stop()

    # Turn Right
    move(0.3, 0.1, 2)
    stop()

    print("Motion sequence completed.")

except KeyboardInterrupt:

    print("\nStopping robot...")
    stop()
    print("Robot stopped.")
