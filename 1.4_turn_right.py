#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped

rospy.init_node("turn_right", disable_signals=True)

pub = rospy.Publisher(
    "/duck4/wheels_driver_node/wheels_cmd",
    WheelsCmdStamped,
    queue_size=1
)

rate = rospy.Rate(10)

try:
    while True:
        msg = WheelsCmdStamped()

        msg.vel_left = 0.3
        msg.vel_right = 0.1

        pub.publish(msg)
        rate.sleep()

except KeyboardInterrupt:
    print("\nStopping robot...")

    msg = WheelsCmdStamped()
    msg.vel_left = 0.0
    msg.vel_right = 0.0

    for i in range(20):
        pub.publish(msg)
        rate.sleep()

    print("Robot stopped.")
