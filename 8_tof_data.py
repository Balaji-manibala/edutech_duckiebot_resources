#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

def tof_callback(msg):

    distance = msg.range

    if distance < msg.min_range or distance > msg.max_range:
        print("Distance: OUT OF RANGE")
    else:
        print("Distance: %.1f cm" % (distance * 100))

rospy.init_node("tof_display")

rospy.Subscriber(
    "/duck4/front_center_tof_driver_node/range",
    Range,
    tof_callback
)

rospy.spin()
