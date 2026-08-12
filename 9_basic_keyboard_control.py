#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
import select
from duckietown_msgs.msg import WheelsCmdStamped

rospy.init_node("keyboard_control", disable_signals=True)

pub = rospy.Publisher(
    "/duck4/wheels_driver_node/wheels_cmd",
    WheelsCmdStamped,
    queue_size=1
)

speed = 0.3


def move(left, right):
    msg = WheelsCmdStamped()
    msg.vel_left = left
    msg.vel_right = right
    pub.publish(msg)


old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

print("W=Forward  S=Reverse  A=Left  D=Right")
print("Release key = Stop   Q=Quit")

try:

    while not rospy.is_shutdown():

        if select.select([sys.stdin], [], [], 0.05)[0]:

            key = sys.stdin.read(1).lower()

            if key == 'w':
                move(speed, speed)

            elif key == 's':
                move(-speed, -speed)

            elif key == 'a':
                move(-speed, speed)

            elif key == 'd':
                move(speed, -speed)

            elif key == 'q':
                break

        else:
            move(0.0, 0.0)

except KeyboardInterrupt:
    print("\nCtrl+C pressed!")

finally:

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    for i in range(10):
        move(0.0, 0.0)
        rospy.sleep(0.01)

    print("Robot stopped.")
