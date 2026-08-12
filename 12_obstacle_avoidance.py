#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading

from sensor_msgs.msg import Range, CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


# ---------------- SETTINGS ----------------

FORWARD_SPEED = 0.20
REVERSE_SPEED = 0.15
TURN_SPEED = 0.15

MIN_RANGE = 0.05
MAX_RANGE = 1.20

OBSTACLE_DISTANCE = 0.30

REVERSE_TIME = 0.5
TURN_TIME = 0.8


# ---------------- VARIABLES ----------------

distance = None
latest_image = None

running = True
avoiding = False

lock = threading.Lock()


# ---------------- MOTOR ----------------

pub = None


def move(left, right):

    msg = WheelsCmdStamped()

    msg.vel_left = float(left)
    msg.vel_right = float(right)

    pub.publish(msg)


def stop():

    move(0.0, 0.0)


# ---------------- TOF ----------------

def tof_callback(msg):

    global distance

    with lock:

        if MIN_RANGE <= msg.range <= MAX_RANGE:
            distance = msg.range
        else:
            distance = None


# ---------------- CAMERA ----------------

def camera_callback(msg):

    global latest_image

    image = cv2.imdecode(
        np.frombuffer(msg.data, np.uint8),
        cv2.IMREAD_COLOR
    )

    with lock:
        latest_image = image


# ---------------- AVOID OBSTACLE ----------------

def avoid_obstacle():

    global avoiding

    avoiding = True

    print("Obstacle detected!")
    print("Stopping...")

    stop()
    rospy.sleep(0.2)

    print("Reversing...")

    move(-REVERSE_SPEED, -REVERSE_SPEED)
    rospy.sleep(REVERSE_TIME)

    print("Turning right...")

    move(TURN_SPEED, -TURN_SPEED)
    rospy.sleep(TURN_TIME)

    print("Continuing forward...")

    stop()

    avoiding = False


# ---------------- CONTROL ----------------

def control_loop():

    global running

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and running:

        with lock:
            d = distance

        if not avoiding:

            if d is not None and d <= OBSTACLE_DISTANCE:

                avoid_obstacle()

            else:

                move(FORWARD_SPEED, FORWARD_SPEED)

        rate.sleep()


# ---------------- DISPLAY ----------------

def display_loop():

    global running

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and running:

        with lock:

            image = latest_image
            d = distance

        if image is not None:

            frame = image.copy()

            if d is None:

                text = "Distance: Out of Range"
                status = "SENSOR OUT OF RANGE"

            else:

                text = "Distance: %.2f m" % d

                if d <= OBSTACLE_DISTANCE:
                    status = "OBSTACLE DETECTED"
                else:
                    status = "PATH CLEAR"

            cv2.putText(
                frame,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                status,
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255) if d is not None and d <= OBSTACLE_DISTANCE
                else (0, 255, 0),
                2
            )

            if avoiding:

                cv2.putText(
                    frame,
                    "AVOIDING OBSTACLE",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            cv2.imshow(
                "Duckiebot Obstacle Avoidance",
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:

                running = False
                stop()
                rospy.signal_shutdown("User quit")

            elif key == ord('x'):

                running = False
                stop()
                rospy.signal_shutdown("Manual stop")

        rate.sleep()


# ---------------- MAIN ----------------

rospy.init_node("obstacle_avoidance")

pub = rospy.Publisher(
    "/duck4/wheels_driver_node/wheels_cmd",
    WheelsCmdStamped,
    queue_size=1
)

rospy.Subscriber(
    "/duck4/front_center_tof_driver_node/range",
    Range,
    tof_callback
)

rospy.Subscriber(
    "/duck4/camera_node/image/compressed",
    CompressedImage,
    camera_callback
)


print("--------------------------------")
print(" Duckiebot Obstacle Avoidance")
print("--------------------------------")
print("Forward speed :", FORWARD_SPEED)
print("Obstacle      :", OBSTACLE_DISTANCE, "m")
print("Reverse time  :", REVERSE_TIME, "sec")
print("Turn time     :", TURN_TIME, "sec")
print("--------------------------------")
print("X = Stop")
print("Q / ESC = Quit")
print("--------------------------------")


control_thread = threading.Thread(
    target=control_loop
)

control_thread.daemon = True
control_thread.start()


try:

    display_loop()

except KeyboardInterrupt:

    print("\nCtrl+C pressed!")

finally:

    running = False

    stop()

    rospy.sleep(0.1)

    stop()

    cv2.destroyAllWindows()

    print("Robot stopped.")
