#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading
import sys

from sensor_msgs.msg import Range, CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


# ---------------- SETTINGS ----------------

MAX_SPEED = 0.25
TURN_SPEED = 0.10

MIN_RANGE = 0.05
MAX_RANGE = 1.20

AEB_DISTANCE = 0.30
RELEASE_DISTANCE = 0.40


# ---------------- VARIABLES ----------------

distance = None
latest_image = None

aeb_active = False

forward = False
reverse = False
left = False
right = False

running = True

lock = threading.Lock()


# ---------------- MOTOR ----------------

pub = None


def move(left_speed, right_speed):

    msg = WheelsCmdStamped()

    msg.vel_left = float(left_speed)
    msg.vel_right = float(right_speed)

    pub.publish(msg)


def stop_robot():

    move(0.0, 0.0)


# ---------------- TOF ----------------

def tof_callback(msg):

    global distance
    global aeb_active

    with lock:

        if MIN_RANGE <= msg.range <= MAX_RANGE:

            distance = msg.range

            if distance <= AEB_DISTANCE:
                aeb_active = True

            elif distance >= RELEASE_DISTANCE:
                aeb_active = False

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


# ---------------- CONTROL ----------------

def control_loop():

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and running:

        with lock:

            brake = aeb_active

            f = forward
            r = reverse
            l = left
            d = right

        # AEB has highest priority

        if brake:

            move(0.0, 0.0)

        # Forward + Left

        elif f and l:

            move(MAX_SPEED - TURN_SPEED, MAX_SPEED)

        # Forward + Right

        elif f and d:

            move(MAX_SPEED, MAX_SPEED - TURN_SPEED)

        # Reverse + Left

        elif r and l:

            move(-(MAX_SPEED - TURN_SPEED), -MAX_SPEED)

        # Reverse + Right

        elif r and d:

            move(-MAX_SPEED, -(MAX_SPEED - TURN_SPEED))

        # Forward

        elif f:

            move(MAX_SPEED, MAX_SPEED)

        # Reverse

        elif r:

            move(-MAX_SPEED, -MAX_SPEED)

        # Nothing pressed

        else:

            move(0.0, 0.0)

        rate.sleep()


# ---------------- DISPLAY ----------------

def display_loop():

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and running:

        with lock:

            image = latest_image

            d = distance

            brake = aeb_active

        if image is not None:

            frame = image.copy()

            # Distance

            if d is None:

                text = "Distance: Out of Range"

            else:

                text = "Distance: %.2f m" % d

            cv2.putText(
                frame,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            # AEB status

            if brake:

                cv2.putText(
                    frame,
                    "AEB ACTIVE - BRAKING!",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            else:

                cv2.putText(
                    frame,
                    "AEB: READY",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            cv2.imshow(
                "Duckiebot Manual Driving + AEB",
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            keyboard(key)

        rate.sleep()


# ---------------- KEYBOARD ----------------

def keyboard(key):

    global forward
    global reverse
    global left
    global right
    global running

    # Q or ESC

    if key == ord('q') or key == 27:

        running = False
        stop_robot()
        rospy.signal_shutdown("User quit")
        return

    # X = stop

    if key == ord('x'):

        forward = False
        reverse = False
        left = False
        right = False

        stop_robot()

        return

    # W

    if key == ord('w'):

        forward = True
        reverse = False

    # S

    if key == ord('s'):

        reverse = True
        forward = False

    # A

    if key == ord('a'):

        left = True

    # D

    if key == ord('d'):

        right = True


# ---------------- MAIN ----------------

rospy.init_node("manual_aeb")

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
print(" Duckiebot Manual Driving + AEB")
print("--------------------------------")
print("W = Forward")
print("S = Reverse")
print("A = Left")
print("D = Right")
print("X = Stop")
print("Q / ESC = Quit")
print("--------------------------------")
print("AEB distance:", AEB_DISTANCE, "m")
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

    stop_robot()

    rospy.sleep(0.1)

    stop_robot()

    cv2.destroyAllWindows()

    print("Robot stopped.")
