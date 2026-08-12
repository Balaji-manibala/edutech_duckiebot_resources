#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading

from sensor_msgs.msg import Range, CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


# ---------------- SETTINGS ----------------

FAST_SPEED = 0.25
MEDIUM_SPEED = 0.18
SLOW_SPEED = 0.10

MIN_RANGE = 0.05
MAX_RANGE = 1.20

STOP_DISTANCE = 0.30
SLOW_DISTANCE = 0.50
MEDIUM_DISTANCE = 0.80


# ---------------- VARIABLES ----------------

distance = None
latest_image = None
running = True

current_speed = 0.0

lock = threading.Lock()


# ---------------- MOTOR ----------------

pub = None


def move(speed):

    msg = WheelsCmdStamped()

    msg.vel_left = float(speed)
    msg.vel_right = float(speed)

    pub.publish(msg)


def stop():

    move(0.0)


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


# ---------------- ACC CONTROL ----------------

def acc_loop():

    global current_speed
    global running

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and running:

        with lock:

            d = distance

        if d is None:

            current_speed = 0.0
            stop()

        elif d <= STOP_DISTANCE:

            current_speed = 0.0
            stop()

        elif d <= SLOW_DISTANCE:

            current_speed = SLOW_SPEED
            move(current_speed)

        elif d <= MEDIUM_DISTANCE:

            current_speed = MEDIUM_SPEED
            move(current_speed)

        else:

            current_speed = FAST_SPEED
            move(current_speed)

        rate.sleep()


# ---------------- DISPLAY ----------------

def display_loop():

    global running

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and running:

        with lock:

            image = latest_image
            d = distance
            speed = current_speed

        if image is not None:

            frame = image.copy()

            if d is None:

                distance_text = "Distance: Out of Range"
                status = "STOPPED"

            else:

                distance_text = "Distance: %.2f m" % d

                if d <= STOP_DISTANCE:

                    status = "STOP"

                elif d <= SLOW_DISTANCE:

                    status = "SLOW"

                elif d <= MEDIUM_DISTANCE:

                    status = "MEDIUM"

                else:

                    status = "FAST"

            cv2.putText(
                frame,
                distance_text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "ACC Speed: %.2f m/s" % speed,
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "ACC Status: " + status,
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255) if status == "STOP"
                else (0, 255, 0),
                2
            )

            cv2.imshow(
                "Duckiebot ACC",
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

rospy.init_node("acc_tof")

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
print(" ToF-Based Adaptive Cruise Control")
print("--------------------------------")
print("FAST   :", FAST_SPEED, "m/s")
print("MEDIUM :", MEDIUM_SPEED, "m/s")
print("SLOW   :", SLOW_SPEED, "m/s")
print("STOP   :", STOP_DISTANCE, "m")
print("--------------------------------")
print("X = Stop")
print("Q / ESC = Quit")
print("--------------------------------")


control_thread = threading.Thread(
    target=acc_loop
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
