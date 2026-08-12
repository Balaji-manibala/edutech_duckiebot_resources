#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import signal
import sys

from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


# ============================================================
# 8.7.3 YELLOW LINE FOLLOWER
# ============================================================

# ------------------------------------------------------------
# ROS TOPICS
# ------------------------------------------------------------

CAMERA_TOPIC = "/duck4/camera_node/image/compressed"

WHEELS_TOPIC = "/duck4/wheels_driver_node/wheels_cmd"


# ------------------------------------------------------------
# ROBOT SPEED
# ------------------------------------------------------------

# Start with 0.20 m/s.
# After successful testing you can increase this to 0.25.
FORWARD_SPEED = 0.20


# ------------------------------------------------------------
# PROPORTIONAL CONTROLLER
# ------------------------------------------------------------

KP = 0.004

# Maximum steering correction
MAX_CORRECTION = 0.06


# ------------------------------------------------------------
# DEAD BAND
# ------------------------------------------------------------

DEADBAND = 10


# ------------------------------------------------------------
# YELLOW HSV RANGE
# ------------------------------------------------------------

LOWER_YELLOW = np.array([15, 80, 80])
UPPER_YELLOW = np.array([40, 255, 255])


# ------------------------------------------------------------
# MINIMUM YELLOW AREA
# ------------------------------------------------------------

MIN_CONTOUR_AREA = 100


# ------------------------------------------------------------
# CAMERA
# ------------------------------------------------------------

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


# ============================================================
# GLOBAL VARIABLES
# ============================================================

latest_frame = None

running = True

motor_pub = None


# ============================================================
# CAMERA CALLBACK
# ============================================================

def camera_callback(msg):

    global latest_frame

    frame = cv2.imdecode(
        np.frombuffer(msg.data, np.uint8),
        cv2.IMREAD_COLOR
    )

    if frame is not None:

        latest_frame = frame


# ============================================================
# MOTOR COMMAND
# ============================================================

def set_wheel_speed(left_speed, right_speed):

    msg = WheelsCmdStamped()

    msg.vel_left = float(left_speed)
    msg.vel_right = float(right_speed)

    motor_pub.publish(msg)


# ============================================================
# STOP ROBOT
# ============================================================

def stop_robot():

    if motor_pub is None:
        return

    msg = WheelsCmdStamped()

    msg.vel_left = 0.0
    msg.vel_right = 0.0

    # Publish multiple times so the stop command
    # reaches the robot reliably.

    for _ in range(5):

        motor_pub.publish(msg)

        rospy.sleep(0.02)


# ============================================================
# CTRL+C HANDLER
# ============================================================

def shutdown_handler(signum, frame):

    global running

    print("")
    print("Ctrl+C received!")
    print("Stopping robot...")

    running = False

    stop_robot()

    print("Robot stopped.")

    rospy.signal_shutdown(
        "User requested shutdown"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    global motor_pub
    global running

    # --------------------------------------------------------
    # ROS NODE
    # --------------------------------------------------------

    rospy.init_node(
        "yellow_lane_follower"
    )

    # --------------------------------------------------------
    # MOTOR PUBLISHER
    # --------------------------------------------------------

    motor_pub = rospy.Publisher(
        WHEELS_TOPIC,
        WheelsCmdStamped,
        queue_size=1
    )

    # --------------------------------------------------------
    # CAMERA SUBSCRIBER
    # --------------------------------------------------------

    rospy.Subscriber(
        CAMERA_TOPIC,
        CompressedImage,
        camera_callback,
        queue_size=1,
        buff_size=2**24
    )

    # --------------------------------------------------------
    # CTRL+C
    # --------------------------------------------------------

    signal.signal(
        signal.SIGINT,
        shutdown_handler
    )

    signal.signal(
        signal.SIGTERM,
        shutdown_handler
    )

    # --------------------------------------------------------
    # STARTUP INFORMATION
    # --------------------------------------------------------

    print("")
    print("==============================================")
    print("       8.7.3 YELLOW LINE FOLLOWER")
    print("==============================================")
    print("")
    print("Camera:")
    print(CAMERA_TOPIC)
    print("")
    print("Motor:")
    print(WHEELS_TOPIC)
    print("")
    print("----------------------------------------------")
    print("Forward Speed :", FORWARD_SPEED, "m/s")
    print("Kp            :", KP)
    print("Max Correction:", MAX_CORRECTION)
    print("Deadband      :", DEADBAND, "pixels")
    print("----------------------------------------------")
    print("")
    print("Q / ESC = Stop + Quit")
    print("Ctrl+C  = Stop + Quit")
    print("")
    print("Robot will start when camera data is received.")
    print("==============================================")
    print("")

    rate = rospy.Rate(20)

    # ========================================================
    # MAIN LOOP
    # ========================================================

    while not rospy.is_shutdown() and running:

        # ----------------------------------------------------
        # WAIT FOR CAMERA
        # ----------------------------------------------------

        if latest_frame is None:

            stop_robot()

            cv2.waitKey(1)

            rate.sleep()

            continue

        # ----------------------------------------------------
        # COPY CAMERA FRAME
        # ----------------------------------------------------

        frame = latest_frame.copy()

        height, width = frame.shape[:2]

        # ----------------------------------------------------
        # ROI
        # Bottom 1/3 of camera
        # ----------------------------------------------------

        roi_start_y = int(
            height * 2 / 3
        )

        roi = frame[
            roi_start_y:height,
            0:width
        ]

        # ----------------------------------------------------
        # HSV
        # ----------------------------------------------------

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # ----------------------------------------------------
        # YELLOW MASK
        # ----------------------------------------------------

        mask = cv2.inRange(
            hsv,
            LOWER_YELLOW,
            UPPER_YELLOW
        )

        # ----------------------------------------------------
        # MORPHOLOGICAL FILTERING
        # ----------------------------------------------------

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # ----------------------------------------------------
        # FIND CONTOURS
        # ----------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        yellow_detected = False

        yellow_center_x = None
        yellow_center_y = None

        # ----------------------------------------------------
        # FIND LARGEST YELLOW CONTOUR
        # ----------------------------------------------------

        if contours:

            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(
                largest_contour
            )

            if area > MIN_CONTOUR_AREA:

                M = cv2.moments(
                    largest_contour
                )

                if M["m00"] != 0:

                    yellow_center_x = int(
                        M["m10"] / M["m00"]
                    )

                    yellow_center_y = int(
                        M["m01"] / M["m00"]
                    )

                    yellow_detected = True

                    # Draw contour

                    cv2.drawContours(
                        roi,
                        [largest_contour],
                        -1,
                        (0, 255, 0),
                        2
                    )

                    # Draw yellow center

                    cv2.circle(
                        roi,
                        (
                            yellow_center_x,
                            yellow_center_y
                        ),
                        8,
                        (0, 0, 255),
                        -1
                    )

        # ----------------------------------------------------
        # CAMERA CENTER
        # ----------------------------------------------------

        image_center_x = width // 2

        cv2.line(
            roi,
            (
                image_center_x,
                0
            ),
            (
                image_center_x,
                roi.shape[0]
            ),
            (255, 0, 0),
            2
        )

        # ====================================================
        # YELLOW DETECTED
        # ====================================================

        if yellow_detected:

            # ------------------------------------------------
            # ERROR
            # ------------------------------------------------

            error = (
                yellow_center_x
                - image_center_x
            )

            # ------------------------------------------------
            # NORMALIZED ERROR
            # ------------------------------------------------

            normalized_error = (
                float(error)
                / float(image_center_x)
            )

            # ------------------------------------------------
            # PROPORTIONAL CONTROL
            # ------------------------------------------------

            if abs(error) <= DEADBAND:

                correction = 0.0

                direction = "CENTER"

            else:

                correction = KP * error

                direction = (
                    "LEFT"
                    if error < 0
                    else "RIGHT"
                )

            # ------------------------------------------------
            # LIMIT CORRECTION
            # ------------------------------------------------

            correction = max(
                -MAX_CORRECTION,
                min(
                    MAX_CORRECTION,
                    correction
                )
            )

            # =================================================
            # WHEEL SPEED CALCULATION
            # =================================================

            # Positive correction:
            #
            # Yellow line is RIGHT
            # -> steer RIGHT
            #
            # Left wheel becomes faster
            # Right wheel becomes slower

            left_speed = (
                FORWARD_SPEED
                + correction
            )

            right_speed = (
                FORWARD_SPEED
                - correction
            )

            # ------------------------------------------------
            # SAFETY LIMIT
            # ------------------------------------------------

            left_speed = max(
                0.0,
                left_speed
            )

            right_speed = max(
                0.0,
                right_speed
            )

            # ------------------------------------------------
            # SEND MOTOR COMMAND
            # ------------------------------------------------

            set_wheel_speed(
                left_speed,
                right_speed
            )

            # ------------------------------------------------
            # ERROR LINE
            # ------------------------------------------------

            cv2.line(
                roi,
                (
                    image_center_x,
                    yellow_center_y
                ),
                (
                    yellow_center_x,
                    yellow_center_y
                ),
                (0, 255, 255),
                3
            )

            # ------------------------------------------------
            # DISPLAY
            # ------------------------------------------------

            cv2.putText(
                frame,
                "YELLOW DETECTED",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "Center: %d px"
                % yellow_center_x,
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Error: %+d px"
                % error,
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Norm: %+0.3f"
                % normalized_error,
                (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Direction: " + direction,
                (20, 175),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Left: %.3f"
                % left_speed,
                (20, 210),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                "Right: %.3f"
                % right_speed,
                (20, 245),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

        # ====================================================
        # YELLOW NOT DETECTED
        # ====================================================

        else:

            # ------------------------------------------------
            # SAFETY:
            # STOP if yellow line is lost
            # ------------------------------------------------

            stop_robot()

            cv2.putText(
                frame,
                "YELLOW NOT DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                "ROBOT STOPPED",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                "Direction: SEARCH",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # ----------------------------------------------------
        # ROI LINE
        # ----------------------------------------------------

        cv2.line(
            frame,
            (
                0,
                roi_start_y
            ),
            (
                width,
                roi_start_y
            ),
            (255, 255, 255),
            2
        )

        # ----------------------------------------------------
        # DISPLAY
        # ----------------------------------------------------

        cv2.imshow(
            "Yellow Lane Follower",
            frame
        )

        cv2.imshow(
            "Yellow Mask",
            mask
        )

        # ----------------------------------------------------
        # KEYBOARD
        # ----------------------------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:

            print("")
            print("Quit requested.")

            running = False

            break

        rate.sleep()

    # ========================================================
    # FINAL STOP
    # ========================================================

    stop_robot()

    cv2.destroyAllWindows()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("")
        print("Keyboard interrupt!")

    except Exception as e:

        print("")
        print("ERROR:", e)

    finally:

        stop_robot()

        cv2.destroyAllWindows()

        print("Robot stopped.")
