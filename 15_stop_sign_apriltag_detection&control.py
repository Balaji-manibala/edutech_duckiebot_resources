#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import apriltag
import threading
import time

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped


# ============================================================
# SETTINGS
# ============================================================

CAMERA_TOPIC = "/duck4/camera_node/image/compressed"

TOF_TOPIC = "/duck4/front_center_tof_driver_node/range"

WHEELS_TOPIC = "/duck4/wheels_driver_node/wheels_cmd"


# AprilTag
TAG_FAMILY = "tag36h11"
STOP_SIGN_ID = 27


# Robot speed
FORWARD_SPEED = 0.10


# Stop sign distance
STOP_DISTANCE = 0.40


# Stop duration
WAIT_TIME = 5.0


# Valid ToF range
MIN_TOF_RANGE = 0.05
MAX_TOF_RANGE = 1.20


# After GO, wait until sign disappears
SIGN_CLEAR_TIME = 1.0


# ============================================================
# GLOBAL VARIABLES
# ============================================================

latest_image = None

tof_distance = None

tag_detected = False

state = "DRIVING"

stop_start_time = None

sign_clear_start_time = None

running = True

lock = threading.Lock()

motor_pub = None


# ============================================================
# APRILTAG DETECTOR
# ============================================================

options = apriltag.DetectorOptions(
    families=TAG_FAMILY
)

detector = apriltag.Detector(options)


# ============================================================
# MOTOR FUNCTIONS
# ============================================================

def move_forward():

    msg = WheelsCmdStamped()

    msg.vel_left = float(FORWARD_SPEED)
    msg.vel_right = float(FORWARD_SPEED)

    motor_pub.publish(msg)


def stop_robot():

    msg = WheelsCmdStamped()

    msg.vel_left = 0.0
    msg.vel_right = 0.0

    motor_pub.publish(msg)


# ============================================================
# TOF CALLBACK
# ============================================================

def tof_callback(msg):

    global tof_distance

    value = msg.range

    with lock:

        if np.isnan(value) or np.isinf(value):

            tof_distance = None

        elif MIN_TOF_RANGE <= value <= MAX_TOF_RANGE:

            tof_distance = float(value)

        else:

            tof_distance = None


# ============================================================
# CAMERA CALLBACK
# ============================================================

def camera_callback(msg):

    global latest_image
    global tag_detected

    image = cv2.imdecode(
        np.frombuffer(msg.data, np.uint8),
        cv2.IMREAD_COLOR
    )

    if image is None:
        return

    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY
    )

    detections = detector.detect(gray)

    target_found = False

    # --------------------------------------------------------
    # PROCESS APRILTAG DETECTIONS
    # --------------------------------------------------------

    for detection in detections:

        tag_id = detection.tag_id

        corners = detection.corners.astype(int)

        # Draw tag outline

        for i in range(4):

            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])

            cv2.line(
                image,
                p1,
                p2,
                (0, 255, 0),
                3
            )

        # Tag center

        center = detection.center.astype(int)

        cx = int(center[0])
        cy = int(center[1])

        cv2.circle(
            image,
            (cx, cy),
            5,
            (0, 0, 255),
            -1
        )

        # ----------------------------------------------------
        # TARGET ID 27
        # ----------------------------------------------------

        if tag_id == STOP_SIGN_ID:

            target_found = True

            cv2.putText(
                image,
                "APRILTAG ID: 27",
                (cx - 100, cy - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                image,
                "STOP SIGN",
                (cx - 80, cy + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        else:

            cv2.putText(
                image,
                "ID: %d" % tag_id,
                (cx - 40, cy - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

    # --------------------------------------------------------
    # UPDATE GLOBAL CAMERA STATE
    # --------------------------------------------------------

    with lock:

        tag_detected = target_found

        latest_image = image


# ============================================================
# CONTROL STATE MACHINE
# ============================================================

def control_loop():

    global state
    global stop_start_time
    global sign_clear_start_time
    global running

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and running:

        with lock:

            tag = tag_detected
            distance = tof_distance
            current_state = state

        # ====================================================
        # DRIVING
        # ====================================================

        if current_state == "DRIVING":

            move_forward()

            # ------------------------------------------------
            # ID 27 detected
            # ------------------------------------------------

            if tag:

                if distance is not None:

                    if distance <= STOP_DISTANCE:

                        stop_robot()

                        with lock:

                            state = "WAITING"

                        stop_start_time = time.time()

                        sign_clear_start_time = None

                        rospy.logwarn(
                            "STOP SIGN ID 27 + ToF %.2f m -> STOP"
                            % distance
                        )

        # ====================================================
        # WAITING
        # ====================================================

        elif current_state == "WAITING":

            stop_robot()

            if stop_start_time is not None:

                elapsed = (
                    time.time()
                    - stop_start_time
                )

                if elapsed >= WAIT_TIME:

                    with lock:

                        state = "GO"

                    sign_clear_start_time = None

                    rospy.loginfo(
                        "5 SECOND STOP COMPLETE -> GO"
                    )

        # ====================================================
        # GO
        # ====================================================

        elif current_state == "GO":

            move_forward()

            # ------------------------------------------------
            # Wait for sign to disappear
            # ------------------------------------------------

            if not tag:

                if sign_clear_start_time is None:

                    sign_clear_start_time = time.time()

                else:

                    clear_time = (
                        time.time()
                        - sign_clear_start_time
                    )

                    if clear_time >= SIGN_CLEAR_TIME:

                        with lock:

                            state = "DRIVING"

                        sign_clear_start_time = None

                        rospy.loginfo(
                            "STOP SIGN CLEARED -> DRIVING"
                        )

            else:

                sign_clear_start_time = None

        rate.sleep()


# ============================================================
# DISPLAY LOOP
# ============================================================

def display_loop():

    global running
    global state
    global stop_start_time

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and running:

        # ----------------------------------------------------
        # COPY GLOBAL DATA
        # ----------------------------------------------------

        with lock:

            image = latest_image

            tag = tag_detected

            distance = tof_distance

            current_state = state

        # ----------------------------------------------------
        # CAMERA RECEIVED?
        # ----------------------------------------------------

        if image is not None:

            frame = image.copy()

            # ------------------------------------------------
            # TOF STATUS
            # ------------------------------------------------

            if distance is None:

                tof_text = "ToF: OUT OF RANGE"

            else:

                tof_text = "ToF: %.2f m" % distance

            cv2.putText(
                frame,
                tof_text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            # ------------------------------------------------
            # APRILTAG STATUS
            # ------------------------------------------------

            if tag:

                tag_text = "ID 27: DETECTED"

                tag_color = (0, 255, 0)

            else:

                tag_text = "ID 27: NOT DETECTED"

                tag_color = (255, 255, 0)

            cv2.putText(
                frame,
                tag_text,
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                tag_color,
                2
            )

            # ------------------------------------------------
            # STATE STATUS
            # ------------------------------------------------

            if current_state == "DRIVING":

                state_color = (0, 255, 0)

            elif current_state == "WAITING":

                state_color = (0, 0, 255)

            else:

                state_color = (0, 255, 255)

            cv2.putText(
                frame,
                "STATE: " + current_state,
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                state_color,
                2
            )

            # ------------------------------------------------
            # WAIT TIMER
            # ------------------------------------------------

            if current_state == "WAITING":

                if stop_start_time is not None:

                    elapsed = (
                        time.time()
                        - stop_start_time
                    )

                    remaining = max(
                        0.0,
                        WAIT_TIME - elapsed
                    )

                    cv2.putText(
                        frame,
                        "WAIT: %.1f sec" % remaining,
                        (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2
                    )

            # ------------------------------------------------
            # SPEED DISPLAY
            # ------------------------------------------------

            if current_state == "WAITING":

                display_speed = 0.0

            else:

                display_speed = FORWARD_SPEED

            cv2.putText(
                frame,
                "Speed: %.2f m/s" % display_speed,
                (20, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            # ------------------------------------------------
            # STOP DISTANCE
            # ------------------------------------------------

            cv2.putText(
                frame,
                "Stop distance: %.2f m"
                % STOP_DISTANCE,
                (20, 225),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # ------------------------------------------------
            # DISPLAY
            # ------------------------------------------------

            cv2.imshow(
                "Stop Sign ADAS - AprilTag + ToF",
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            # ------------------------------------------------
            # MANUAL STOP
            # ------------------------------------------------

            if key == ord('x'):

                stop_robot()

                with lock:

                    state = "WAITING"

                stop_start_time = time.time()

                rospy.logwarn(
                    "MANUAL STOP"
                )

            # ------------------------------------------------
            # QUIT
            # ------------------------------------------------

            elif key == ord('q') or key == 27:

                stop_robot()

                running = False

                rospy.signal_shutdown(
                    "User quit"
                )

                break

        rate.sleep()


# ============================================================
# MAIN
# ============================================================

rospy.init_node(
    "stop_sign_adas_tof"
)


# ============================================================
# MOTOR PUBLISHER
# ============================================================

motor_pub = rospy.Publisher(
    WHEELS_TOPIC,
    WheelsCmdStamped,
    queue_size=1
)


# ============================================================
# CAMERA SUBSCRIBER
# ============================================================

rospy.Subscriber(
    CAMERA_TOPIC,
    CompressedImage,
    camera_callback,
    queue_size=1,
    buff_size=2**24
)


# ============================================================
# TOF SUBSCRIBER
# ============================================================

rospy.Subscriber(
    TOF_TOPIC,
    Range,
    tof_callback,
    queue_size=1
)


# ============================================================
# STARTUP
# ============================================================

print("")
print("==============================================")
print("       STOP SIGN ADAS - 8.6.3")
print("==============================================")
print("")
print("AprilTag family :", TAG_FAMILY)
print("Stop Sign ID    :", STOP_SIGN_ID)
print("")
print("Forward speed   :", FORWARD_SPEED, "m/s")
print("Stop distance   :", STOP_DISTANCE, "m")
print("Wait time       :", WAIT_TIME, "seconds")
print("")
print("Camera:")
print(CAMERA_TOPIC)
print("")
print("ToF:")
print(TOF_TOPIC)
print("")
print("Motor:")
print(WHEELS_TOPIC)
print("")
print("X      = STOP")
print("Q/ESC  = QUIT")
print("Ctrl+C = STOP + QUIT")
print("")
print("==============================================")
print("Starting Stop Sign ADAS...")
print("==============================================")


# ============================================================
# START CONTROL THREAD
# ============================================================

control_thread = threading.Thread(
    target=control_loop
)

control_thread.daemon = True

control_thread.start()


# ============================================================
# WAIT FOR CAMERA / TOF
# ============================================================

rospy.sleep(1.0)


# ============================================================
# START MOVING
# ============================================================

with lock:

    state = "DRIVING"

move_forward()


# ============================================================
# DISPLAY LOOP
# ============================================================

try:

    display_loop()

except KeyboardInterrupt:

    print("")
    print("Ctrl+C pressed!")


# ============================================================
# SHUTDOWN
# ============================================================

finally:

    running = False

    stop_robot()

    rospy.sleep(0.1)

    stop_robot()

    cv2.destroyAllWindows()

    print("Robot stopped.")

