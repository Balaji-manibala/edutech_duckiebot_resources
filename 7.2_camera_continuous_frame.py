#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()

def image_callback(msg):
    image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Duckiebot Camera", image)
    cv2.waitKey(1)

rospy.init_node("camera_view")

rospy.Subscriber(
    "/duck4/camera_node/image/compressed",
    CompressedImage,
    image_callback
)

try:
    rospy.spin()

except KeyboardInterrupt:
    print("\nCamera stopped.")

finally:
    cv2.destroyAllWindows()
