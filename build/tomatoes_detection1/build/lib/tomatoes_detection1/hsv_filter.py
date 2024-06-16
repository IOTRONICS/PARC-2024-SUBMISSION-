#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ColourFilter(Node):

    def __init__(self):
        super().__init__('hsv_colour_filter')
        self.subscription = self.create_subscription(
            Image,
            '/zed2_center_camera/image_raw',
            self.camera_callback,
            1)
        self.bridge_object = CvBridge()

        # Create TrackBar Sliders for HSV colour range
        cv2.namedWindow("HSV Filter")
        cv2.resizeWindow("HSV Filter", 600, 300)
        cv2.createTrackbar("hue_min", "HSV Filter", 0, 179, self.empty)
        cv2.createTrackbar("hue_max", "HSV Filter", 179, 179, self.empty)
        cv2.createTrackbar("sat_min", "HSV Filter", 0, 255, self.empty)
        cv2.createTrackbar("sat_max", "HSV Filter", 255, 255, self.empty)
        cv2.createTrackbar("val_min", "HSV Filter", 0, 255, self.empty)
        cv2.createTrackbar("val_max", "HSV Filter", 255, 255, self.empty)

    def empty(self, img):
        pass

    def camera_callback(self, data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hue_min = cv2.getTrackbarPos("hue_min", "HSV Filter")
        hue_max = cv2.getTrackbarPos("hue_max", "HSV Filter")
        sat_min = cv2.getTrackbarPos("sat_min", "HSV Filter")
        sat_max = cv2.getTrackbarPos("sat_max", "HSV Filter")
        val_min = cv2.getTrackbarPos("val_min", "HSV Filter")
        val_max = cv2.getTrackbarPos("val_max", "HSV Filter")

        lower_colour = np.array([hue_min, sat_min, val_min])
        upper_colour = np.array([hue_max, sat_max, val_max])

        # Threshold the HSV image to get only colours within the lower and upper boundary
        colour_mask = cv2.inRange(hsv, lower_colour, upper_colour)
        colour = cv2.bitwise_and(cv_image, cv_image, mask=colour_mask)

        cv2.imshow("Camera View", colour)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    colour_filter = ColourFilter()

    try:
        rclpy.spin(colour_filter)
    except KeyboardInterrupt:
        colour_filter.get_logger().info('Shutting down')

    cv2.destroyAllWindows()
    colour_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
