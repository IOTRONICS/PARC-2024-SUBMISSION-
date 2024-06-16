#!/usr/bin/env python3
"""
Script to detect tomatoes
"""
import rclpy
from rclpy.node import Node
from parc_robot_interfaces.msg import CropYield
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Import Image message type
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time
import os
import yaml


from ament_index_python.packages import get_package_share_directory


class TomatoDetector(Node):
    def __init__(self):
        super().__init__('tomato_detector')
        self.publisher_yield = self.create_publisher(CropYield, '/parc_robot/crop_yield', 1)
        self.bridge = CvBridge()
        
         # Get the path to the package share directory
        package_share_directory = get_package_share_directory('tomatoes_detection1')
        
        # Construct the path to the YAML configuration file
        config_path = os.path.join(package_share_directory, 'config/tomato.yaml')
        
        # Ensure the config path is correct
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        # Load the configuration
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Adjust paths in the configuration
        for key in ['train', 'val', 'test']:
            config[key] = os.path.join(package_share_directory, config[key])
            if not os.path.exists(config[key]):
                raise FileNotFoundError(f"{key.capitalize()} path not found: {config[key]}")
        
        # Use the paths and other parameters from the config
        self.train_path = config['train']
        self.val_path = config['val']
        self.test_path = config['test']
        self.nc = config['nc']
        self.names = config['names']
        self.roboflow = config['roboflow']
        
        # Load YOLO model with the correct path
        model_path = os.path.join(package_share_directory, 'runs/detect/train3/weights/best.pt')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        self.model = YOLO(model_path)

        # Subscribe to the right and left camera topics
        self.subscriber_right_camera = self.create_subscription(
            Image, '/right_camera/image_raw', self.right_camera_callback, 1)
        self.subscriber_left_camera = self.create_subscription(
            Image, '/left_camera/image_raw', self.left_camera_callback, 1)
        
        # Subscribe to the robot's movement status topic
        self.subscriber_robot_status = self.create_subscription(
            String, '/parc_robot/robot_status', self.robot_status_callback, 1)

        self.right_camera_data = None
        self.left_camera_data = None
        self.robot_status = "moving"
        self.last_movement_time = time.time()
        self.detected_tomatoes = []  # list to store detected tomatoes

        self.create_timer(1.0, self.detect_and_publish)

    def right_camera_callback(self, msg):
        self.get_logger().info("Received right camera image.")
        self.right_camera_data = msg

    def left_camera_callback(self, msg):
        self.get_logger().info("Received left camera image.")
        self.left_camera_data = msg

    def robot_status_callback(self, msg):
        self.get_logger().info("Received robot status: " + str(msg.data))
        # Update the robot's movement status based on the received message
        if msg.data == "stopped":
            if self.robot_status != "stopped":
                self.robot_status = "stopped"
                self.last_movement_time = time.time()
                self.get_logger().info("Robot status changed to stopped.")
        elif msg.data == "moving":
            self.robot_status = "moving"
            self.last_movement_time = time.time()  # reset the timer when the robot starts moving again
            self.get_logger().info("Robot status changed to moving.")
        elif msg.data == "finished":
            self.get_logger().info("Robot finished.")
            self.publish_yield(len(self.detected_tomatoes))
            self.get_logger().info("Final crop yield published: " + str(len(self.detected_tomatoes)) + " tomatoes")
            rclpy.shutdown()

    def detect_and_publish(self):
        # Check if both camera data is available
        if self.right_camera_data is None or self.left_camera_data is None:
            self.get_logger().warn("Camera data not available.")
            return

        self.get_logger().info("Processing camera data for detection.")

        # Convert camera data to OpenCV format
        right_image = self.bridge.imgmsg_to_cv2(self.right_camera_data, desired_encoding="bgr8")
        left_image = self.bridge.imgmsg_to_cv2(self.left_camera_data, desired_encoding="bgr8")

        # Perform tomato detection using YOLOv8 model
        right_results = self.model(right_image)[0]
        left_results = self.model(left_image)[0]

        # Process detection results
        right_new_detections = self.filter_and_count(right_results, right_image)
        left_new_detections = self.filter_and_count(left_results, left_image)

        self.detected_tomatoes.extend(right_new_detections)
        self.detected_tomatoes.extend(left_new_detections)

        self.get_logger().info("Current detected tomatoes: " + str(len(self.detected_tomatoes)))
        
        # Visualize the results
        try:
            right_overlay = right_results.plot() if hasattr(right_results, 'plot') else right_image
            left_overlay = left_results.plot() if hasattr(left_results, 'plot') else left_image
        
            cv2.imshow('Right Camera', right_overlay)
            cv2.imshow('Left Camera', left_overlay)
            cv2.waitKey(1)  # A short delay to allow the image windows to update
        except cv2.error as e:
            self.get_logger().warn(f"OpenCV error: {e}. Saving images to disk instead.")
            cv2.imwrite('/tmp/right_camera.jpg', right_overlay)
            cv2.imwrite('/tmp/left_camera.jpg', left_overlay)

        # Check if the robot has been stopped for 3 or more seconds
        if self.robot_status == "stopped" and (time.time() - self.last_movement_time >= 3):
            total_count = len(self.detected_tomatoes)
            self.publish_yield(total_count)
            self.detected_tomatoes = []
            self.get_logger().info("Robot stopped for >= 3 seconds. Total count: " + str(total_count))
            print(f"Robot stopped for >= 3 seconds. Total count: " + str(total_count))

    def filter_and_count(self, results, image):
        new_detections = []

        # Ensure results have the 'boxes' attribute
        if hasattr(results, 'boxes'):
            boxes = results.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                # Check if this detection is far enough from previous detections
                if all(np.linalg.norm(np.array([center_x, center_y]) - np.array(prev)) > 50 for prev in self.detected_tomatoes):
                    new_detections.append((center_x, center_y))
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            self.get_logger().info("Detected " + str(len(new_detections)) + " new tomatoes.")
        else:
            self.get_logger().warn("Results do not have the 'boxes' attribute. Results: " + str(results))

        return new_detections

    def publish_yield(self, count):
        msg_yield = CropYield()
        msg_yield.data = count
        self.publisher_yield.publish(msg_yield)
        self.get_logger().info("Published crop yield: " + str(count) + " tomatoes")
        print(f"Published crop yield: {count} tomatoes")


def main(args=None):
    rclpy.init(args=args)

    tomato_detector = TomatoDetector()
    rclpy.spin(tomato_detector)
    tomato_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
