import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

def detect_peg(image, lower_peg, upper_peg):
    height, width, _ = image.shape
    roi = image[int(height / 3):int(2 * height / 3), int(width / 4):int(3 * width / 4)]
    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, lower_peg, upper_peg)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    detected = False
    closest_contour = None
    min_distance_to_bottom = height

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) >= 4:
            x, y, w, h = cv2.boundingRect(approx)
            distance_to_bottom = height - (y + h)
            if h > w and h > 20 and distance_to_bottom < min_distance_to_bottom:
                closest_contour = (x, y, w, h)
                min_distance_to_bottom = distance_to_bottom

    peg_position = None
    if closest_contour is not None:
        x, y, w, h = closest_contour
        cv2.rectangle(image, (x + int(width / 4), y + int(height / 3)),
                      (x + w + int(width / 4), y + h + int(height / 3)), (0, 255, 0), 2)
        cv2.putText(image, 'Peg detected', (x + int(width / 4), y + int(height / 3) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
        detected = True
        if x + w / 2 > roi.shape[1] / 2:
            peg_position = 'right'
        else:
            peg_position = 'left'

    return detected, min_distance_to_bottom, image, peg_position

class FarmNavigator(Node):
    def __init__(self):
        super().__init__('farm_navigator')
        self.publisher_ = self.create_publisher(Twist, '/robot_base_controller/cmd_vel_unstamped', 10)
        self.subscriber_center_camera = self.create_subscription(Image, '/zed2_center_camera/image_raw', self.center_camera_callback, 1)
        self.bridge = CvBridge()
        self.obstacle_detected = False
        self.last_image_time = None
        self.camera_data_received = False
        self.peg_position = None
        self.last_obstacle_state = None
        self.deviation_angle = 0  # Track the deviation angle from the original path

        self.lower_peg = np.array([0, 117, 99])
        self.upper_peg = np.array([18, 225, 255])
        self.peg_detection_threshold = 340

        self.get_logger().info("FarmNavigator node has been started.")

    def run(self):
        while not self.camera_data_received:
            self.get_logger().warn("Waiting for camera data...")
            rclpy.spin_once(self, timeout_sec=1.0)

        while rclpy.ok():
            self.straight_movement()

            while self.obstacle_detected:
                self.stop_movement()
                if self.last_obstacle_state != 'detected':
                    self.get_logger().info("Peg detected, choosing turn direction to avoid it.")
                    self.get_logger().info("Peg position detected: " + str(self.peg_position))
                    self.last_obstacle_state = 'detected'
                
                if self.should_turn_left():
                    self.get_logger().info("Turning left to avoid peg")
                    self.turn(90, 2)
                    self.deviation_angle += 90
                else:
                    self.get_logger().info("Turning right to avoid peg")
                    self.turn(-90, 2)
                    self.deviation_angle -= 90

                self.check_if_obstacle_is_cleared()
                rclpy.spin_once(self, timeout_sec=0.1)

                if not self.obstacle_detected:
                    self.get_logger().info("Moving straight after avoidance")
                    self.straight_movement2()  # Move straight for a few seconds after avoidance
                    self.return_to_original_path()

        self.stop_movement()

    def straight_movement(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.7
        move_cmd.angular.z = 0.0

        start_time = time.time()
        while time.time() - start_time < 1:
            self.publisher_.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def straight_movement2(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.8
        move_cmd.angular.z = 0.0

        start_time = time.time()
        while time.time() - start_time < 6:
            self.publisher_.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop_movement(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        duration = 1  # duration of the stop in seconds
        self.get_logger().info("Stopping movement for " + str(duration) + " seconds.")

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

    def turn(self, angle_degrees, duration):
        self.get_logger().info("Executing turn: " + str(angle_degrees) + " degrees for " + str(duration) + " seconds")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = math.radians(angle_degrees) / duration

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop_movement()

    def should_turn_left(self):
        return self.peg_position == 'right'

    def check_if_obstacle_is_cleared(self):
        self.obstacle_detected = False
        self.last_obstacle_state = 'cleared'
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.publisher_.publish(move_cmd)
        rclpy.spin_once(self, timeout_sec=1.0)

        # Check if the peg is still detected after turning
        if self.obstacle_detected:
            self.get_logger().info("Peg still detected, continuing to avoid it.")
        else:
            self.get_logger().info("Peg avoided successfully.")

    def return_to_original_path(self):
        while self.deviation_angle != 0:
            if self.deviation_angle > 0:
                self.get_logger().info("Returning to original path by turning right")
                self.turn(-90, 0.5)
                self.deviation_angle -= 90
            elif self.deviation_angle < 0:
                self.get_logger().info("Returning to original path by turning left")
                self.turn(90, 0.5)
                self.deviation_angle += 90

    def center_camera_callback(self, msg):
        self.process_camera_image(msg, 'center')

    def process_camera_image(self, msg, camera_position):
        self.get_logger().info(f"Received {camera_position} camera data.")
        self.last_image_time = self.get_clock().now()
        self.camera_data_received = True
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if camera_position == 'center':
            detected, distance_to_bottom, cv_image_with_detection, peg_position = detect_peg(cv_image, self.lower_peg, self.upper_peg)
            self.obstacle_detected = detected and distance_to_bottom < self.peg_detection_threshold
            self.peg_position = peg_position

            if self.obstacle_detected:
                self.get_logger().info("Peg detected by center camera at distance " + str(distance_to_bottom) + " on " + str(peg_position) + " side")
            elif self.last_obstacle_state != 'cleared':
                self.get_logger().info("No peg detected by center camera")
                self.last_obstacle_state = 'cleared'

        cv2.imshow(str(camera_position.capitalize()) + " Camera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    farm_navigator = FarmNavigator()
    farm_navigator.run()
    farm_navigator.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
