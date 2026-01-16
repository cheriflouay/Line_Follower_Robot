#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollowerNode(Node):
    """
    Line follower node that processes camera images and controls the robot
    to follow a black line on a white background.
    """

    def __init__(self):
        super().__init__('line_follower_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/line_camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp', 0.005)  # Proportional gain
        self.declare_parameter('kd', 0.001)  # Derivative gain
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        
        # Error tracking for derivative control
        self.last_error = 0.0
        
        # Image processing parameters
        self.image_width = 640
        self.image_height = 480
        self.roi_top = int(self.image_height * 0.6)  # Start ROI at 60% of image height
        
        self.get_logger().info('Line Follower Node has been started')
        self.get_logger().info(f'Parameters: linear_speed={self.linear_speed}, kp={self.kp}, kd={self.kd}')

    def image_callback(self, msg):
        """
        Callback function for processing camera images.
        Detects the line and computes control commands.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Extract region of interest (bottom portion of image)
            roi = cv_image[self.roi_top:self.image_height, 0:self.image_width]
            
            # Convert to grayscale
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Threshold the image to isolate black line
            # Black line will have low pixel values
            _, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
            
            # Find contours of the black line
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Initialize twist message
            twist = Twist()
            
            if len(contours) > 0:
                # Find the largest contour (assumed to be the line)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Calculate moments to find centroid
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                    # Calculate centroid x-coordinate
                    cx = int(M['m10'] / M['m00'])
                    
                    # Calculate error (deviation from center)
                    # Negative error means line is to the left, positive means right
                    image_center = self.image_width / 2
                    error = cx - image_center
                    
                    # Calculate derivative of error
                    error_derivative = error - self.last_error
                    self.last_error = error
                    
                    # PD controller: compute angular velocity
                    angular_z = -self.kp * error - self.kd * error_derivative
                    
                    # Set velocities
                    twist.linear.x = self.linear_speed
                    twist.angular.z = angular_z
                    
                    # Log information periodically
                    if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # Log every ~1 second
                        self.get_logger().info(f'Error: {error:.2f}, Angular velocity: {angular_z:.3f}')
                else:
                    # Line detected but no valid centroid
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().warn('Line detected but no valid centroid')
            else:
                # No line detected - stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.last_error = 0.0
                self.get_logger().warn('No line detected')
            
            # Publish velocity command
            self.publisher.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    line_follower_node = LineFollowerNode()
    
    try:
        rclpy.spin(line_follower_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        line_follower_node.publisher.publish(twist)
        
        line_follower_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()