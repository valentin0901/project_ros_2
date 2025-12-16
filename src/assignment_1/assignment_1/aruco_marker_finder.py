#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoMarkerFinder(Node):
    def __init__(self):
        super().__init__('aruco_marker_finder')
        
        # Parameters
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('image_center_tolerance', 50)
        
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.tolerance = self.get_parameter('image_center_tolerance').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marked_image_pub = self.create_publisher(Image, '/aruco_marked_image', 10)
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        # State variables
        self.detected_ids = set()
        self.all_ids_found = False
        self.target_marker = None
        self.current_image = None
        self.target_position = None
        self.image_width = None
        self.state = 'SCANNING'  # States: SCANNING, APPROACHING, CENTERING, DONE
        
        self.bridge = CvBridge()
        
        self.get_logger().info('Aruco Marker Finder Node Started')
        self.get_logger().info('State: SCANNING - Rotating to find all markers...')
    
    def image_callback(self, msg):
        """Store the current image"""
        self.current_image = msg
        if self.image_width is None:
            self.image_width = msg.width
    
    def aruco_callback(self, msg):
        """Process Aruco detections"""
        # Add detected IDs to our set
        for marker in msg.markers:
            marker_id = marker.marker_id
            if marker_id not in self.detected_ids:
                self.get_logger().info(f'New marker detected: ID {marker_id}')
                self.detected_ids.add(marker_id)
        
        # Check if we found all 5 markers
        if len(self.detected_ids) >= 5 and not self.all_ids_found:
            self.all_ids_found = True
            self.target_marker = min(self.detected_ids)
            self.stop_robot()
            self.get_logger().info(f'All 5 markers found! IDs: {sorted(self.detected_ids)}')
            self.get_logger().info(f'Target marker with lowest ID: {self.target_marker}')
            self.state = 'APPROACHING'
        
        # Process markers if we're looking for the target
        if self.all_ids_found and len(msg.markers) > 0:
            self.process_target_marker(msg)
    
    def process_target_marker(self, msg):
        """Process the target marker and move robot accordingly"""
        target_found = False
        
        for marker in msg.markers:
            if marker.marker_id == self.target_marker:
                target_found = True
                # Get marker position in image (using pose)
                # The marker pose is in camera frame, we need the image coordinates
                # We'll use the center of the image and the marker's x position
                
                # Extract x position from marker pose (assuming it's in camera frame)
                marker_x = marker.pose.position.x
                marker_z = marker.pose.position.z
                
                # Store for centering logic
                self.target_position = marker_x
                
                if self.state == 'APPROACHING':
                    self.approach_marker(marker_x, marker_z)
                elif self.state == 'CENTERING':
                    self.center_marker(marker_x, marker_z)
                
                break
        
        if not target_found and self.state in ['APPROACHING', 'CENTERING']:
            # Lost sight of target, rotate to find it
            self.rotate_to_find_target()
    
    def approach_marker(self, marker_x, marker_z):
        """Move towards the marker"""
        if abs(marker_x) > 0.1:  # If marker is not centered
            # Rotate to center the marker
            twist = Twist()
            twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.5
            self.cmd_vel_pub.publish(twist)
        elif marker_z > 1.0:  # If marker is far (adjust threshold as needed)
            # Move forward
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist)
        else:
            # Close enough, switch to centering mode
            self.state = 'CENTERING'
            self.get_logger().info('Switched to CENTERING mode')
    
    def center_marker(self, marker_x, marker_z):
        """Fine-tune to center the marker in the image"""
        if abs(marker_x) < 0.05:  # Marker is centered
            self.stop_robot()
            self.state = 'DONE'
            self.get_logger().info('Marker centered! Publishing marked image...')
            self.publish_marked_image()
        else:
            # Fine rotation adjustment
            twist = Twist()
            twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.3
            self.cmd_vel_pub.publish(twist)
    
    def rotate_to_find_target(self):
        """Rotate to find the target marker"""
        twist = Twist()
        twist.angular.z = self.angular_speed * 0.5
        self.cmd_vel_pub.publish(twist)
    
    def publish_marked_image(self):
        """Publish the current image with a circle around the target marker"""
        if self.current_image is None:
            self.get_logger().warn('No image available to publish')
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')
            
            # Draw a circle in the center of the image
            height, width = cv_image.shape[:2]
            center = (width // 2, height // 2)
            radius = 100  # Adjust as needed
            
            # Draw circle
            cv2.circle(cv_image, center, radius, (0, 255, 0), 3)
            
            # Add text
            text = f'Marker ID: {self.target_marker}'
            cv2.putText(cv_image, text, (center[0] - 80, center[1] - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Convert back to ROS Image
            marked_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            marked_image_msg.header = self.current_image.header
            
            # Publish
            self.marked_image_pub.publish(marked_image_msg)
            self.get_logger().info(f'Published marked image on /aruco_marked_image')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def scan_for_markers(self):
        """Rotate to scan for markers"""
        if self.state == 'SCANNING':
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerFinder()
    
    try:
        # Create a timer for scanning
        timer = node.create_timer(0.1, node.scan_for_markers)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()