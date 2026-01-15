#!/usr/bin/env python3
"""
Visit/Photograph Marker Action Node for PlanSys2
Approaches a marker, centers it in the camera, and takes a photo
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from plansys2_msgs.action import ExecuteAction
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os


class VisitMarkerActionNode(Node):
    """
    PlanSys2 Action Executor for photographing ArUco markers.
    Approaches the marker, centers it, and takes a photo.
    """
    
    def __init__(self):
        super().__init__('visit_marker_action_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('approach_distance', 1.0)
        self.declare_parameter('center_tolerance', 0.08)
        self.declare_parameter('save_images', True)
        self.declare_parameter('image_save_path', '/tmp/aruco_photos')
        
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        self.save_images = self.get_parameter('save_images').value
        self.image_save_path = self.get_parameter('image_save_path').value
        
        # Create image save directory
        if self.save_images:
            os.makedirs(self.image_save_path, exist_ok=True)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marked_image_pub = self.create_publisher(Image, '/aruco_marked_image', 10)
        self.current_marker_pub = self.create_publisher(Int32, '/current_target_marker', 10)
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # State
        self.target_marker_id = None
        self.current_marker_pose = None
        self.current_image = None
        self.marker_visible = False
        self.bridge = CvBridge()
        self.photographed_markers = set()
        
        # PlanSys2 action server for photograph_marker
        self._action_server = ActionServer(
            self,
            ExecuteAction,
            'photograph_marker',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # PlanSys2 action server for set_next_target
        self._next_target_server = ActionServer(
            self,
            ExecuteAction,
            'set_next_target',
            execute_callback=self.execute_set_next_target,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Visit Marker Action Node initialized')
    
    def goal_callback(self, goal_request):
        """Accept all goals"""
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept cancel requests"""
        self.stop_robot()
        return CancelResponse.ACCEPT
    
    def aruco_callback(self, msg):
        """Process ArUco detections"""
        self.marker_visible = False
        
        for marker in msg.markers:
            if marker.marker_id == self.target_marker_id:
                self.marker_visible = True
                self.current_marker_pose = marker.pose
                break
    
    def image_callback(self, msg):
        """Store current camera image"""
        self.current_image = msg
    
    async def execute_callback(self, goal_handle):
        """Execute the photograph_marker action"""
        self.get_logger().info('Executing photograph_marker action...')
        
        args = goal_handle.request.arguments
        parts = args.split()
        
        if len(parts) < 3:
            self.get_logger().error(f'Invalid arguments: {args}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            return result
        
        robot_name = parts[0]
        marker_name = parts[1]  # e.g., "marker_0"
        waypoint = parts[2]
        
        # Extract marker ID from name (e.g., "marker_11" -> 11)
        try:
            # Try to get the actual ID from the marker name
            marker_id_str = marker_name.split('_')[-1]
            self.target_marker_id = int(marker_id_str)
        except ValueError:
            self.get_logger().error(f'Cannot parse marker ID from: {marker_name}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            return result
        
        self.get_logger().info(f'Photographing marker {self.target_marker_id} at {waypoint}')
        
        # Publish current target
        target_msg = Int32()
        target_msg.data = self.target_marker_id
        self.current_marker_pub.publish(target_msg)
        
        # Phase 1: Find the marker by rotating
        if not await self.find_marker(timeout=15.0):
            self.get_logger().error(f'Could not find marker {self.target_marker_id}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.error_info = f'Marker {self.target_marker_id} not visible'
            return result
        
        # Phase 2: Approach the marker
        if not await self.approach_marker():
            self.get_logger().warn('Could not fully approach marker, continuing anyway')
        
        # Phase 3: Center the marker in the image
        if not await self.center_marker():
            self.get_logger().warn('Could not fully center marker, continuing anyway')
        
        # Phase 4: Take and publish the photo
        self.take_photo()
        
        self.photographed_markers.add(self.target_marker_id)
        self.get_logger().info(f'Successfully photographed marker {self.target_marker_id}')
        
        goal_handle.succeed()
        result = ExecuteAction.Result()
        result.success = True
        return result
    
    async def find_marker(self, timeout=15.0):
        """Rotate to find the target marker"""
        self.get_logger().info(f'Searching for marker {self.target_marker_id}...')
        
        start_time = time.time()
        twist = Twist()
        twist.angular.z = self.angular_speed
        
        while time.time() - start_time < timeout:
            if self.marker_visible:
                self.stop_robot()
                self.get_logger().info(f'Found marker {self.target_marker_id}')
                return True
            
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop_robot()
        return False
    
    async def approach_marker(self, timeout=20.0):
        """Approach the marker until at desired distance"""
        self.get_logger().info(f'Approaching marker {self.target_marker_id}...')
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.marker_visible or self.current_marker_pose is None:
                # Lost sight, try to find again
                time.sleep(0.1)
                continue
            
            marker_x = self.current_marker_pose.position.x
            marker_z = self.current_marker_pose.position.z
            
            twist = Twist()
            
            # First, correct lateral position (centering)
            if abs(marker_x) > self.center_tolerance:
                twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.5
            
            # Then, approach if too far
            if marker_z > self.approach_distance:
                twist.linear.x = self.linear_speed
            else:
                # Close enough
                self.stop_robot()
                self.get_logger().info(f'Reached approach distance: {marker_z:.2f}m')
                return True
            
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop_robot()
        return False
    
    async def center_marker(self, timeout=10.0):
        """Fine-tune to center the marker in the camera view"""
        self.get_logger().info(f'Centering marker {self.target_marker_id}...')
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.marker_visible or self.current_marker_pose is None:
                time.sleep(0.1)
                continue
            
            marker_x = self.current_marker_pose.position.x
            
            if abs(marker_x) < self.center_tolerance:
                self.stop_robot()
                self.get_logger().info('Marker centered')
                return True
            
            twist = Twist()
            twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.3
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop_robot()
        return False
    
    def take_photo(self):
        """Take a photo and mark it with the marker ID"""
        if self.current_image is None:
            self.get_logger().warn('No image available')
            return
        
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')
            
            height, width = cv_image.shape[:2]
            center = (width // 2, height // 2)
            
            # Draw circle around marker
            cv2.circle(cv_image, center, 100, (0, 255, 0), 3)
            
            # Add marker ID text
            text = f'Marker ID: {self.target_marker_id}'
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size = cv2.getTextSize(text, font, 1.0, 2)[0]
            text_x = center[0] - text_size[0] // 2
            text_y = center[1] - 120
            
            # Background rectangle for text
            cv2.rectangle(cv_image, 
                         (text_x - 10, text_y - text_size[1] - 10),
                         (text_x + text_size[0] + 10, text_y + 10),
                         (0, 0, 0), -1)
            cv2.putText(cv_image, text, (text_x, text_y), font, 1.0, (0, 255, 0), 2)
            
            # Add timestamp
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(cv_image, timestamp, (10, height - 20), font, 0.6, (255, 255, 255), 1)
            
            # Convert back to ROS message and publish
            marked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            marked_msg.header = self.current_image.header
            self.marked_image_pub.publish(marked_msg)
            
            # Save to file if enabled
            if self.save_images:
                filename = os.path.join(
                    self.image_save_path,
                    f'marker_{self.target_marker_id}_{int(time.time())}.jpg'
                )
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Saved photo: {filename}')
            
            self.get_logger().info(f'Published marked image for marker {self.target_marker_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    async def execute_set_next_target(self, goal_handle):
        """Execute set_next_target action"""
        self.get_logger().info('Executing set_next_target action...')
        
        args = goal_handle.request.arguments
        parts = args.split()
        
        if len(parts) >= 2:
            prev_marker = parts[0]
            next_marker = parts[1]
            self.get_logger().info(f'Setting next target from {prev_marker} to {next_marker}')
        
        goal_handle.succeed()
        result = ExecuteAction.Result()
        result.success = True
        return result
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    node = VisitMarkerActionNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
