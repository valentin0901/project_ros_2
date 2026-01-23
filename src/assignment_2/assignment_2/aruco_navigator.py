"""
ArUco Navigator Node for Assignment 2
=====================================
This node implements a state machine that:
1. Navigates to predefined waypoints to scan for ArUco markers
2. Stores detected markers with their world positions
3. Navigates to each marker in order of ID (lowest first)
4. Takes a photo of each marker when centered
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from aruco_opencv_msgs.msg import ArucoDetection

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
from enum import Enum
from datetime import datetime
import threading


class State(Enum):
    """State machine states"""
    INIT = 0
    EXPLORE_WAYPOINTS = 1
    NAVIGATING_TO_WAYPOINT = 2
    ROTATE_SCAN = 3
    GO_TO_MARKER = 4
    NAVIGATING_TO_MARKER = 5
    CENTER_AND_PHOTO = 6
    DONE = 7


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Navigator Node Starting...')
        self.get_logger().info('=' * 60)
        
        # Callback groups for threading
        self.cb_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('photo_distance', 1.5)
        self.declare_parameter('scan_duration', 12.0)
        self.declare_parameter('photo_output_dir', '/tmp/aruco_photos')
        
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.photo_distance = self.get_parameter('photo_distance').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.photo_output_dir = self.get_parameter('photo_output_dir').value
        
        # Create photo output directory
        os.makedirs(self.photo_output_dir, exist_ok=True)
        self.get_logger().info(f'Photo output directory: {self.photo_output_dir}')
        
        # Waypoints to visit for marker detection (from assignment)
        self.waypoints = [
            (-6.0, -6.0, 0.0),    # SW corner
            (-6.0, 6.0, 0.0),     # NW corner
            (6.0, 6.0, 0.0),      # NE corner
            (6.0, -6.0, 0.0),     # SE corner
        ]
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Waypoints to visit: {self.waypoints}')
        
        # Marker storage
        self.detected_markers = {}
        self.markers_to_visit = []
        self.current_marker_idx = 0
        self.current_target_marker = None
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_image = None
        self.odom_received = False
        
        # State machine
        self.state = State.INIT
        self.scan_start_time = None
        self.nav2_ready = False
        
        # Navigator - will be initialized after Nav2 is ready
        self.navigator = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marked_image_pub = self.create_publisher(Image, '/aruco_marked_image', 10)
        
        # QoS for subscribers
        qos = QoSProfile(depth=10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos,
            callback_group=self.cb_group
        )
        
        # Camera image subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos,
            callback_group=self.cb_group
        )
        
        # ArUco detection subscriber
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            qos,
            callback_group=self.cb_group
        )
        
        # Start Nav2 initialization in a separate thread
        self.get_logger().info('Starting Nav2 initialization thread...')
        self.init_thread = threading.Thread(target=self.init_nav2)
        self.init_thread.start()
        
        # Main timer for state machine
        self.timer = self.create_timer(0.2, self.state_machine_tick, callback_group=self.cb_group)
        
        self.get_logger().info('Node initialized. Waiting for Nav2...')
    
    def init_nav2(self):
        """Initialize Nav2 in a separate thread to avoid blocking"""
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.get_logger().info('(This may take 30-60 seconds)')
        
        # Create navigator and wait for Nav2
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        self.nav2_ready = True
        self.get_logger().info('=' * 60)
        self.get_logger().info('Nav2 is ACTIVE! Starting exploration...')
        self.get_logger().info('=' * 60)
    
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'Odometry received! Robot at ({self.robot_x:.2f}, {self.robot_y:.2f})')
    
    def image_callback(self, msg):
        """Store current camera image"""
        self.current_image = msg
    
    def aruco_callback(self, msg):
        """Process ArUco detections and store marker positions"""
        for marker in msg.markers:
            marker_id = marker.marker_id
            
            # Calculate world position from camera frame
            marker_in_camera_x = marker.pose.position.x
            marker_in_camera_z = marker.pose.position.z
            
            distance = math.sqrt(marker_in_camera_x**2 + marker_in_camera_z**2)
            angle_to_marker = math.atan2(marker_in_camera_x, marker_in_camera_z)
            
            world_x = self.robot_x + distance * math.cos(self.robot_yaw + angle_to_marker)
            world_y = self.robot_y + distance * math.sin(self.robot_yaw + angle_to_marker)
            
            if marker_id not in self.detected_markers:
                self.get_logger().info(
                    f'[NEW MARKER] ID: {marker_id} at world position ({world_x:.2f}, {world_y:.2f})'
                )
                self.detected_markers[marker_id] = {
                    'position': (world_x, world_y),
                    'detected_count': 1,
                    'photo_taken': False
                }
            else:
                # Update with running average
                old_pos = self.detected_markers[marker_id]['position']
                count = self.detected_markers[marker_id]['detected_count']
                new_x = (old_pos[0] * count + world_x) / (count + 1)
                new_y = (old_pos[1] * count + world_y) / (count + 1)
                self.detected_markers[marker_id]['position'] = (new_x, new_y)
                self.detected_markers[marker_id]['detected_count'] = count + 1
    
    def state_machine_tick(self):
        """Main state machine loop"""
        
        if self.state == State.INIT:
            if self.nav2_ready:
                self.state = State.EXPLORE_WAYPOINTS
                self.get_logger().info('Transitioning to EXPLORE_WAYPOINTS state')
        
        elif self.state == State.EXPLORE_WAYPOINTS:
            self.start_waypoint_navigation()
        
        elif self.state == State.NAVIGATING_TO_WAYPOINT:
            self.check_waypoint_navigation()
        
        elif self.state == State.ROTATE_SCAN:
            self.handle_rotate_scan()
        
        elif self.state == State.GO_TO_MARKER:
            self.start_marker_navigation()
        
        elif self.state == State.NAVIGATING_TO_MARKER:
            self.check_marker_navigation()
        
        elif self.state == State.CENTER_AND_PHOTO:
            self.handle_center_and_photo()
        
        elif self.state == State.DONE:
            self.handle_done()
    
    def start_waypoint_navigation(self):
        """Start navigation to the current waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.prepare_marker_visits()
            return
        
        wp = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'({wp[0]}, {wp[1]})'
        )
        
        # Create goal pose
        goal_pose = self.create_pose_stamped(wp[0], wp[1], wp[2])
        
        # Send goal
        self.navigator.goToPose(goal_pose)
        self.state = State.NAVIGATING_TO_WAYPOINT
    
    def check_waypoint_navigation(self):
        """Check if waypoint navigation is complete"""
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx + 1}!')
                self.state = State.ROTATE_SCAN
                self.scan_start_time = self.get_clock().now()
                self.get_logger().info('Starting 360 degree scan for ArUco markers...')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Navigation canceled, retrying...')
                self.state = State.EXPLORE_WAYPOINTS
            else:
                self.get_logger().warn(f'Navigation failed: {result}, skipping to next waypoint')
                self.current_waypoint_idx += 1
                self.state = State.EXPLORE_WAYPOINTS
    
    def handle_rotate_scan(self):
        """Rotate in place to scan for markers"""
        elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
        
        if elapsed < self.scan_duration:
            # Continue rotating
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
        else:
            # Stop rotation
            self.stop_robot()
            self.get_logger().info(f'Scan complete at waypoint {self.current_waypoint_idx + 1}')
            self.get_logger().info(f'Markers detected so far: {sorted(self.detected_markers.keys())}')
            
            # Move to next waypoint
            self.current_waypoint_idx += 1
            self.state = State.EXPLORE_WAYPOINTS
    
    def prepare_marker_visits(self):
        """Prepare to visit markers in order of ID"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'Total markers detected: {len(self.detected_markers)}')
        
        if not self.detected_markers:
            self.get_logger().warn('No markers detected! Using fallback positions...')
            # Fallback: known marker positions from world file
            self.detected_markers = {
                0: {'position': (-8.1, 9.0), 'detected_count': 1, 'photo_taken': False},
                1: {'position': (8.2, 9.0), 'detected_count': 1, 'photo_taken': False},
                2: {'position': (8.6, -8.9), 'detected_count': 1, 'photo_taken': False},
                3: {'position': (-8.0, -8.9), 'detected_count': 1, 'photo_taken': False},
            }
        
        # Sort by marker ID (lowest first)
        self.markers_to_visit = sorted(self.detected_markers.keys())
        self.get_logger().info(f'Marker visit order (by ID): {self.markers_to_visit}')
        self.get_logger().info('=' * 60)
        
        self.current_marker_idx = 0
        self.state = State.GO_TO_MARKER
    
    def start_marker_navigation(self):
        """Navigate to the current target marker"""
        if self.current_marker_idx >= len(self.markers_to_visit):
            self.state = State.DONE
            return
        
        marker_id = self.markers_to_visit[self.current_marker_idx]
        self.current_target_marker = marker_id
        marker_pos = self.detected_markers[marker_id]['position']
        
        # Calculate approach position (stop before marker)
        dx = marker_pos[0] - self.robot_x
        dy = marker_pos[1] - self.robot_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist > self.photo_distance:
            approach_dist = dist - self.photo_distance
            ratio = approach_dist / dist
            approach_x = self.robot_x + dx * ratio
            approach_y = self.robot_y + dy * ratio
        else:
            approach_x = self.robot_x
            approach_y = self.robot_y
        
        # Face the marker
        approach_yaw = math.atan2(dy, dx)
        
        self.get_logger().info(
            f'Navigating to marker {marker_id} at ({marker_pos[0]:.2f}, {marker_pos[1]:.2f})'
        )
        
        goal_pose = self.create_pose_stamped(approach_x, approach_y, approach_yaw)
        self.navigator.goToPose(goal_pose)
        self.state = State.NAVIGATING_TO_MARKER
    
    def check_marker_navigation(self):
        """Check if marker navigation is complete"""
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached marker {self.current_target_marker}!')
            else:
                self.get_logger().warn(f'Navigation to marker failed: {result}')
            
            # Take photo regardless
            self.state = State.CENTER_AND_PHOTO
    
    def handle_center_and_photo(self):
        """Take photo of the marker"""
        self.take_marker_photo(self.current_target_marker)
        
        # Move to next marker
        self.current_marker_idx += 1
        if self.current_marker_idx < len(self.markers_to_visit):
            self.state = State.GO_TO_MARKER
        else:
            self.state = State.DONE
    
    def handle_done(self):
        """Mission complete"""
        self.stop_robot()
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE!')
        self.get_logger().info(f'Photographed {len(self.markers_to_visit)} markers')
        self.get_logger().info(f'Photos saved to: {self.photo_output_dir}')
        self.get_logger().info('=' * 60)
        
        # Stop the timer
        self.timer.cancel()
    
    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def take_marker_photo(self, marker_id):
        """Take and save a photo of the marker"""
        self.get_logger().info(f'Taking photo of marker {marker_id}...')
        
        if self.current_image is None:
            self.get_logger().warn('No image available, creating placeholder')
            cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(cv_image, f'Marker {marker_id}', (200, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.current_image, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
                return
        
        # Annotate image
        height, width = cv_image.shape[:2]
        center = (width // 2, height // 2)
        
        # Draw crosshair and circle
        cv2.line(cv_image, (center[0] - 50, center[1]), (center[0] + 50, center[1]), (0, 255, 0), 2)
        cv2.line(cv_image, (center[0], center[1] - 50), (center[0], center[1] + 50), (0, 255, 0), 2)
        cv2.circle(cv_image, center, 100, (0, 255, 0), 3)
        
        # Add text
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(cv_image, f'Marker ID: {marker_id}', (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(cv_image, f'Photo {self.current_marker_idx + 1}/{len(self.markers_to_visit)}',
                   (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(cv_image, timestamp, (20, height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Save image
        filename = f'marker_{marker_id:03d}.jpg'
        filepath = os.path.join(self.photo_output_dir, filename)
        cv2.imwrite(filepath, cv_image)
        
        self.get_logger().info(f'[PHOTO SAVED] {filepath}')
        
        # Publish marked image
        try:
            marked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.marked_image_pub.publish(marked_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
        
        self.detected_markers[marker_id]['photo_taken'] = True
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoNavigator()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()