#!/usr/bin/env python3
"""
Autonomous Explorer Node for Assignment 2
This is a simplified version that uses a state machine directly instead of PlanSys2.

States:
1. INIT - Wait for systems to be ready
2. GO_TO_WAYPOINT - Navigate to next waypoint using Nav2
3. SCAN_FOR_MARKERS - Rotate to detect ArUco markers
4. GO_TO_MARKER - Navigate to marker waypoint
5. PHOTOGRAPH_MARKER - Approach and photograph the marker
6. DONE - All markers photographed
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import os


class AutonomousExplorer(Node):
    """
    Main node that handles the complete mission:
    - Explore waypoints to find markers
    - Visit markers in order of ID
    - Photograph each marker
    """
    
    # Waypoints from assignment (robot detection positions)
    WAYPOINTS = {
        'wp_start': (2.5, 1.5),   # Starting position (avoids inner_wall_1 at 0,0)
        'wp_nw': (-6.0, 6.0),     # Can see marker at (-8.13, 8.96)
        'wp_ne': (6.0, 6.0),      # Can see marker at (8.20, 9.01)
        'wp_sw': (-6.0, -6.0),    # Can see marker at (-8.04, -8.85)
        'wp_se': (6.0, -6.0),     # Can see marker at (8.60, -8.90)
    }
    
    # Exploration order
    EXPLORATION_ORDER = ['wp_nw', 'wp_ne', 'wp_se', 'wp_sw']
    
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('scan_duration', 12.0)
        self.declare_parameter('approach_distance', 1.2)
        self.declare_parameter('save_images', True)
        self.declare_parameter('image_save_path', '/tmp/aruco_photos')
        
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.save_images = self.get_parameter('save_images').value
        self.image_save_path = self.get_parameter('image_save_path').value
        
        # Create image save directory
        if self.save_images:
            os.makedirs(self.image_save_path, exist_ok=True)
        
        # State machine
        self.state = 'INIT'
        self.exploration_idx = 0
        self.detected_markers = {}  # {marker_id: waypoint}
        self.photographed_markers = set()
        self.markers_to_visit = []
        self.current_target_marker = None
        
        # Navigation state
        self.nav_in_progress = False
        self.nav_succeeded = False
        
        # ArUco state
        self.current_marker_pose = None
        self.marker_visible = False
        self.current_image = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marked_image_pub = self.create_publisher(Image, '/aruco_marked_image', 10)
        self.state_pub = self.create_publisher(String, '/explorer_state', 10)
        
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
        
        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # State machine timer
        self.state_timer = self.create_timer(0.5, self.state_machine_tick)
        
        # Startup delay
        self.start_time = time.time()
        self.startup_delay = 8.0  # Wait for Nav2 to be ready
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Autonomous Explorer Node Started')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Waypoints: {list(self.WAYPOINTS.keys())}')
        self.get_logger().info(f'Waiting {self.startup_delay}s for systems to initialize...')
    
    def aruco_callback(self, msg):
        """Process ArUco detections"""
        self.marker_visible = False
        
        for marker in msg.markers:
            marker_id = marker.marker_id
            
            # Track all detected markers
            if marker_id not in self.detected_markers:
                # Determine which waypoint we're at based on current navigation
                if self.exploration_idx > 0 and self.exploration_idx <= len(self.EXPLORATION_ORDER):
                    current_wp = self.EXPLORATION_ORDER[self.exploration_idx - 1]
                else:
                    current_wp = 'unknown'
                
                self.detected_markers[marker_id] = current_wp
                self.get_logger().info(f'🎯 NEW MARKER DETECTED: ID {marker_id} at {current_wp}')
            
            # Track current target marker
            if self.current_target_marker is not None and marker_id == self.current_target_marker:
                self.marker_visible = True
                self.current_marker_pose = marker.pose
    
    def image_callback(self, msg):
        """Store current camera image"""
        self.current_image = msg
    
    def state_machine_tick(self):
        """Main state machine"""
        # Publish current state
        state_msg = String()
        state_msg.data = f'{self.state} | Detected: {sorted(self.detected_markers.keys())} | Photographed: {sorted(self.photographed_markers)}'
        self.state_pub.publish(state_msg)
        
        # Wait for startup
        if time.time() - self.start_time < self.startup_delay:
            return
        
        # State machine logic
        if self.state == 'INIT':
            self.handle_init_state()
        elif self.state == 'GO_TO_WAYPOINT':
            self.handle_go_to_waypoint_state()
        elif self.state == 'SCAN_FOR_MARKERS':
            self.handle_scan_state()
        elif self.state == 'GO_TO_MARKER':
            self.handle_go_to_marker_state()
        elif self.state == 'PHOTOGRAPH_MARKER':
            self.handle_photograph_state()
        elif self.state == 'DONE':
            pass  # Mission complete
    
    def handle_init_state(self):
        """Initialize and start exploration"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('STARTING EXPLORATION PHASE')
        self.get_logger().info('=' * 50)
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not ready yet, waiting...')
            return
        
        self.exploration_idx = 0
        self.state = 'GO_TO_WAYPOINT'
    
    def handle_go_to_waypoint_state(self):
        """Navigate to next waypoint"""
        if self.nav_in_progress:
            return  # Wait for navigation to complete
        
        if self.nav_succeeded:
            # Navigation complete, start scanning
            self.nav_succeeded = False
            self.state = 'SCAN_FOR_MARKERS'
            self.scan_start_time = time.time()
            wp_name = self.EXPLORATION_ORDER[self.exploration_idx - 1]
            self.get_logger().info(f'📍 Arrived at {wp_name}, scanning for markers...')
            return
        
        # Check if exploration is complete
        if self.exploration_idx >= len(self.EXPLORATION_ORDER):
            self.finish_exploration()
            return
        
        # Navigate to next waypoint
        wp_name = self.EXPLORATION_ORDER[self.exploration_idx]
        wp_coords = self.WAYPOINTS[wp_name]
        
        self.get_logger().info(f'🚗 Navigating to {wp_name} ({wp_coords[0]}, {wp_coords[1]})')
        self.exploration_idx += 1
        
        self.send_nav_goal(wp_coords[0], wp_coords[1])
    
    def handle_scan_state(self):
        """Rotate to scan for markers"""
        elapsed = time.time() - self.scan_start_time
        
        if elapsed < self.scan_duration:
            # Keep rotating
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
        else:
            # Scan complete
            self.stop_robot()
            self.get_logger().info(f'✅ Scan complete. Total markers found: {len(self.detected_markers)}')
            self.state = 'GO_TO_WAYPOINT'
    
    def finish_exploration(self):
        """Exploration complete, start photographing markers"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('EXPLORATION COMPLETE')
        self.get_logger().info(f'Found {len(self.detected_markers)} markers: {sorted(self.detected_markers.keys())}')
        self.get_logger().info('=' * 50)
        
        if len(self.detected_markers) == 0:
            self.get_logger().error('No markers found! Check ArUco detection.')
            self.state = 'DONE'
            return
        
        # Sort markers by ID for visiting order
        self.markers_to_visit = sorted(self.detected_markers.keys())
        self.get_logger().info(f'Visit order: {self.markers_to_visit}')
        
        # Start photographing
        self.state = 'GO_TO_MARKER'
    
    def handle_go_to_marker_state(self):
        """Navigate to next marker's waypoint"""
        if self.nav_in_progress:
            return
        
        if self.nav_succeeded:
            self.nav_succeeded = False
            self.state = 'PHOTOGRAPH_MARKER'
            self.photograph_phase = 'FIND'
            self.get_logger().info(f'📍 Arrived at marker waypoint, preparing to photograph marker {self.current_target_marker}')
            return
        
        # Get next unphotographed marker
        remaining = [m for m in self.markers_to_visit if m not in self.photographed_markers]
        
        if not remaining:
            self.mission_complete()
            return
        
        # Target the next marker
        self.current_target_marker = remaining[0]
        waypoint = self.detected_markers[self.current_target_marker]
        
        self.get_logger().info(f'🎯 Next target: Marker {self.current_target_marker} at {waypoint}')
        
        if waypoint in self.WAYPOINTS:
            coords = self.WAYPOINTS[waypoint]
            self.send_nav_goal(coords[0], coords[1])
        else:
            self.get_logger().warn(f'Unknown waypoint {waypoint}, skipping marker')
            self.photographed_markers.add(self.current_target_marker)
    
    def handle_photograph_state(self):
        """Approach and photograph the marker"""
        if self.photograph_phase == 'FIND':
            # Rotate to find the marker
            if self.marker_visible:
                self.stop_robot()
                self.photograph_phase = 'APPROACH'
                self.get_logger().info(f'👁️ Marker {self.current_target_marker} found, approaching...')
            else:
                twist = Twist()
                twist.angular.z = self.angular_speed * 0.5
                self.cmd_vel_pub.publish(twist)
        
        elif self.photograph_phase == 'APPROACH':
            if not self.marker_visible:
                self.photograph_phase = 'FIND'
                return
            
            marker_x = self.current_marker_pose.position.x
            marker_z = self.current_marker_pose.position.z
            
            twist = Twist()
            
            # Center the marker
            if abs(marker_x) > 0.1:
                twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.4
            
            # Approach
            if marker_z > self.approach_distance:
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.photograph_phase = 'CENTER'
                self.get_logger().info('📸 Close enough, centering...')
        
        elif self.photograph_phase == 'CENTER':
            if not self.marker_visible:
                self.photograph_phase = 'FIND'
                return
            
            marker_x = self.current_marker_pose.position.x
            
            if abs(marker_x) < 0.05:
                self.stop_robot()
                self.take_photo()
                self.photographed_markers.add(self.current_target_marker)
                self.get_logger().info(f'✅ Marker {self.current_target_marker} photographed!')
                self.state = 'GO_TO_MARKER'
            else:
                twist = Twist()
                twist.angular.z = -np.sign(marker_x) * self.angular_speed * 0.3
                self.cmd_vel_pub.publish(twist)
    
    def take_photo(self):
        """Capture and annotate photo"""
        if self.current_image is None:
            self.get_logger().warn('No image available!')
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')
            h, w = cv_image.shape[:2]
            center = (w // 2, h // 2)
            
            # Draw circle and text
            cv2.circle(cv_image, center, 100, (0, 255, 0), 3)
            text = f'Marker ID: {self.current_target_marker}'
            cv2.putText(cv_image, text, (center[0] - 80, center[1] - 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Add timestamp and marker count
            info_text = f'Photographed: {len(self.photographed_markers) + 1}/{len(self.markers_to_visit)}'
            cv2.putText(cv_image, info_text, (10, h - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish marked image
            marked_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.marked_image_pub.publish(marked_msg)
            
            # Save to file
            if self.save_images:
                filename = os.path.join(
                    self.image_save_path,
                    f'marker_{self.current_target_marker}.jpg'
                )
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'💾 Saved: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Photo error: {e}')
    
    def mission_complete(self):
        """All markers photographed"""
        self.state = 'DONE'
        self.get_logger().info('=' * 50)
        self.get_logger().info('🎉 MISSION COMPLETE!')
        self.get_logger().info(f'Photographed markers: {sorted(self.photographed_markers)}')
        self.get_logger().info('=' * 50)
    
    def send_nav_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2"""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.nav_in_progress = True
        self.nav_succeeded = False
        
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.nav_in_progress = False
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        self.nav_in_progress = False
        
        if result.status == 4:  # SUCCEEDED
            self.nav_succeeded = True
        else:
            self.get_logger().warn(f'Navigation failed with status {result.status}')
            # Try to continue anyway
            self.nav_succeeded = True
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    
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
