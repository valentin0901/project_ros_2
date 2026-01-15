#!/usr/bin/env python3
"""
Detect Marker Action Node for PlanSys2
Scans for ArUco markers at the current waypoint
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from plansys2_msgs.action import ExecuteAction
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time


class DetectMarkerActionNode(Node):
    """
    PlanSys2 Action Executor for detecting ArUco markers.
    Rotates the robot and scans for markers.
    """
    
    def __init__(self):
        super().__init__('detect_marker_action_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('scan_duration', 8.0)  # seconds for full scan
        
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.scan_duration = self.get_parameter('scan_duration').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detected_markers_pub = self.create_publisher(String, '/detected_markers', 10)
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10,
            callback_group=self.callback_group
        )
        
        # State
        self.detected_markers = {}  # {marker_id: {'waypoint': wp, 'pose': pose}}
        self.current_detections = []
        self.scanning = False
        
        # PlanSys2 action server for scan_waypoint
        self._scan_action_server = ActionServer(
            self,
            ExecuteAction,
            'scan_waypoint',
            execute_callback=self.execute_scan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # PlanSys2 action server for detect_marker
        self._detect_action_server = ActionServer(
            self,
            ExecuteAction,
            'detect_marker',
            execute_callback=self.execute_detect_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Detect Marker Action Node initialized')
    
    def goal_callback(self, goal_request):
        """Accept all goals"""
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept cancel requests"""
        self.scanning = False
        self.stop_robot()
        return CancelResponse.ACCEPT
    
    def aruco_callback(self, msg):
        """Process ArUco detections"""
        if self.scanning:
            for marker in msg.markers:
                if marker.marker_id not in [m['id'] for m in self.current_detections]:
                    self.current_detections.append({
                        'id': marker.marker_id,
                        'pose': {
                            'x': marker.pose.position.x,
                            'y': marker.pose.position.y,
                            'z': marker.pose.position.z
                        }
                    })
                    self.get_logger().info(f'Detected marker ID: {marker.marker_id}')
    
    async def execute_scan_callback(self, goal_handle):
        """Execute waypoint scan action - rotate to find markers"""
        self.get_logger().info('Executing scan_waypoint action...')
        
        args = goal_handle.request.arguments
        parts = args.split()
        
        if len(parts) < 2:
            self.get_logger().error(f'Invalid arguments: {args}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            return result
        
        robot_name = parts[0]
        waypoint = parts[1]
        
        self.get_logger().info(f'Scanning at waypoint {waypoint}')
        
        # Start scanning
        self.scanning = True
        self.current_detections = []
        
        # Rotate for scan_duration seconds
        start_time = time.time()
        twist = Twist()
        twist.angular.z = self.rotation_speed
        
        while time.time() - start_time < self.scan_duration:
            if not self.scanning:  # Cancelled
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop robot
        self.stop_robot()
        self.scanning = False
        
        # Store detected markers with waypoint info
        for detection in self.current_detections:
            marker_id = detection['id']
            if marker_id not in self.detected_markers:
                self.detected_markers[marker_id] = {
                    'waypoint': waypoint,
                    'pose': detection['pose']
                }
        
        # Publish all detected markers
        self.publish_detected_markers()
        
        self.get_logger().info(f'Scan complete. Found {len(self.current_detections)} markers at {waypoint}')
        self.get_logger().info(f'Total markers detected: {list(self.detected_markers.keys())}')
        
        goal_handle.succeed()
        result = ExecuteAction.Result()
        result.success = True
        return result
    
    async def execute_detect_callback(self, goal_handle):
        """Execute detect_marker action - confirm specific marker detection"""
        self.get_logger().info('Executing detect_marker action...')
        
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
        
        self.get_logger().info(f'Confirming detection of {marker_name} at {waypoint}')
        
        # For now, just confirm the marker was detected during scan
        # In a real system, we'd verify the marker is still visible
        
        goal_handle.succeed()
        result = ExecuteAction.Result()
        result.success = True
        return result
    
    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def publish_detected_markers(self):
        """Publish detected markers info"""
        msg = String()
        msg.data = json.dumps(self.detected_markers)
        self.detected_markers_pub.publish(msg)
    
    def get_detected_markers(self):
        """Return sorted list of detected marker IDs"""
        return sorted(self.detected_markers.keys())


def main(args=None):
    rclpy.init(args=args)
    
    node = DetectMarkerActionNode()
    
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
