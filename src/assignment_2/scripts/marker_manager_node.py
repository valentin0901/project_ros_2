#!/usr/bin/env python3
"""
Marker Manager Node
Keeps track of detected markers and their states
Provides services for querying marker information
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
import json


class MarkerManagerNode(Node):
    """
    Manages ArUco marker detection state.
    Tracks which markers have been seen and at which waypoints.
    """
    
    # Waypoint positions for determining which waypoint a marker belongs to
    # Assignment waypoints (robot detection positions)
    WAYPOINTS = {
        'wp_nw': (-6.0, 6.0),   # Northwest - sees marker at (-8.13, 8.96)
        'wp_ne': (6.0, 6.0),    # Northeast - sees marker at (8.20, 9.01)
        'wp_sw': (-6.0, -6.0),  # Southwest - sees marker at (-8.04, -8.85)
        'wp_se': (6.0, -6.0),   # Southeast - sees marker at (8.60, -8.90)
    }
    
    # Actual ArUco marker positions from the world file
    MARKER_POSITIONS = {
        'aruco_box': (-8.13, 8.96),     # NW corner
        'aruco_box_1': (8.20, 9.01),    # NE corner
        'aruco_box_2': (8.60, -8.90),   # SE corner
        'aruco_box_3': (-8.04, -8.85),  # SW corner
    }
    
    def __init__(self):
        super().__init__('marker_manager_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State tracking
        self.detected_markers = {}  # {marker_id: {'waypoint': str, 'detected': bool, 'photographed': bool}}
        self.current_waypoint = 'wp_center'
        
        # Publishers
        self.marker_list_pub = self.create_publisher(
            Int32MultiArray, '/marker_ids_sorted', 10)
        self.marker_info_pub = self.create_publisher(
            String, '/marker_info', 10)
        self.next_target_pub = self.create_publisher(
            Int32MultiArray, '/next_target_marker', 10)
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.waypoint_sub = self.create_subscription(
            String,
            '/current_waypoint',
            self.waypoint_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.photographed_sub = self.create_subscription(
            Int32MultiArray,
            '/photographed_markers',
            self.photographed_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Timer for periodic publishing
        self.create_timer(1.0, self.publish_marker_info)
        
        self.get_logger().info('Marker Manager Node initialized')
    
    def aruco_callback(self, msg):
        """Process ArUco detections and update marker database"""
        for marker in msg.markers:
            marker_id = marker.marker_id
            
            if marker_id not in self.detected_markers:
                self.detected_markers[marker_id] = {
                    'waypoint': self.current_waypoint,
                    'detected': True,
                    'photographed': False,
                    'pose': {
                        'x': marker.pose.position.x,
                        'y': marker.pose.position.y,
                        'z': marker.pose.position.z
                    }
                }
                self.get_logger().info(
                    f'New marker detected: ID {marker_id} at {self.current_waypoint}')
                self.publish_marker_list()
    
    def waypoint_callback(self, msg):
        """Update current waypoint"""
        self.current_waypoint = msg.data
        self.get_logger().debug(f'Current waypoint: {self.current_waypoint}')
    
    def photographed_callback(self, msg):
        """Mark markers as photographed"""
        for marker_id in msg.data:
            if marker_id in self.detected_markers:
                self.detected_markers[marker_id]['photographed'] = True
                self.get_logger().info(f'Marker {marker_id} marked as photographed')
    
    def publish_marker_list(self):
        """Publish sorted list of detected marker IDs"""
        sorted_ids = sorted(self.detected_markers.keys())
        
        msg = Int32MultiArray()
        msg.data = sorted_ids
        self.marker_list_pub.publish(msg)
    
    def publish_marker_info(self):
        """Publish full marker information as JSON"""
        info = {
            'markers': self.detected_markers,
            'total_detected': len(self.detected_markers),
            'total_photographed': sum(
                1 for m in self.detected_markers.values() if m['photographed']
            ),
            'sorted_ids': sorted(self.detected_markers.keys())
        }
        
        msg = String()
        msg.data = json.dumps(info)
        self.marker_info_pub.publish(msg)
        
        # Also publish next target
        self.publish_next_target()
    
    def publish_next_target(self):
        """Publish the next marker to photograph (lowest ID not yet photographed)"""
        unphotographed = [
            mid for mid, info in self.detected_markers.items()
            if not info['photographed']
        ]
        
        msg = Int32MultiArray()
        if unphotographed:
            msg.data = [min(unphotographed)]
        else:
            msg.data = []
        
        self.next_target_pub.publish(msg)
    
    def get_sorted_marker_ids(self):
        """Return sorted list of detected marker IDs"""
        return sorted(self.detected_markers.keys())
    
    def get_next_target(self):
        """Get the next marker to photograph"""
        unphotographed = [
            mid for mid, info in self.detected_markers.items()
            if not info['photographed']
        ]
        return min(unphotographed) if unphotographed else None
    
    def get_marker_waypoint(self, marker_id):
        """Get the waypoint where a marker was detected"""
        if marker_id in self.detected_markers:
            return self.detected_markers[marker_id]['waypoint']
        return None
    
    def all_markers_photographed(self):
        """Check if all detected markers have been photographed"""
        if not self.detected_markers:
            return False
        return all(m['photographed'] for m in self.detected_markers.values())


def main(args=None):
    rclpy.init(args=args)
    
    node = MarkerManagerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
