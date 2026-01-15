#!/usr/bin/env python3
"""
Mission Controller Node
Orchestrates the PlanSys2 planning and execution for the ArUco marker mission.

This node:
1. Sets up the PDDL problem
2. Triggers planning and execution
3. Monitors progress
4. Handles replanning when needed
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from plansys2_msgs.srv import AddProblem, GetPlan, AddProblemGoal
from plansys2_msgs.srv import AffectParam, AffectNode
from plansys2_msgs.msg import ActionExecutionInfo
from std_msgs.msg import String, Int32MultiArray
import json
import time


class MissionControllerNode(Node):
    """
    Main mission controller that uses PlanSys2 for planning.
    
    Mission phases:
    1. EXPLORE: Visit all waypoints to find markers
    2. PHOTOGRAPH: Visit markers in order of ID and photograph them
    """
    
    # Waypoints in the environment
    WAYPOINTS = ['wp_center', 'wp_nw', 'wp_ne', 'wp_sw', 'wp_se']
    EXPLORATION_ORDER = ['wp_nw', 'wp_ne', 'wp_se', 'wp_sw']
    
    def __init__(self):
        super().__init__('mission_controller_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State
        self.state = 'INIT'  # INIT, EXPLORING, PHOTOGRAPHING, COMPLETE
        self.detected_markers = {}
        self.photographed_markers = set()
        self.current_waypoint_idx = 0
        self.markers_to_visit = []
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.current_wp_pub = self.create_publisher(String, '/current_waypoint', 10)
        
        # Subscribers
        self.marker_info_sub = self.create_subscription(
            String,
            '/marker_info',
            self.marker_info_callback,
            10,
            callback_group=self.callback_group
        )
        
        # PlanSys2 service clients
        self.domain_expert_client = None  # Will be set up
        self.problem_expert_client = None
        self.planner_client = None
        self.executor_client = None
        
        # Timer for state machine
        self.create_timer(0.5, self.state_machine_tick)
        
        # Timer for publishing state
        self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('Mission Controller Node initialized')
        self.get_logger().info('Waiting for system to be ready...')
        
        # Give other nodes time to start
        self.startup_delay = 5.0
        self.startup_time = time.time()
    
    def marker_info_callback(self, msg):
        """Update marker information from marker manager"""
        try:
            info = json.loads(msg.data)
            self.detected_markers = info.get('markers', {})
            
            # Convert string keys to int
            self.detected_markers = {
                int(k): v for k, v in self.detected_markers.items()
            }
            
        except json.JSONDecodeError:
            pass
    
    def state_machine_tick(self):
        """Main state machine logic"""
        # Wait for startup delay
        if time.time() - self.startup_time < self.startup_delay:
            return
        
        if self.state == 'INIT':
            self.state = 'EXPLORING'
            self.get_logger().info('Starting exploration phase...')
            self.current_waypoint_idx = 0
            
        elif self.state == 'EXPLORING':
            self.exploration_phase()
            
        elif self.state == 'PHOTOGRAPHING':
            self.photographing_phase()
            
        elif self.state == 'COMPLETE':
            pass  # Mission complete
    
    def exploration_phase(self):
        """
        Exploration phase: Visit all waypoints to detect markers.
        In a full PlanSys2 implementation, this would be done through
        the planner. Here we show the logic directly.
        """
        if self.current_waypoint_idx >= len(self.EXPLORATION_ORDER):
            # Exploration complete
            self.get_logger().info(
                f'Exploration complete. Found {len(self.detected_markers)} markers: '
                f'{sorted(self.detected_markers.keys())}'
            )
            
            if len(self.detected_markers) >= 4:
                # Got all markers, proceed to photographing
                self.markers_to_visit = sorted(self.detected_markers.keys())
                self.state = 'PHOTOGRAPHING'
                self.get_logger().info(
                    f'Starting photographing phase. Order: {self.markers_to_visit}'
                )
            else:
                self.get_logger().warn(
                    f'Only found {len(self.detected_markers)} markers. '
                    'Repeating exploration...'
                )
                self.current_waypoint_idx = 0
            return
        
        # Publish current waypoint
        current_wp = self.EXPLORATION_ORDER[self.current_waypoint_idx]
        wp_msg = String()
        wp_msg.data = current_wp
        self.current_wp_pub.publish(wp_msg)
    
    def photographing_phase(self):
        """
        Photographing phase: Visit markers in order and photograph them.
        """
        # Filter out already photographed markers
        remaining = [
            m for m in self.markers_to_visit
            if m not in self.photographed_markers
        ]
        
        if not remaining:
            self.state = 'COMPLETE'
            self.get_logger().info('=== MISSION COMPLETE ===')
            self.get_logger().info(
                f'Photographed all markers: {sorted(self.photographed_markers)}'
            )
            return
        
        # Next target is the lowest ID remaining
        next_target = remaining[0]
        
        # Get waypoint for this marker
        if next_target in self.detected_markers:
            wp = self.detected_markers[next_target].get('waypoint', 'wp_center')
            
            wp_msg = String()
            wp_msg.data = wp
            self.current_wp_pub.publish(wp_msg)
    
    def publish_state(self):
        """Publish current mission state"""
        state_info = {
            'state': self.state,
            'detected_markers': list(self.detected_markers.keys()),
            'photographed_markers': list(self.photographed_markers),
            'remaining': len(self.detected_markers) - len(self.photographed_markers)
        }
        
        msg = String()
        msg.data = json.dumps(state_info)
        self.state_pub.publish(msg)
    
    def setup_pddl_problem(self):
        """
        Set up the PDDL problem for PlanSys2.
        This would be called to configure the planner.
        """
        # In a full implementation, this would call PlanSys2 services
        # to set up the problem dynamically based on detected markers
        pass
    
    def get_marker_waypoint_mapping(self):
        """
        Get mapping from marker IDs to waypoints.
        Based on actual positions in the world file (simple_world.sdf).
        
        ArUco markers are placed near the corners:
        - aruco_box:   (-8.13, 8.96)  -> NW corner, visible from wp_nw (-6, 6)
        - aruco_box_1: (8.20, 9.01)   -> NE corner, visible from wp_ne (6, 6)
        - aruco_box_2: (8.60, -8.90)  -> SE corner, visible from wp_se (6, -6)
        - aruco_box_3: (-8.04, -8.85) -> SW corner, visible from wp_sw (-6, -6)
        
        The waypoints from the assignment are intermediate positions from which
        the robot can safely detect the markers.
        """
        # Marker positions from world file
        marker_positions = {
            'wp_nw': (-8.13, 8.96),   # aruco_box
            'wp_ne': (8.20, 9.01),    # aruco_box_1
            'wp_se': (8.60, -8.90),   # aruco_box_2
            'wp_sw': (-8.04, -8.85),  # aruco_box_3
        }
        return marker_positions


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionControllerNode()
    
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
