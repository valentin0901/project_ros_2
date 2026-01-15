#!/usr/bin/env python3
"""
Move Action Node for PlanSys2
Navigates the robot from one waypoint to another using Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from plansys2_msgs.action import ExecuteAction
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading


class MoveActionNode(Node):
    """
    PlanSys2 Action Executor for move action.
    Sends navigation goals to Nav2.
    """
    
    # Waypoint coordinates from assignment hints
    # These are positions from which the robot can see the ArUco markers
    # Markers are located near the corners (~8m from center)
    # Waypoints are at 6m to have good visibility
    WAYPOINTS = {
        'wp_center': (0.0, 0.0, 0.0),
        'wp_nw': (-6.0, 6.0, 2.36),    # Face NW corner (marker at -8.13, 8.96)
        'wp_ne': (6.0, 6.0, 0.78),     # Face NE corner (marker at 8.20, 9.01)
        'wp_sw': (-6.0, -6.0, -2.36),  # Face SW corner (marker at -8.04, -8.85)
        'wp_se': (6.0, -6.0, -0.78),   # Face SE corner (marker at 8.60, -8.90)
    }
    
    # Actual marker positions (for reference)
    MARKER_POSITIONS = {
        'aruco_box': (-8.13, 8.96),     # NW
        'aruco_box_1': (8.20, 9.01),    # NE
        'aruco_box_2': (8.60, -8.90),   # SE
        'aruco_box_3': (-8.04, -8.85),  # SW
    }
    
    def __init__(self):
        super().__init__('move_action_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # PlanSys2 action server
        self._action_server = ActionServer(
            self,
            ExecuteAction,
            'move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.current_nav_goal = None
        self.nav_result = None
        self.nav_done = threading.Event()
        
        self.get_logger().info('Move Action Node initialized')
        self.get_logger().info(f'Available waypoints: {list(self.WAYPOINTS.keys())}')
    
    def goal_callback(self, goal_request):
        """Accept all goals"""
        self.get_logger().info('Received move action goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept all cancel requests"""
        self.get_logger().info('Received cancel request')
        if self.current_nav_goal:
            self.current_nav_goal.cancel_goal_async()
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the move action"""
        self.get_logger().info('Executing move action...')
        
        # Parse arguments: move robot from to
        args = goal_handle.request.arguments
        self.get_logger().info(f'Arguments: {args}')
        
        # Extract waypoints from arguments
        # Format: "robot from_waypoint to_waypoint"
        parts = args.split()
        if len(parts) < 3:
            self.get_logger().error(f'Invalid arguments: {args}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.error_info = 'Invalid arguments'
            return result
        
        robot_name = parts[0]
        from_wp = parts[1]
        to_wp = parts[2]
        
        self.get_logger().info(f'Moving {robot_name} from {from_wp} to {to_wp}')
        
        # Get target coordinates
        if to_wp not in self.WAYPOINTS:
            self.get_logger().error(f'Unknown waypoint: {to_wp}')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.error_info = f'Unknown waypoint: {to_wp}'
            return result
        
        target_x, target_y, target_yaw = self.WAYPOINTS[to_wp]
        
        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.error_info = 'Nav2 not available'
            return result
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = target_x
        nav_goal.pose.pose.position.y = target_y
        nav_goal.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified, only yaw rotation)
        import math
        nav_goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        nav_goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        
        self.get_logger().info(f'Sending navigation goal to ({target_x}, {target_y})')
        
        # Send goal and wait for result
        self.nav_done.clear()
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.nav_feedback_callback
        )
        
        # Wait for goal acceptance
        goal_accepted = await send_goal_future
        if not goal_accepted.accepted:
            self.get_logger().error('Navigation goal rejected')
            goal_handle.abort()
            result = ExecuteAction.Result()
            result.success = False
            result.error_info = 'Navigation goal rejected'
            return result
        
        self.current_nav_goal = goal_accepted
        self.get_logger().info('Navigation goal accepted, waiting for result...')
        
        # Wait for navigation to complete
        result_future = goal_accepted.get_result_async()
        nav_result = await result_future
        
        self.current_nav_goal = None
        
        # Check result
        result = ExecuteAction.Result()
        if nav_result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully arrived at {to_wp}')
            goal_handle.succeed()
            result.success = True
        else:
            self.get_logger().error(f'Navigation failed with status: {nav_result.status}')
            goal_handle.abort()
            result.success = False
            result.error_info = f'Navigation failed: {nav_result.status}'
        
        return result
    
    def nav_feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback
        # Could publish progress here if needed
        pass


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveActionNode()
    
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
