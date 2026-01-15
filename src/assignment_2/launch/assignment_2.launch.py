#!/usr/bin/env python3
"""
Main Launch File for Assignment 2
Launches Gazebo, Nav2, PlanSys2, and all action nodes

Compatible with bme_gazebo_sensors package structure.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_assignment_2 = get_package_share_directory('assignment_2')
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_yaw = LaunchConfiguration('yaw')
    
    # ==================== DECLARE ARGUMENTS ====================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_assignment_2, 'worlds', 'assignment2.world'),
        description='Full path to the world file'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='rviz.rviz',  # Uses bme_gazebo_sensors rviz config
        description='RViz configuration file name (in bme_gazebo_sensors/rviz/)'
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_assignment_2, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )
    
    # Robot spawn position - avoid inner_wall_1 at (0,0)
    declare_x = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='X coordinate of spawned robot'
    )
    
    declare_y = DeclareLaunchArgument(
        'y', default_value='1.5',
        description='Y coordinate of spawned robot'
    )
    
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Yaw angle of spawned robot'
    )
    
    # ==================== ROBOT SPAWN (bme_gazebo_sensors) ====================
    
    # Include spawn_robot.launch.py from bme_gazebo_sensors
    # This launches Gazebo (via erl1), spawns robot, sets up bridges
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'rviz': 'true',
            'rviz_config': rviz_config,
            'use_sim_time': use_sim_time,
            'x': spawn_x,
            'y': spawn_y,
            'yaw': spawn_yaw,
        }.items()
    )
    
    # ==================== ARUCO TRACKER ====================
    
    # ArUco tracker from aruco_opencv package
    aruco_tracker_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aruco_opencv'),
                'launch',
                'aruco_tracker.launch.xml'
            )
        ),
        launch_arguments={
            'cam_image_topic': '/camera/image',
            'cam_info_topic': '/camera/camera_info',
        }.items()
    )
    
    # ==================== NAV2 NAVIGATION ====================
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )
    
    # ==================== PLANSYS2 ====================
    
    pddl_domain_file = os.path.join(pkg_assignment_2, 'pddl', 'domain.pddl')
    
    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('plansys2_bringup'),
                'launch',
                'plansys2_bringup_launch_monolithic.py'
            )
        ),
        launch_arguments={
            'model_file': pddl_domain_file,
        }.items()
    )
    
    # ==================== ASSIGNMENT 2 NODES ====================
    
    action_nodes = GroupAction([
        # Move action node - handles Nav2 navigation
        Node(
            package='assignment_2',
            executable='move_action_node.py',
            name='move_action_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Detect marker action node - scans for ArUco markers
        Node(
            package='assignment_2',
            executable='detect_marker_action_node.py',
            name='detect_marker_action_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rotation_speed': 0.3,
                'scan_duration': 10.0
            }]
        ),
        
        # Visit marker action node - approaches and photographs markers
        Node(
            package='assignment_2',
            executable='visit_marker_action_node.py',
            name='visit_marker_action_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'angular_speed': 0.3,
                'linear_speed': 0.15,
                'approach_distance': 1.0,
                'center_tolerance': 0.08,
                'save_images': True,
                'image_save_path': '/tmp/aruco_photos'
            }]
        ),
        
        # Marker manager node - tracks marker states
        Node(
            package='assignment_2',
            executable='marker_manager_node.py',
            name='marker_manager_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Mission controller node - orchestrates the mission
        Node(
            package='assignment_2',
            executable='mission_controller_node.py',
            name='mission_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
    
    # ==================== LAUNCH DESCRIPTION ====================
    
    return LaunchDescription([
        # Declare all arguments
        declare_use_sim_time,
        declare_world,
        declare_rviz_config,
        declare_nav2_params,
        declare_x,
        declare_y,
        declare_yaw,
        
        # 1. Spawn robot and start Gazebo (immediate)
        spawn_robot_launch,
        
        # 2. Start ArUco tracker (after 3 seconds)
        TimerAction(
            period=3.0,
            actions=[aruco_tracker_launch]
        ),
        
        # 3. Start Nav2 (after 5 seconds - needs robot to be spawned)
        TimerAction(
            period=5.0,
            actions=[nav2_launch]
        ),
        
        # 4. Start PlanSys2 (after 8 seconds)
        TimerAction(
            period=8.0,
            actions=[plansys2_launch]
        ),
        
        # 5. Start assignment nodes (after 12 seconds - all systems ready)
        TimerAction(
            period=12.0,
            actions=[action_nodes]
        ),
    ])
