#!/usr/bin/env python3
"""
Assignment 2 Launch File - WITHOUT PlanSys2
Uses autonomous_explorer node with a simple state machine.

This is the recommended launch file for testing.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_assignment_2 = get_package_share_directory('assignment_2')
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ==================== ARGUMENTS ====================
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_x = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='Robot spawn X coordinate'
    )
    
    declare_y = DeclareLaunchArgument(
        'y', default_value='1.5', 
        description='Robot spawn Y coordinate'
    )
    
    # World file
    world_file = os.path.join(pkg_assignment_2, 'worlds', 'assignment2.world')
    nav2_params = os.path.join(pkg_assignment_2, 'config', 'nav2_params.yaml')
    
    # ==================== SPAWN ROBOT ====================
    
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'rviz': 'true',
            'rviz_config': 'rviz.rviz',
            'use_sim_time': 'True',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': '0.0',
        }.items()
    )
    
    # ==================== ARUCO TRACKER ====================
    
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
    
    # ==================== NAV2 ====================
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
        }.items()
    )
    
    # ==================== AUTONOMOUS EXPLORER ====================
    
    autonomous_explorer_node = Node(
        package='assignment_2',
        executable='autonomous_explorer.py',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'angular_speed': 0.4,
            'linear_speed': 0.15,
            'scan_duration': 12.0,
            'approach_distance': 1.2,
            'save_images': True,
            'image_save_path': '/tmp/aruco_photos'
        }]
    )
    
    # ==================== LAUNCH ====================
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_x,
        declare_y,
        
        # 1. Spawn robot (immediate)
        spawn_robot_launch,
        
        # 2. ArUco tracker (after 3s)
        TimerAction(
            period=3.0,
            actions=[aruco_tracker_launch]
        ),
        
        # 3. Nav2 (after 5s)
        TimerAction(
            period=5.0,
            actions=[nav2_launch]
        ),
        
        # 4. Autonomous explorer (after 15s - wait for Nav2)
        TimerAction(
            period=15.0,
            actions=[autonomous_explorer_node]
        ),
    ])
