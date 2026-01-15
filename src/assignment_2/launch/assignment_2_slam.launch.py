#!/usr/bin/env python3
"""
Assignment 2 Launch File with SLAM
Uses slam_toolbox for mapping (no pre-built map needed).

This is the most complete version for the assignment.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_assignment_2 = get_package_share_directory('assignment_2')
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Files
    world_file = os.path.join(pkg_assignment_2, 'worlds', 'assignment2.world')
    nav2_params = os.path.join(pkg_assignment_2, 'config', 'nav2_params.yaml')
    
    # ==================== ARGUMENTS ====================
    
    declare_x = DeclareLaunchArgument('x', default_value='2.5')
    declare_y = DeclareLaunchArgument('y', default_value='1.5')
    
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
    
    # ==================== SLAM TOOLBOX ====================
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )
    
    # ==================== NAV2 NAVIGATION ====================
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
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
        declare_x,
        declare_y,
        
        LogInfo(msg='\n' + '='*60),
        LogInfo(msg='ASSIGNMENT 2 - With SLAM (no map required)'),
        LogInfo(msg='='*60 + '\n'),
        
        # 1. Spawn robot (immediate)
        spawn_robot_launch,
        
        # 2. ArUco tracker (after 3s)
        TimerAction(
            period=3.0,
            actions=[aruco_tracker_launch]
        ),
        
        # 3. SLAM (after 5s)
        TimerAction(
            period=5.0,
            actions=[slam_launch]
        ),
        
        # 4. Nav2 (after 8s)
        TimerAction(
            period=8.0,
            actions=[nav2_launch]
        ),
        
        # 5. Autonomous explorer (after 18s - wait for SLAM & Nav2)
        TimerAction(
            period=18.0,
            actions=[autonomous_explorer_node]
        ),
    ])
