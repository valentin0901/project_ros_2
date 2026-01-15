#!/usr/bin/env python3
"""
Simple Test Launch File for Assignment 2
Tests robot spawn and ArUco detection WITHOUT Nav2.

Usage:
  ros2 launch assignment_2 simple_test.launch.py

Then in another terminal, use teleop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
    
    # ==================== ARGUMENTS ====================
    
    declare_x = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='Robot spawn X'
    )
    
    declare_y = DeclareLaunchArgument(
        'y', default_value='1.5',
        description='Robot spawn Y'
    )
    
    # World file
    world_file = os.path.join(pkg_assignment_2, 'worlds', 'assignment2.world')
    
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
    
    # ==================== MARKER MONITOR ====================
    
    # Simple node to print detected markers
    marker_monitor_node = Node(
        package='assignment_2',
        executable='marker_manager_node.py',
        name='marker_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ==================== LAUNCH ====================
    
    return LaunchDescription([
        declare_x,
        declare_y,
        
        LogInfo(msg='\n' + '='*60),
        LogInfo(msg='SIMPLE TEST - Robot + ArUco Detection'),
        LogInfo(msg='Use teleop to move: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
        LogInfo(msg='='*60 + '\n'),
        
        # 1. Spawn robot
        spawn_robot_launch,
        
        # 2. ArUco tracker (after 3s)
        TimerAction(
            period=3.0,
            actions=[aruco_tracker_launch]
        ),
        
        # 3. Marker monitor (after 5s)
        TimerAction(
            period=5.0,
            actions=[marker_monitor_node]
        ),
    ])
