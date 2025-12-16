from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('assignment_1'),
        'worlds',
        'my_world.sdf'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('assignment_1'),
        'rviz',
        'my_config.rviz'
    ])
    
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bme_gazebo_sensors'),
                'launch',
                'spawn_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'rviz_config': rviz_config_path,
        }.items()
    )
    
    aruco_tracker_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_opencv'),
                'launch',
                'aruco_tracker.launch.xml'
            ])
        ])
    )


    aruco_finder_node = Node(
        package='assignment_1',
        executable='aruco_marker_finder',
        name='aruco_marker_finder',
        output='screen'
    )
    
    return LaunchDescription([
        spawn_robot_launch,
        aruco_tracker_launch,
        aruco_finder_node,
    ])