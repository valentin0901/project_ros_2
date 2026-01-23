import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_assignment_2 = get_package_share_directory('assignment_2')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    map_yaml_file = os.path.join(pkg_assignment_2, 'maps', 'map.yaml')
    world_sdf_file = os.path.join(pkg_assignment_2, 'world', 'simple_world.sdf')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_gazebo, 'models')
    )

    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'world': world_sdf_file,
            'headless': 'False',
            'use_sim_time': 'True',
            'initial_pose_x': '0.0', 
            'initial_pose_y': '0.0', 
            'initial_pose_yaw': '0.0',
            'slam': 'False'
        }.items()
    )


    aruco_tracker_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_opencv'),
                'launch',
                'aruco_tracker.launch.xml'
            ])
        ]),
        launch_arguments={
            'cam_base_topic': '/camera/image_raw'
        }.items()
    )


    aruco_navigator_node = Node(
        package='assignment_2',
        executable='aruco_navigator',
        name='aruco_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_tb3_model,
        set_gz_resource_path,
        nav2_simulation,
        aruco_tracker_launch,
        #aruco_navigator_node
    ])