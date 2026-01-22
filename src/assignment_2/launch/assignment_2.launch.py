import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Chemins des fichiers et dossiers
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_assignment_2 = get_package_share_directory('assignment_2')
    
    # Chemins absolus vers vos ressources
    map_yaml_file = os.path.join(pkg_assignment_2, 'maps', 'map.yaml')
    world_sdf_file = os.path.join(pkg_assignment_2, 'world', 'simple_world.sdf')

    # 2. Forcer le modèle Waffle (pour avoir la caméra comme discuté)
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # 3. Inclusion du launch file officiel de Nav2
    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'world': world_sdf_file,
            'headless': 'False',
            'use_sim_time': 'True',
            'slam': 'False' # On utilise la carte enregistrée, pas le SLAM
        }.items()
    )

    return LaunchDescription([
        set_tb3_model,
        nav2_simulation
    ])