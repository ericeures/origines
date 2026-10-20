import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_pkg = get_package_share_directory('origines_navigation')
    bringup = get_package_share_directory('nav2_bringup')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(nav_pkg, 'maps', 'arena.yaml'),
            'params_file': os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )
    return LaunchDescription([nav])
