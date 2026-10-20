import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_pkg = get_package_share_directory('origines_navigation')
    slam_pkg = get_package_share_directory('slam_toolbox')
    params = os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': params,
        }.items(),
    )
    return LaunchDescription([slam])
