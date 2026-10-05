import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('origines_sim')
    desc_pkg = get_package_share_directory('origines_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(sim_pkg, 'worlds', 'arena.sdf')
    urdf = os.path.join(desc_pkg, 'urdf', 'origines_rover.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    # 1. Gazebo avec l'arène (-r : la simulation démarre tout de suite)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world}.items(),
    )

    # 2. Description du rover pour ROS
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # 3. Placer le rover dans Gazebo (x = -1,5 m : face à la caisse)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'origines_rover',
                   '-x', '-1.5', '-y', '0.0', '-z', '0.02'],
    )

    # 4. Pont Gazebo <-> ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([gz, rsp, spawn, bridge])
