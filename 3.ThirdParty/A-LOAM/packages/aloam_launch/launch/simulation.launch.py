import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare(package='aloam_launch').find('aloam_launch')
    webots_simulation_pkg = FindPackageShare(package='webots_simulation').find('webots_simulation')
    aloam_pkg = FindPackageShare(package='aloam').find('aloam')

    tf2_node1 = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen", emulate_tty=True,
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'Lidar',
            # '--x', '0.0', '--y', '0.0', '--z', '-0.6',
            # '--roll', '0', '--pitch', '0', '--yaw', '0',
            # '--frame-id', 'odom', '--child-frame-id', 'base_link',
            '--ros-args', '--log-level', 'warn'
        ]
    )
    tf2_node2 = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen", emulate_tty=True,
        arguments=[
            # '--x', '0.0', '--y', '0.0', '--z', '0.6',
            # '--roll', '0', '--pitch', '0', '--yaw', '0',
            # '--frame-id', 'base_link', '--child-frame-id', 'Lidar',
            '--x', '0.0', '--y', '0.0', '--z', '-0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
            '--ros-args', '--log-level', 'warn'
        ]
    )

    aloam_params_file = os.path.join(aloam_pkg, "config/params.yaml")
    aloam_node = Node(
        package="aloam", executable="aloam_system", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[aloam_params_file]
    )

    launch_webots_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([webots_simulation_pkg, "/launch", "/webots.launch.py"]))

    launch_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([this_pkg, "/launch", "/rviz2_display.launch.py"]))

    ld = LaunchDescription()
    ld.add_action(launch_webots_simulation)
    ld.add_action(tf2_node1)
    ld.add_action(tf2_node2)
    ld.add_action(TimerAction(period=2.0, actions=[aloam_node]))
    ld.add_action(TimerAction(period=6.0, actions=[launch_rviz2]))  # Delay 6 seconds before launch rviz2

    return ld
