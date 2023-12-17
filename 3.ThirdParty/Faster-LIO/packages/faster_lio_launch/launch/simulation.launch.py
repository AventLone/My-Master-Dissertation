import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare(package='faster_lio_launch').find('faster_lio_launch')
    webots_simulation_pkg = FindPackageShare(package='webots_simulation').find('webots_simulation')
    faster_lio_pkg = FindPackageShare(package='faster_lio').find('faster_lio')

    faster_lio_params_path = os.path.join(faster_lio_pkg, "config/params.yaml")
    faster_lio_node = Node(
        package="faster_lio", executable="faster_lio_system", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[faster_lio_params_path]
    )

    tf2_node = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen", emulate_tty=True,
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'Lidar',
            '--x', '0.0', '--y', '0.0', '--z', '-0.6',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
            '--ros-args', '--log-level', 'warn'
        ]
    )

    launch_webots_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([webots_simulation_pkg, "/launch", "/webots.launch.py"]))

    launch_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([this_pkg, "/launch", "/rviz2_display.launch.py"]))

    ld = LaunchDescription()
    ld.add_action(launch_webots_simulation)
    ld.add_action(tf2_node)
    ld.add_action(TimerAction(period=2.0, actions=[faster_lio_node]))
    ld.add_action(TimerAction(period=6.0, actions=[launch_rviz2]))  # Delay 6 seconds before launch rviz2

    return ld
