import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    # ----- Define the path of some packages ------ #
    this_pkg = FindPackageShare(package="robot_launch").find("robot_launch")
    simulation_pkg = FindPackageShare(package="webots_simulation").find("webots_simulation")
    orb_slam3_pkg = FindPackageShare(package="orb_slam3").find("orb_slam3")

    # Launch Orb-SLAM3 Node
    orb_slam3_params_file = os.path.join(orb_slam3_pkg, "config/params.yaml")
    orb_slam3_node = Node(
        package="orb_slam3", executable="orb_slam3", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[orb_slam3_params_file]
    )

    tf2_node = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen", emulate_tty=True,
        arguments=[
            # '--x', '-0.187658698280417', '--y', '0.0', '--z', '-0.4583819003855137',
            '--x', '-0.187658698280417', '--y', '0.0', '--z', '-0.6583819003855137',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
            '--ros-args', '--log-level', 'warn'
        ]
    )

    # ------------- Define launch actions ----------------- #
    launch_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_pkg, "/launch", "/webots.launch.py"]))
    launch_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([this_pkg, "/launch", "/rviz2.launch.py"]))

    ld = LaunchDescription()
    ld.add_action(launch_webots)
    ld.add_action(tf2_node)
    ld.add_action(TimerAction(period=1.0, actions=[orb_slam3_node]))
    ld.add_action(TimerAction(period=6.0, actions=[launch_rviz2]))  # Delay 6 seconds before launch rviz2
    return ld
