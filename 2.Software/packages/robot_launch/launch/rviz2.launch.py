import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare(package="robot_launch").find("robot_launch")
    robot_description_pkg = FindPackageShare(package="robot_description").find("robot_description")

    urdf_path = os.path.join(robot_description_pkg, "urdf/track_cart.urdf")
    rviz_config_path = os.path.join(this_pkg, "config/simulation_putn.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", emulate_tty=True,
        arguments=[urdf_path, '--ros-args', '--log-level', 'warn']
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
        output="screen", emulate_tty=True,
        arguments=[urdf_path, '--ros-args', '--log-level', 'warn']
    )

    # log output = "screen" or "log"
    rviz2_node = Node(
        package="rviz2", executable="rviz2", output="screen", emulate_tty=True,
        arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'warn']
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
