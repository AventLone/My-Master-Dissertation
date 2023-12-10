import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "track_cart_description"
    urdf_name = "track_cart.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    rviz_config_path = os.path.join(pkg_share, "config/track_cart.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[urdf_model_path]
    )

    # "screen", "log" or "both"
    rviz2_node = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", rviz_config_path]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
