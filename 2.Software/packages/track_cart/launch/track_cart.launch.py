import os
from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    # ----- Define the path of some packages ------ #
    track_cart_pkg = FindPackageShare(package="track_cart").find("track_cart")
    track_cart_drive_pkg = FindPackageShare(package="track_cart_drive_sim").find("track_cart_drive_sim")
    track_cart_description_pkg = FindPackageShare(package="track_cart_description").find("track_cart_description")
    orb_slam3_pkg = FindPackageShare(package="orb_slam3").find("orb_slam3")

    # Launch Orb-SLAM3 Node
    orb_slam3_param_file = os.path.join(orb_slam3_pkg, "config/params.yaml")
    orb_slam3_octomap = Node(
        package="orb_slam3", executable="rgbd", name="orb_slam3_rgbd",
        output="screen", emulate_tty=True,
        parameters=[orb_slam3_param_file]
    )

    tf2_node = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen", emulate_tty=True,
        arguments=["-0.187658698280417", "0.0", "-0.0983819003855137", "0", "0", "0", "odom", "base_link"]
    )

    # ------------- Define launch actions ----------------- #
    launch_webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([track_cart_drive_pkg, "/launch", "/webots.launch.py"]))
    launch_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([track_cart_description_pkg, "/launch", "/rviz2.launch.py"]))

    ld = LaunchDescription()
    ld.add_action(launch_webots)
    ld.add_action(orb_slam3_octomap)
    ld.add_action(tf2_node)
    ld.add_action(TimerAction(period=6.0, actions=[launch_rviz2]))  # Delay 6 seconds before launch rviz2

    return ld
