import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsen_camera_dir = FindPackageShare(package="realsense2_camera").find("realsense2_camera")

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", os.path.join(FindPackageShare("orb_slam3").find("orb_slam3"), "config", "rviz/orb_slam3.rviz")]
    )
    rgbd_node = Node(
        package="orb_slam3", executable="rgbd", name="rgbd_node", output="screen",
        arguments=["camera/color/image_raw", "camera/aligned_depth_to_color/image_raw"]
    )

    #------- Define launch actions ---------#
    launch_realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsen_camera_dir, "/launch", "/rs_launch.py"]),
        launch_arguments={
            'enable_sync': 'true',
            'align_depth.enable': 'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(launch_realsense_camera)
    ld.add_action(rgbd_node)
    ld.add_action(rviz2_node)

    return ld
