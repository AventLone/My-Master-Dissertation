import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare(package="aloam").find(package_name="aloam")

    params_path = os.path.join(this_pkg, "config/params.yaml")

    system_node = Node(
        package="aloam", executable="aloam_system", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[params_path]
    )

    ld = LaunchDescription()
    ld.add_action(system_node)

    return ld
