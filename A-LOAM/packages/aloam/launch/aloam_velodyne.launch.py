import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare(package="aloam").find(package_name="aloam")

    params_path = os.path.join(this_pkg, "config/params.yaml")

    scan_registration_node = Node(
        package="aloam", executable="aloam_scan_registration", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[params_path]
    )
    localization_node = Node(
        package="aloam", executable="aloam_localization", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    mapping_node = Node(
        package="aloam", executable="aloam_mapping", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[params_path]
    )
    system_node = Node(
        package="aloam", executable="aloam_system", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[params_path]
    )

    ld = LaunchDescription()
    # ld.add_action(scan_registration_node)
    # ld.add_action(localization_node)
    # ld.add_action(mapping_node)
    ld.add_action(system_node)

    return ld
