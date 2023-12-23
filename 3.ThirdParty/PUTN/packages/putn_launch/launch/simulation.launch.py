import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction


def generate_launch_description():
    gpr_pkg = FindPackageShare(package="gpr").find("gpr")
    urdf_path = os.path.join(gpr_pkg, "config/hyperparam.txt")

    waypoint_generator_node = Node(
        package="waypoint_generating", executable="waypoint_generator", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    global_planning_node = Node(
        package="global_planning", executable="global_planner", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    gpr_node = Node(
        package="gpr", executable="gpr_path", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{"ConfigFilePath", urdf_path}]
    )
    local_planning_node = Node(
        package="local_planning", executable="local_planner", output="screen", emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld = LaunchDescription()
    ld.add_action(waypoint_generator_node)
    ld.add_action(global_planning_node)
    ld.add_action(gpr_node)
    ld.add_action(local_planning_node)
    return ld
