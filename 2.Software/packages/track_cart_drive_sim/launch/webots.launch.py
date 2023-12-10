import os
import launch
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg_share = FindPackageShare(package='track_cart_drive_sim').find('track_cart_drive_sim')
    robot_description_path = os.path.join(pkg_share, 'urdf', 'track_cart.urdf')

    webots = WebotsLauncher(world=os.path.join(pkg_share, 'worlds', 'wild_environment.wbt'))

    my_robot_driver = WebotsController(
        robot_name='track cart',
        parameters=[{'robot_description': robot_description_path}]
    )

    event_ = launch.actions.RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
        )
    )

    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(my_robot_driver)
    ld.add_action(event_)

    return ld
