import os
# import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    this_pkg = FindPackageShare(package='webots_simulation').find('webots_simulation')
    robot_description_path = os.path.join(this_pkg, 'urdf', 'robot.urdf')

    # webots = WebotsLauncher(world=os.path.join(this_pkg, 'worlds', 'indoor.wbt'))
    webots = WebotsLauncher(world=os.path.join(this_pkg, 'worlds', 'wild_with_grass.wbt'))
    # webots = WebotsLauncher(world=os.path.join(this_pkg, 'worlds', 'wild_without_grass_1.wbt'))

    my_robot_driver = WebotsController(
        robot_name='BD_Roamer',
        parameters=[{'robot_description': robot_description_path}]
    )

    # event_ = launch.actions.RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=webots,
    #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
    #     )
    # )

    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(TimerAction(period=1.0, actions=[my_robot_driver]))
    # ld.add_action(event_)

    return ld
