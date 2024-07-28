import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory("elkapod_sim")
    robot_description_path = os.path.join(package_dir, "resource", "my_robot.urdf")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="flat_world.wbt",
            description="Choose one of the world files from `/webots_ros2_universal_robot/worlds` directory",
        )
    )

    world = LaunchConfiguration("world")

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
    )

    my_robot_driver = WebotsController(
        robot_name="Elkapod",
        parameters=[
            {"robot_description": robot_description_path},
            {"use_sim_time": True},
        ],
    )

    ld.add_action(webots)
    ld.add_action(my_robot_driver)
    ld.add_action(
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    )

    return ld
