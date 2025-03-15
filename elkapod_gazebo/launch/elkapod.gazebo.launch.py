from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_package = 'elkapod_description'
    elkapod_core = "elkapod_core_bringup"

    package_name = 'elkapod_gazebo'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(elkapod_core), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'Elkapod',
                                   '-z', '0.1'],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory(robot_description_package), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    joint_broad_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )])



    joint_position_controller_spawner = TimerAction(period=5.0, actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_position_controller"],
            )])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(robot_description_package))),
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        joint_position_controller_spawner,
        ros_gz_bridge,
    ])
