from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_package = 'elkapod_description'
    elkapod_core = "elkapod_core_bringup"
    package_name = 'elkapod_gazebo'

    desc_share = os.path.dirname(get_package_share_directory('elkapod_description'))
    gazebo_share = os.path.join(
        os.path.dirname(get_package_share_directory('elkapod_gazebo')),
        'elkapod_gazebo', 'models'
    )

    gz_resource = f"{desc_share}:{gazebo_share}"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(elkapod_core), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': 'true'}.items()
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
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true', "emulate_tty": 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'Elkapod',
                                   '-z', '0.1'],
                        output='screen',
                        emulate_tty=True
                        )

    bridge_params = os.path.join(get_package_share_directory(robot_description_package), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        emulate_tty=True
    )

    joint_broad_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen',
        emulate_tty=True
    )])

    elkapod_ik_controller_spawner = TimerAction(period=8.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["elkapod_ik_controller"],
        output='screen',
        emulate_tty=True
    )])

    joint_position_controller_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["passthrough_controller"],
        output='screen',
        emulate_tty=True
    )])

    return LaunchDescription([
            SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource
        ),
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        joint_position_controller_spawner,
        elkapod_ik_controller_spawner,
        ros_gz_bridge,
    ])
