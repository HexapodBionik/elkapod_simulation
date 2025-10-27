from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_description_package = 'elkapod_description'
    elkapod_core = "elkapod_core_bringup"
    package_name = 'elkapod_gazebo'

    worlds_directory = os.path.join(
        get_package_share_directory(package_name),
        'worlds'
    )
    avaliable_worlds = {world for world in next(os.walk(worlds_directory))[1]}

    world_name = LaunchConfiguration('world').perform(context)

    gazebo_models_share = os.path.join(
        os.path.dirname(get_package_share_directory('elkapod_gazebo')),
        'elkapod_gazebo', 'models'
    )

    world_file = None
    world_dir = ''
    media_dir = ''
    if world_name in avaliable_worlds:
        world_dir = os.path.join(
            worlds_directory,
            world_name
        )
        gazebo_models_share = os.path.join(
            world_dir,
            'models'
        )
        world_file = os.path.join(
            world_dir,
            world_name
        ) + '.world'

        media_dir = os.path.join(world_dir, 'media')
    else:
        raise ValueError(f"World named {world_name} doesn't exist")

    desc_share = os.path.dirname(
        get_package_share_directory('elkapod_description'))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                elkapod_core), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': 'true'}.items()
    )
    avaliable_worlds = {world for world in next(os.walk(worlds_directory))[1]}
    avaliable_worlds.add('empty')

    gazebo_headless_mode = LaunchConfiguration('headless').perform(context)
    print(gazebo_headless_mode, type(gazebo_headless_mode))
    gazebo_headless_mode = gazebo_headless_mode == 'true' or gazebo_headless_mode == "True"

    gazebo_mode = '-s ' if gazebo_headless_mode else ''

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            '-r -v4 ' + gazebo_mode, world_file], 'on_exit_shutdown': 'true', "emulate_tty": 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'Elkapod',
                                   '-z', '0.1'],
                        output='screen',
                        emulate_tty=True
                        )

    bridge_params = os.path.join(get_package_share_directory(
        robot_description_package), 'config', 'gz_bridge.yaml')
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

    joint_broad_spawner = TimerAction(period=10.0, actions=[Node(
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

    # gz_resource = f"{desc_share}:{world_dir}:{media_dir}:{gazebo_models_share}"
    gz_resource = f"{desc_share}:{world_dir}:{gazebo_models_share}"

    gz_environment = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource
    )
    return [
        gz_environment,
        rsp,
        spawn_entity,
        joint_broad_spawner,
        elkapod_ik_controller_spawner,
        gazebo,
        ros_gz_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World to load'
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='To open gazebo sim in headless mode, less ressource demanding'
        ),

        OpaqueFunction(function=launch_setup),
    ])
