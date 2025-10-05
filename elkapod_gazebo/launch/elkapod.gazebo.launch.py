from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# WORLDS = {
#     'bookstore': 0,
#     'factory': 0,
#     'office': 0,
#     'small_house': 0
# }

# def launch_setup(context: LaunchContext, *args, **kwargs):
#     pass

# def generate_launch_description():
#     robot_description_package = 'elkapod_description'
#     elkapod_core = "elkapod_core_bringup"
#     package_name = 'elkapod_gazebo'

#     worlds_directory = os.path.join(
#         get_package_share_directory(package_name),
#         'worlds'
#     )
#     avaliable_worlds = {world for world in next(os.walk(worlds_directory))[1]}
#     avaliable_worlds.add('empty')

#     default_world = os.path.join(
#         get_package_share_directory(package_name),
#         'worlds',
#         'empty.world'
#     )
#     world_arg = DeclareLaunchArgument(
#         'world',
#         default_value=default_world,
#         description='World to load'
#     )

#     world_config = LaunchConfiguration('world')
#     print(world_config)
#     print([method_name for method_name in dir(world_config)
#            if callable(getattr(world_config, method_name))])
#     print(str(world_config))


#     gazebo_share = os.path.join(
#         os.path.dirname(get_package_share_directory('elkapod_gazebo')),
#         'elkapod_gazebo', 'models'
#     )

#     world = default_world

#     if world_config != default_world or world_config != 'empty':
#         world_dir = os.path.join(
#             worlds_directory,
#             world_config
#         )
#         gazebo_share = os.path.join(
#             world_dir,
#             'models'
#         )
#         world = os.path.join(
#             world_dir,
#             world_config
#         ) + '.world'

#     print(
#         f"SANITY CHECK\n {world_config}\n {world_arg}, \n {world},\n {gazebo_share}")
#     desc_share = os.path.dirname(
#         get_package_share_directory('elkapod_description'))

#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory(
#                 elkapod_core), 'launch', 'rsp.launch.py'
#         )]), launch_arguments={'sim_mode': 'true'}.items()
#     )

#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
#         launch_arguments={'gz_args': [
#             '-r -v4 ', world], 'on_exit_shutdown': 'true', "emulate_tty": 'true'}.items()
#     )

#     spawn_entity = Node(package='ros_gz_sim', executable='create',
#                         arguments=['-topic', 'robot_description',
#                                    '-name', 'Elkapod',
#                                    '-z', '0.1'],
#                         output='screen',
#                         emulate_tty=True
#                         )

#     bridge_params = os.path.join(get_package_share_directory(
#         robot_description_package), 'config', 'gz_bridge.yaml')
#     ros_gz_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             '--ros-args',
#             '-p',
#             f'config_file:={bridge_params}',
#         ],
#         output='screen',
#         emulate_tty=True
#     )

#     joint_broad_spawner = TimerAction(period=5.0, actions=[Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#         output='screen',
#         emulate_tty=True
#     )])

#     elkapod_ik_controller_spawner = TimerAction(period=8.0, actions=[Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["elkapod_ik_controller"],
#         output='screen',
#         emulate_tty=True
#     )])

#     gz_resource = f"{desc_share}:{gazebo_share}"

#     return LaunchDescription([
#         SetEnvironmentVariable(
#             name='GZ_SIM_RESOURCE_PATH',
#             value=gz_resource
#         ),
#         rsp,
#         world_arg,
#         gazebo,
#         spawn_entity,
#         joint_broad_spawner,
#         elkapod_ik_controller_spawner,
#         ros_gz_bridge,
#     ])


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_description_package = 'elkapod_description'
    elkapod_core = "elkapod_core_bringup"
    package_name = 'elkapod_gazebo'

    worlds_directory = os.path.join(
        get_package_share_directory(package_name),
        'worlds'
    )
    avaliable_worlds = {world for world in next(os.walk(worlds_directory))[1]}
    avaliable_worlds.add('empty')

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world_config = LaunchConfiguration('world').perform(context)

    gazebo_models_share = os.path.join(
        os.path.dirname(get_package_share_directory('elkapod_gazebo')),
        'elkapod_gazebo', 'models'
    )

    world = default_world
    world_dir = ''
    media = ''
    if world_config != default_world or world_config != 'empty':
        world_dir = os.path.join(
            worlds_directory,
            world_config
        )
        gazebo_models_share = os.path.join(
            world_dir,
            'models'
        )
        world = os.path.join(
            world_dir,
            world_config
        ) + '.world'
        media = os.path.join(world_dir, 'media')

    desc_share = os.path.dirname(
        get_package_share_directory('elkapod_description'))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                elkapod_core), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            '-r -v4 ', world], 'on_exit_shutdown': 'true', "emulate_tty": 'true'}.items()
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

    gz_resource = f"{desc_share}:{world_dir}:{media}:{gazebo_models_share}"
    gz_environment = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource
    )
    print('\n'.join(gz_resource.split(':')))
    # print(gz_resource)
    # os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource
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
        OpaqueFunction(function=launch_setup),
    ])
