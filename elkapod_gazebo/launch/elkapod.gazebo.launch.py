from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the package name and URDF path (update accordingly)
    robot_description_package = 'elkapod_description'
    package_name = 'elkapod_gazebo'

    urdf_file_path = os.path.join(
        get_package_share_directory(robot_description_package),
        'urdf',
        'elkapod.urdf.xacro'
    )

    # Path to your SDF world file
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf'  # Make sure 'empty.sdf' exists in your package worlds directory
    )

    # Declare launch argument for the world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to the world SDF file to load into Gazebo'
    )

    # Gazebo Simulator
    gz_proc = ExecuteProcess(cmd=['gz', 'sim', world_file_path, '-v'], output='screen')

    # Robot State Publisher
    robot_description = Command(['xacro ', urdf_file_path])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(robot_description_package))),
        gz_proc,
        world_arg,
        robot_state_publisher,
        spawn_entity,
    ])
