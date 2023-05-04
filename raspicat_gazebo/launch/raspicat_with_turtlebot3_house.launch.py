import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('raspicat_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    world = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('raspicat_gazebo'),
        'worlds',
        'turtlebot3_house.world')
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "false" to run headless.'
    )

    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('raspicat_gazebo'),
            'worlds',
            'turtlebot3_house.world'),
        description='world configuration file path'
    )

    declare_x_position = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x position of robot')

    declare_y_position = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y position of robot')

    raspicat_with_emptyworld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'raspicat_with_emptyworld.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'world': world,
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_x_position)
    ld.add_action(declare_y_position)

    ld.add_action(raspicat_with_emptyworld)

    return ld
