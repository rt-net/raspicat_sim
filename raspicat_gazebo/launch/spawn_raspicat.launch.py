from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    declare_x_position = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x position of robot')

    declare_y_position = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y position of robot')

    gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'raspicat',
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.0'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_x_position)
    ld.add_action(declare_y_position)

    ld.add_action(gazebo_ros_spawner)

    return ld
