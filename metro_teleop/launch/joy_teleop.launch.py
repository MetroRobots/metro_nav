from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'joy_params_path',
        default_value='',
        description='The path to a parameters file for the metro_teleop/joy_teleop node'))

    ld.add_action(Node(
        name='metro_teleop',
        package='metro_teleop',
        executable='joy_teleop',
        parameters=[
            LaunchConfiguration('joy_params_path'),
        ]
    ))
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
    ))

    return ld
