from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        name='metro_teleop',
        package='metro_teleop',
        executable='keyboard_teleop',
        parameters=[
            [FindPackageShare('metro_teleop'), '/config/key_config.yaml']
        ],
        prefix='xterm -e',

    ))
    return ld
