from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare('metro_nav_demo_utils')
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('map_yaml'))

    ld.add_action(DeclareLaunchArgument('inflate', default_value='false',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('multi', default_value='false',
                                        choices=['true', 'false']))

    config_path_base = PathJoinSubstitution([pkg_path, 'config', 'global_plan_'])

    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            [config_path_base, 'base.yaml'],
            [config_path_base, 'inflate_', LaunchConfiguration('inflate'), '.yaml'],
            [config_path_base, 'multi_', LaunchConfiguration('multi'), '.yaml'],
        ],
    ))

    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    ))

    ld.add_action(Node(
        package='metro_nav_demo_utils',
        executable='global_planner',
        output='screen',
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server', 'planner_server']}]))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'rviz', 'global.rviz'])],
        output='screen'
    ))

    return ld
