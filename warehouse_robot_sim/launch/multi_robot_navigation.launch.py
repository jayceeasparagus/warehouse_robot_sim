import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nav2_bringup(nav2_share, package_share, namespace, params_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(package_share, 'config', params_file),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )


def generate_launch_description():
    package_share = get_package_share_directory('warehouse_robot_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    default_map = os.path.join(package_share, 'maps', 'clean_warehouse_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        nav2_bringup(nav2_share, package_share, 'robot1', 'nav2_params_robot1.yaml'),
        nav2_bringup(nav2_share, package_share, 'robot2', 'nav2_params_robot2.yaml'),
        nav2_bringup(nav2_share, package_share, 'robot3', 'nav2_params_robot3.yaml'),
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='warehouse_robot_sim',
                    executable='initial_pose_publisher_node',
                    name='initial_pose_publisher_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'start_delay_sec': 0.0,
                        'publish_count': 15,
                        'publish_period_sec': 0.5,
                        'require_subscribers': True,
                    }],
                ),
            ],
        ),
    ])
