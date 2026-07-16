import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from warehouse_robot_sim.layout_config import initial_poses, load_layout


def nav2_bringup(nav2_share, package_share, namespace, params_file, map_path):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',
            'map': map_path,
            'params_file': os.path.join(package_share, 'config', params_file),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )


def setup_navigation(context, *args, **kwargs):
    package_share = get_package_share_directory('warehouse_robot_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    layout_name = LaunchConfiguration('layout').perform(context)
    layout = load_layout(layout_name, Path(package_share))
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    robot_names = list(initial_poses(layout))
    if robot_count < 1 or robot_count > len(robot_names):
        raise ValueError(f'robot_count must be between 1 and {len(robot_names)}')

    robot_names = robot_names[:robot_count]
    map_override = LaunchConfiguration('map').perform(context)
    map_path = map_override or os.path.join(package_share, 'maps', layout['map'])

    actions = []
    for robot_name in robot_names:
        params_file = f'nav2_params_{robot_name}.yaml'
        actions.append(
            nav2_bringup(nav2_share, package_share, robot_name, params_file, map_path)
        )

    actions.append(
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
                        'layout': layout_name,
                        'robots': robot_names,
                    }],
                ),
            ],
        )
    )
    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='standard'),
        DeclareLaunchArgument('robot_count', default_value='3'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=setup_navigation),
    ])
