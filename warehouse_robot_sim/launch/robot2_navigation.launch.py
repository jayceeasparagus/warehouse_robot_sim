import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('warehouse_robot_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(package_share, 'maps', 'clean_warehouse_map.yaml')
    default_params = os.path.join(package_share, 'config', 'nav2_params_robot2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': 'robot2',
                'use_namespace': 'true',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])
