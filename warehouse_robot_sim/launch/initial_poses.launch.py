from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('publish_count', default_value='8'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        Node(
            package='warehouse_robot_sim',
            executable='initial_pose_publisher_node',
            name='initial_pose_publisher_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'start_delay_sec': LaunchConfiguration('start_delay_sec'),
                'publish_count': LaunchConfiguration('publish_count'),
                'publish_period_sec': LaunchConfiguration('publish_period_sec'),
            }],
        ),
    ])
