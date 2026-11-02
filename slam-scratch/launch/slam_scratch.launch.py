"""Launch the existing synthetic talker/listener around scratch SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the talker -> scratch SLAM -> listener demonstration."""
    return LaunchDescription([
        DeclareLaunchArgument('pattern_period', default_value='37.0'),
        DeclareLaunchArgument('motion_period', default_value='30.0'),
        DeclareLaunchArgument('motion_radius', default_value='0.8'),
        DeclareLaunchArgument('publish_period', default_value='0.1'),
        DeclareLaunchArgument('start_delay', default_value='1.0'),
        DeclareLaunchArgument('map_update_period', default_value='2.0'),
        DeclareLaunchArgument('dump_every_n_maps', default_value='10'),
        DeclareLaunchArgument('dump_directory', default_value='slam_maps_live'),
        Node(
            package='slam_scratch',
            executable='sensor_talker',
            name='synthetic_sensor_talker',
            parameters=[{
                'pattern_period': LaunchConfiguration('pattern_period'),
                'motion_period': LaunchConfiguration('motion_period'),
                'motion_radius': LaunchConfiguration('motion_radius'),
                'publish_period': LaunchConfiguration('publish_period'),
                'start_delay': LaunchConfiguration('start_delay'),
            }],
            output='screen',
        ),
        Node(
            package='slam_scratch',
            executable='scratch_slam',
            name='scratch_slam',
            parameters=[{
                'publish_period': LaunchConfiguration('map_update_period'),
            }],
            output='screen',
        ),
        Node(
            package='slam_scratch',
            executable='map_listener',
            name='map_listener',
            parameters=[{
                'dump_every_n_maps': LaunchConfiguration('dump_every_n_maps'),
                'dump_directory': LaunchConfiguration('dump_directory'),
            }],
            output='screen',
        ),
    ])
