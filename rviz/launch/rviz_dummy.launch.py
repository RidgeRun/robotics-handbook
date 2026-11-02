from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the simple RViz-compatible dummy talker."""
    return LaunchDescription([
        Node(
            package='rviz_dummy',
            executable='talker',
            name='rviz_dummy_talker',
            output='screen',
        ),
    ])
