from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the C++ talker and listener together."""
    return LaunchDescription([
        Node(
            package='helloworld_cpp',
            executable='talker',
            name='hello_world_talker',
            output='screen',
        ),
        Node(
            package='helloworld_cpp',
            executable='listener',
            name='hello_world_listener',
            output='screen',
        ),
    ])
