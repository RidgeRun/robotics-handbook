from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helloworld',
            executable='talker',
            name='hello_world_talker',
            output='screen',
        ),
        Node(
            package='helloworld',
            executable='listener',
            name='hello_world_listener',
            output='screen',
        ),
    ])
