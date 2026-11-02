from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch import LaunchDescription
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch synthetic inputs, SLAM Toolbox, and the map listener."""
    dump_every_n_maps = LaunchConfiguration('dump_every_n_maps')
    dump_directory = LaunchConfiguration('dump_directory')
    pattern_period = LaunchConfiguration('pattern_period')
    motion_period = LaunchConfiguration('motion_period')
    motion_radius = LaunchConfiguration('motion_radius')
    publish_period = LaunchConfiguration('publish_period')
    start_delay = LaunchConfiguration('start_delay')

    # Resolve the installed config path instead of relying on the terminal's
    # current working directory when this launch file is run.
    slam_config = PathJoinSubstitution([
        FindPackageShare('slam_toolbox_example'),
        'config',
        'slam_toolbox.yaml',
    ])

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            slam_config,
            # The synthetic sensor uses wall-clock timestamps, not /clock.
            {'use_sim_time': False, 'use_lifecycle_manager': False},
        ],
        output='screen',
    )

    # Once configuration succeeds, move the lifecycle node to active so it
    # subscribes to /scan and publishes /map.
    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # The lifecycle node starts unconfigured; explicitly request the
    # configure transition after the launch system has registered the handler.
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dump_every_n_maps',
            default_value='10',
            description='Save every Nth received map update.',
        ),
        DeclareLaunchArgument(
            'dump_directory',
            default_value='maps',
            description='Directory for dumped PGM and YAML map files.',
        ),
        DeclareLaunchArgument('pattern_period', default_value='37.0'),
        DeclareLaunchArgument('motion_period', default_value='30.0'),
        DeclareLaunchArgument('motion_radius', default_value='0.8'),
        DeclareLaunchArgument('publish_period', default_value='0.1'),
        DeclareLaunchArgument(
            'start_delay',
            default_value='1.0',
            description='Seconds to wait before the first deterministic scan.',
        ),
        # The talker is the synthetic robot/sensor source at the front of the
        # pipeline. It provides LaserScan, odometry, and TF data.
        Node(
            package='slam_toolbox_example',
            executable='sensor_talker',
            name='synthetic_sensor_talker',
            parameters=[{
                'pattern_period': pattern_period,
                'motion_period': motion_period,
                'motion_radius': motion_radius,
                'publish_period': publish_period,
                'start_delay': start_delay,
            }],
            output='screen',
        ),
        # SLAM Toolbox consumes /scan and the TF tree, then publishes /map and
        # the map -> odom transform.
        slam_node,
        activate_after_configure,
        configure_slam,
        # The listener is at the end of the pipeline and confirms that a map
        # was produced by the middle node.
        Node(
            package='slam_toolbox_example',
            executable='map_listener',
            name='map_listener',
            parameters=[
                {
                    'dump_every_n_maps': dump_every_n_maps,
                    'dump_directory': dump_directory,
                }
            ],
            output='screen',
        ),
    ])
