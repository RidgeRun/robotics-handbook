"""Publish simple demo data directly to the topics used by RViz.

The node publishes a small occupancy grid on ``/map``, a circular laser scan
on ``/scan``, and the minimal ``map -> odom -> base_footprint -> laser`` TF
tree. The topic names and frames match ``rviz/slam_demo.rviz`` in this
repository, so RViz can display the demo without Gazebo or SLAM Toolbox.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class RvizDummyTalker(Node):
    """Publish a static map and a simple synthetic scan for RViz."""

    def __init__(self):
        """Create RViz publishers, TF broadcasters, and the update timer."""
        super().__init__('rviz_dummy_talker')

        # RViz's Map display requests transient-local data so it can receive
        # the last map even when it starts after this node.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        # These transforms let RViz convert the laser points into its fixed
        # `map` frame. All transforms are identity transforms in this demo.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_laser_transform()

        # Publish the map once immediately and refresh the scan at 10 Hz.
        self.publish_map()
        self.timer = self.create_timer(0.1, self.publish_scan_and_tf)

    def publish_map(self):
        """Publish a 6 m square occupancy grid with four occupied walls."""
        map_message = OccupancyGrid()
        map_message.header.frame_id = 'map'
        map_message.header.stamp = self.get_clock().now().to_msg()
        map_message.info.resolution = 0.05
        map_message.info.width = 120
        map_message.info.height = 120
        map_message.info.origin.position.x = -3.0
        map_message.info.origin.position.y = -3.0
        map_message.info.origin.orientation.w = 1.0

        # Draw a one-cell occupied border around the otherwise free map.
        width = map_message.info.width
        height = map_message.info.height
        map_message.data = [
            100 if row in (0, height - 1) or column in (0, width - 1)
            else 0
            for row in range(height)
            for column in range(width)
        ]
        self.map_publisher.publish(map_message)

    def publish_scan_and_tf(self):
        """Publish the circular scan and the dynamic identity transforms."""
        stamp = self.get_clock().now().to_msg()
        self.publish_scan(stamp)
        self.publish_map_to_base_transform(stamp)

    def publish_scan(self, stamp):
        """Publish 360 ranges that form a visible circular obstacle."""
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [2.0] * 360
        self.scan_publisher.publish(scan)

    def publish_map_to_base_transform(self, stamp):
        """Publish identity ``map -> odom`` and ``odom -> base_footprint`` TF."""
        map_to_odom = self.make_identity_transform(stamp, 'map', 'odom')
        odom_to_base = self.make_identity_transform(stamp, 'odom', 'base_footprint')
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

    def publish_static_laser_transform(self):
        """Publish the fixed ``base_footprint -> laser`` transform."""
        transform = self.make_identity_transform(
            self.get_clock().now().to_msg(), 'base_footprint', 'laser'
        )
        transform.transform.translation.z = 0.15
        self.static_tf_broadcaster.sendTransform(transform)

    @staticmethod
    def make_identity_transform(stamp, parent, child):
        """Create an identity transform between two named frames.

        Args:
            stamp: ROS timestamp for the transform.
            parent: Parent TF frame name.
            child: Child TF frame name.

        Returns:
            A ``TransformStamped`` with zero translation and identity rotation.
        """
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform


def main(args=None):
    """Run the RViz dummy talker until interrupted."""
    rclpy.init(args=args)
    node = RvizDummyTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
