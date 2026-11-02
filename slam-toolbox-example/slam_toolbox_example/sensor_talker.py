"""Publish a synthetic laser scan and odometry stream for SLAM Toolbox.

The node acts as the "talker" at the beginning of the demo pipeline. It
publishes a 360-degree scan on ``/scan`` and a deterministic circular robot
trajectory through ``/odom`` and TF. Two large circular obstacles rotate
around the robot, making successive scans and maps vary visibly. A static
transform places the ``laser`` sensor frame above the robot base.

This is intentionally a deterministic sensor source: it does not simulate a
robot controller or a physics engine. Its purpose is to provide the minimum
ROS interfaces that an online SLAM node needs to consume changing laser data.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class SyntheticSensorTalker(Node):
    """Publish the scan, odometry, and TF inputs consumed by SLAM Toolbox.

    The synthetic environment is a square room centered on the origin. The
    robot drives a small circle while two large circular obstacles rotate
    around it. This produces changing scans and accepted SLAM poses without
    requiring Gazebo or a physics simulation.

    Attributes:
        scan_publisher: Publisher for ``sensor_msgs/msg/LaserScan`` on
            ``/scan``.
        odom_publisher: Publisher for ``nav_msgs/msg/Odometry`` on ``/odom``.
        tf_broadcaster: Dynamic broadcaster for ``odom`` to
            ``base_footprint``.
        static_tf_broadcaster: Static broadcaster for ``base_footprint`` to
            ``laser``.
        timer: Timer that publishes complete sensor updates at the configured
            period after the optional startup delay.
    """

    ROOM_HALF_SIZE = 3.0
    LASER_HEIGHT = 0.15
    NUMBER_OF_RAYS = 360
    OBSTACLE_COUNT = 2
    OBSTACLE_ORBIT_RADIUS = 1.25
    OBSTACLE_RADIUS = 0.50

    def __init__(self):
        """Initialize publishers, TF broadcasters, and the sensor timer."""
        super().__init__('synthetic_sensor_talker')

        # Publish the same interfaces normally produced by a robot laser and
        # odometry stack. Absolute names make the demo's topic graph explicit.
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # The obstacle pattern completes one rotation over this period. It is
        # intentionally not synchronized with the tenth-map dump interval.
        # A ROS parameter makes it easy to create slower or faster changes.
        self.declare_parameter('pattern_period', 37.0)
        self.pattern_period = self.get_parameter('pattern_period').value
        if self.pattern_period <= 0.0:
            raise ValueError('pattern_period must be greater than zero')
        self.declare_parameter('motion_period', 30.0)
        self.declare_parameter('motion_radius', 0.8)
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('start_delay', 1.0)
        self.motion_period = self.get_parameter('motion_period').value
        self.motion_radius = self.get_parameter('motion_radius').value
        self.publish_period = self.get_parameter('publish_period').value
        self.start_delay = self.get_parameter('start_delay').value
        if self.motion_period <= 0.0:
            raise ValueError('motion_period must be greater than zero')
        if self.motion_radius <= 0.0:
            raise ValueError('motion_radius must be greater than zero')
        if self.publish_period <= 0.0 or self.start_delay < 0.0:
            raise ValueError('publish_period must be positive and start_delay non-negative')
        self.update_count = 0
        self.robot_pose = (0.0, 0.0, 0.0)

        # SLAM Toolbox needs a live odom -> base transform and a stable sensor
        # frame transform in order to place each scan in the map frame.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_sensor_transform()

        # Delay the first update so all consumers can subscribe before the
        # deterministic sequence begins. This makes separate runs comparable.
        self.timer = None
        self.start_timer = self.create_timer(self.start_delay, self.start_publishing)

    def start_publishing(self):
        """Start the deterministic sensor sequence exactly once."""
        self.start_timer.cancel()
        self.publish_sensor_update()
        self.timer = self.create_timer(self.publish_period, self.publish_sensor_update)

    def publish_static_sensor_transform(self):
        """Publish the fixed ``base_footprint`` to ``laser`` transform."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'laser'

        # The laser is centered above the robot base and has no rotation.
        transform.transform.translation.z = self.LASER_HEIGHT
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def publish_sensor_update(self):
        """Publish one scan, odometry message, and odometry TF transform."""
        # Reuse one timestamp so the scan, odometry, and TF describe the same
        # instant to consumers such as SLAM Toolbox.
        stamp = self.get_clock().now().to_msg()
        motion_phase = (
            2.0 * math.pi * self.update_count / (10.0 * self.motion_period)
        )
        # Move around a circle and point tangentially along the trajectory.
        self.robot_pose = (
            self.motion_radius * math.cos(motion_phase),
            self.motion_radius * math.sin(motion_phase),
            motion_phase + math.pi / 2.0,
        )
        self.publish_scan(stamp)
        self.publish_odometry(stamp, self.robot_pose)
        self.publish_odom_transform(stamp, self.robot_pose)
        self.update_count += 1

    def publish_scan(self, stamp):
        """Publish a 360-degree scan generated from the square room walls.

        Args:
            stamp: ROS timestamp shared with the other sensor messages in this
                update.
        """
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.NUMBER_OF_RAYS
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 10.0

        # The phase changes deterministically with each update. Every ray
        # intersects either the room boundary or one of the moving obstacles.
        phase = (
            2.0 * math.pi * self.update_count / (10.0 * self.pattern_period)
        )
        scan.ranges = [
            self.distance_to_pattern(
                scan.angle_min + index * scan.angle_increment,
                phase,
            )
            for index in range(self.NUMBER_OF_RAYS)
        ]
        self.scan_publisher.publish(scan)

    def distance_to_pattern(self, angle, phase):
        """Return the nearest room-wall or moving-obstacle intersection.

        Args:
            angle: Ray direction in radians, measured in the laser frame.
            phase: Current angular phase of the obstacle pattern.

        Returns:
            The positive distance to the first intersection.
        """
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        distances = []

        # For each axis, calculate where the ray reaches either positive or
        # negative room boundary, then keep only forward-facing intersections.
        if abs(direction_x) > 1e-9:
            distances.append(self.ROOM_HALF_SIZE / abs(direction_x))
        if abs(direction_y) > 1e-9:
            distances.append(self.ROOM_HALF_SIZE / abs(direction_y))

        # Place two large circular obstacles opposite each other on the orbit.
        # Their size makes the changing scan pattern obvious in RViz and in
        # successive dumped occupancy maps.
        for obstacle_index in range(self.OBSTACLE_COUNT):
            obstacle_angle = (
                phase
                + obstacle_index * 2.0 * math.pi / self.OBSTACLE_COUNT
            )
            center_x = self.OBSTACLE_ORBIT_RADIUS * math.cos(obstacle_angle)
            center_y = self.OBSTACLE_ORBIT_RADIUS * math.sin(obstacle_angle)
            obstacle_distance = self.distance_to_circle(
                direction_x,
                direction_y,
                center_x,
                center_y,
                self.OBSTACLE_RADIUS,
            )
            if obstacle_distance is not None:
                distances.append(obstacle_distance)

        return min(distances)

    @staticmethod
    def distance_to_circle(direction_x, direction_y, center_x, center_y, radius):
        """Return the nearest forward ray/circle intersection, if one exists.

        The ray starts at the sensor origin and has unit direction
        ``(direction_x, direction_y)``. Solving the quadratic intersection
        equation provides a lightweight geometric obstacle model without a
        physics engine.
        """
        projection = direction_x * center_x + direction_y * center_y
        center_distance_squared = center_x ** 2 + center_y ** 2
        discriminant = (
            projection ** 2 - center_distance_squared + radius ** 2
        )
        if discriminant < 0.0:
            return None

        intersection = projection - math.sqrt(discriminant)
        return intersection if intersection > 0.0 else None

    def publish_odometry(self, stamp, pose):
        """Publish the synthetic robot pose along its circular trajectory.

        Args:
            stamp: ROS timestamp shared with this sensor update.
            pose: ``(x, y, yaw)`` pose in the odometry frame.
        """
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_footprint'
        odometry.pose.pose.position.x = pose[0]
        odometry.pose.pose.position.y = pose[1]
        odometry.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        odometry.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        self.odom_publisher.publish(odometry)

    def publish_odom_transform(self, stamp, pose):
        """Broadcast the moving ``odom`` to ``base_footprint`` transform.

        Args:
            stamp: ROS timestamp shared with this sensor update.
            pose: ``(x, y, yaw)`` pose in the odometry frame.
        """
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = pose[0]
        transform.transform.translation.y = pose[1]
        transform.transform.rotation.z = math.sin(pose[2] / 2.0)
        transform.transform.rotation.w = math.cos(pose[2] / 2.0)
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Run the synthetic sensor talker until interrupted."""
    rclpy.init(args=args)
    node = SyntheticSensorTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
