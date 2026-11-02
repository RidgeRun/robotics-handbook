"""Educational online 2-D SLAM implementation.

The node follows the classic scan-to-map SLAM decomposition:

1. Odometry supplies a predicted robot pose.
2. A laser scan is associated with the closest odometry timestamp.
3. Correlative scan matching searches a local 3-D pose lattice
   ``(x, y, yaw)`` around that prediction.
4. The selected pose is integrated into a sparse log-odds occupancy grid.
5. Keyframes create an odometry pose graph. Spatially close historical
   keyframes can add loop-closure constraints.
6. A lightweight relaxation step distributes loop-closure error through the
   graph and rebuilds the map from the corrected keyframe poses.

The ROS data flow is:

    /scan + /odom -> timestamp association -> pose prediction
                                      -> scan matching -> map update
                                      -> /map and map -> odom TF

This is intentionally readable rather than production-grade. It omits many
SLAM Toolbox details, including continuous-time TF interpolation, robust
optimization, probabilistic data association, sophisticated scan filtering,
and dynamic map resizing.
"""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class ScratchSlam(Node):
    """Build and publish an occupancy grid from live scan and odometry data.

    Coordinate-frame convention:

    * ``odom_pose`` is the robot pose in the odometry frame.
    * ``map_to_odom`` is the estimated transform from ``map`` to ``odom``.
    * ``map_pose = map_to_odom ∘ odom_pose`` is the pose used to project a
      laser scan into the map.

    A pose is represented internally as ``(x, y, yaw)``. Composition follows
    planar rigid-body-transform algebra. The occupancy grid is stored sparsely
    as ``{(cell_x, cell_y): log_odds}``; cells absent from the dictionary are
    unknown.

    Parameters:
        resolution: Map cell size in metres.
        width: Number of cells along the x axis.
        height: Number of cells along the y axis.
        origin_x: World x coordinate of map cell (0, 0).
        origin_y: World y coordinate of map cell (0, 0).
        ray_limit: Maximum range integrated from each laser ray.
        occupied_update: Positive log-odds update for a measured return.
        free_update: Negative log-odds update along free ray cells.
        publish_period: Maximum map publication period in seconds.
        scan_match_translation: Search radius around the odometry-predicted
            pose, in metres.
        scan_match_rotation: Search radius around the predicted yaw, radians.
        scan_match_min_score: Minimum occupied-cell score required before a
            scan-match correction is accepted.
        keyframe_distance: Translation needed before creating a keyframe.
        keyframe_rotation: Rotation needed before creating a keyframe.
        loop_closure_distance: Maximum pose distance for loop candidates.

    Published topics:
        ``/map``: The current ``nav_msgs/msg/OccupancyGrid``.
        ``/tf``: The estimated ``map -> odom`` transform.

    Subscribed topics:
        ``/scan``: Laser ranges and angular metadata.
        ``/odom``: Timestamped odometry pose used as the motion prior.
    """

    def __init__(self):
        """Create the ROS interfaces and initialize the SLAM state.

        The node intentionally keeps its state in ordinary Python structures
        so each stage can be inspected in a debugger or replaced in an
        experiment. No external mapping or optimization library is used.
        """
        super().__init__('scratch_slam')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 240)
        self.declare_parameter('height', 240)
        self.declare_parameter('origin_x', -6.0)
        self.declare_parameter('origin_y', -6.0)
        self.declare_parameter('ray_limit', 10.0)
        self.declare_parameter('occupied_update', 3.0)
        self.declare_parameter('free_update', -2.0)
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('scan_match_translation', 0.30)
        self.declare_parameter('scan_match_rotation', 0.30)
        self.declare_parameter('scan_match_min_score', 1.0)
        self.declare_parameter('keyframe_distance', 0.10)
        self.declare_parameter('keyframe_rotation', 0.10)
        self.declare_parameter('loop_closure_distance', 0.80)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.ray_limit = float(self.get_parameter('ray_limit').value)
        self.occupied_update = float(self.get_parameter('occupied_update').value)
        self.free_update = float(self.get_parameter('free_update').value)
        publish_period = float(self.get_parameter('publish_period').value)
        self.scan_match_translation = float(
            self.get_parameter('scan_match_translation').value
        )
        self.scan_match_rotation = float(
            self.get_parameter('scan_match_rotation').value
        )
        self.scan_match_min_score = float(
            self.get_parameter('scan_match_min_score').value
        )
        self.keyframe_distance = float(self.get_parameter('keyframe_distance').value)
        self.keyframe_rotation = float(self.get_parameter('keyframe_rotation').value)
        self.loop_closure_distance = float(
            self.get_parameter('loop_closure_distance').value
        )
        if self.resolution <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError('map dimensions and resolution must be positive')
        if self.ray_limit <= 0 or publish_period <= 0:
            raise ValueError('ray_limit and publish_period must be positive')

        # Sparse storage keeps unknown cells cheap while the map grows. A
        # value of zero means an uninformative prior; positive values support
        # occupancy and negative values support free space.
        self.log_odds = {}
        self.odom_history = deque(maxlen=200)
        self.pending_scans = deque(maxlen=20)
        # The first pose is anchored at the origin. Future scan matching can
        # change this transform while preserving the odometry frame.
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.keyframes = []
        self.constraints = []
        self.map_sequence = 0

        # TRANSIENT_LOCAL makes a late RViz subscriber receive the latest map,
        # which is the usual behavior expected from a map publisher.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', map_qos
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_timer = self.create_timer(publish_period, self.publish_map)

        self.get_logger().info(
            f'Live scratch mapper ready: {self.width}x{self.height} '
            f'at {self.resolution:.3f} m/cell'
        )

    def on_odom(self, message):
        """Cache one odometry pose for later timestamp association.

        Args:
            message: ``nav_msgs/msg/Odometry`` containing a pose in ``odom``.

        The callback does not integrate the pose immediately. Laser callbacks
        and odometry callbacks can be scheduled in either order, so the pose
        is stored in a bounded history and selected when the scan is processed.
        """
        stamp = self.stamp_to_seconds(message.header.stamp)
        pose = message.pose.pose
        self.odom_history.append((
            stamp,
            (
                pose.position.x,
                pose.position.y,
                self.quaternion_to_yaw(pose.orientation),
            ),
        ))

    def on_scan(self, scan):
        """Queue a scan until its timestamped odometry is available.

        Args:
            scan: ``sensor_msgs/msg/LaserScan`` to be mapped.

        Queueing avoids assuming that the publisher order is the callback
        order. The timer callback later processes scans after odometry has had
        an opportunity to arrive.
        """
        self.pending_scans.append(scan)

    def process_pending_scans(self):
        """Run the prediction, matching, and mapping stages for queued scans.

        For each scan, the algorithm computes:

        ``predicted_map_pose = map_to_odom ∘ odom_pose``

        The scan matcher may correct this pose. Keyframe selection determines
        whether the corrected observation becomes part of the pose graph and
        persistent map.
        """
        while self.pending_scans and self.odom_history:
            scan = self.pending_scans.popleft()
            odom_pose = self.closest_odom_pose(
                self.stamp_to_seconds(scan.header.stamp)
            )
            if odom_pose is None:
                continue

            predicted_pose = self.compose_pose(self.map_to_odom, odom_pose)
            corrected_pose, score = self.match_scan(scan, predicted_pose)
            if self.should_create_keyframe(odom_pose):
                self.add_keyframe(scan, odom_pose, corrected_pose, score)
                self.map_to_odom = self.compose_pose(
                    corrected_pose,
                    self.inverse_pose(odom_pose),
                )
            if score > self.scan_match_min_score:
                self.get_logger().debug(f'Accepted scan score {score:.2f}')

    def should_create_keyframe(self, odom_pose):
        """Decide whether the odometry motion warrants a new keyframe.

        Args:
            odom_pose: Current ``(x, y, yaw)`` pose in the odom frame.

        Returns:
            ``True`` for the first scan or when translation or absolute yaw
            change exceeds its configured threshold. Keyframes reduce graph
            size while retaining observations after meaningful motion.
        """
        if not self.keyframes:
            return True
        previous = self.keyframes[-1]['odom_pose']
        distance = math.hypot(
            odom_pose[0] - previous[0], odom_pose[1] - previous[1]
        )
        rotation = abs(self.normalize_angle(odom_pose[2] - previous[2]))
        return distance >= self.keyframe_distance or rotation >= self.keyframe_rotation

    def add_keyframe(self, scan, odom_pose, map_pose, match_score):
        """Insert a keyframe and connect it to the pose graph.

        Args:
            scan: Laser observation retained for future map rebuilding.
            odom_pose: Raw odometry pose associated with the scan.
            map_pose: Scan-matched pose in the map frame.
            match_score: Occupied-cell score produced by scan matching.

        Consecutive keyframes receive an odometry constraint. After insertion,
        a spatial loop candidate is searched. If one is accepted, the graph
        is relaxed and all keyframe scans are reintegrated using optimized
        poses.
        """
        node = {
            'scan': scan,
            'odom_pose': odom_pose,
            'pose': map_pose,
        }
        node_index = len(self.keyframes)
        if self.keyframes:
            previous = self.keyframes[-1]
            relative = self.compose_pose(
                self.inverse_pose(previous['odom_pose']), odom_pose
            )
            self.constraints.append((node_index - 1, node_index, relative, 1.0))
        self.keyframes.append(node)
        self.integrate_scan(scan, map_pose)

        loop = self.find_loop_closure(node_index)
        if loop is not None:
            previous_index, relative, score = loop
            self.constraints.append((previous_index, node_index, relative, 0.5))
            self.optimize_pose_graph()
            self.rebuild_map()
            self.get_logger().info(
                f'Loop closure {previous_index} -> {node_index}, score={score:.1f}'
            )

    def find_loop_closure(self, node_index):
        """Search for a non-local keyframe that explains the current scan.

        Args:
            node_index: Index of the newly inserted keyframe.

        Returns:
            ``(candidate_index, relative_pose, score)`` when a candidate is
            accepted, otherwise ``None``.

        Candidates must be at least ten keyframes old and close in the current
        estimated map. The current scan is matched around each candidate pose;
        a strong occupied-cell score becomes a loop-closure constraint.
        """
        if node_index < 10:
            return None
        current = self.keyframes[node_index]
        returns = self.scan_returns(current['scan'], stride=4)
        candidates = []
        for index, candidate in enumerate(self.keyframes[:-10]):
            distance = math.hypot(
                current['pose'][0] - candidate['pose'][0],
                current['pose'][1] - candidate['pose'][1],
            )
            if distance <= self.loop_closure_distance:
                candidates.append((distance, index, candidate))
        for _, index, candidate in sorted(candidates):
            matched_pose, score = self.search_pose(
                returns, candidate['pose'], 0.50, 0.50, 0.05
            )
            if score >= max(5.0, self.scan_match_min_score * 5.0):
                relative = self.compose_pose(
                    self.inverse_pose(candidate['pose']), matched_pose
                )
                return index, relative, score
        return None

    def optimize_pose_graph(self):
        """Relax pose-graph constraints with a small Gauss-Seidel-like update.

        Each constraint says that ``pose_j`` should equal
        ``pose_i ∘ relative``. The residual is split between both endpoints,
        except for node zero, which is fixed to remove global gauge freedom.
        This is not a nonlinear least-squares solver; it is a transparent
        educational approximation of graph optimization.
        """
        for _ in range(6):
            for first, second, relative, weight in self.constraints:
                first_pose = self.keyframes[first]['pose']
                second_pose = self.keyframes[second]['pose']
                target = self.compose_pose(first_pose, relative)
                error = (
                    target[0] - second_pose[0],
                    target[1] - second_pose[1],
                    self.normalize_angle(target[2] - second_pose[2]),
                )
                gain = 0.20 * weight
                if first == 0:
                    self.keyframes[second]['pose'] = (
                        second_pose[0] + gain * error[0],
                        second_pose[1] + gain * error[1],
                        self.normalize_angle(second_pose[2] + gain * error[2]),
                    )
                else:
                    self.keyframes[first]['pose'] = (
                        first_pose[0] - 0.5 * gain * error[0],
                        first_pose[1] - 0.5 * gain * error[1],
                        self.normalize_angle(first_pose[2] - 0.5 * gain * error[2]),
                    )
                    self.keyframes[second]['pose'] = (
                        second_pose[0] + 0.5 * gain * error[0],
                        second_pose[1] + 0.5 * gain * error[1],
                        self.normalize_angle(second_pose[2] + 0.5 * gain * error[2]),
                    )
        last = self.keyframes[-1]
        self.map_to_odom = self.compose_pose(
            last['pose'], self.inverse_pose(last['odom_pose'])
        )

    def rebuild_map(self):
        """Clear the grid and replay all retained scans at optimized poses.

        Rebuilding is simple and deterministic, but computationally expensive
        compared with an incremental submap implementation. It is appropriate
        for this small demonstration and makes the effect of loop closure easy
        to observe.
        """
        self.log_odds.clear()
        for keyframe in self.keyframes:
            self.integrate_scan(keyframe['scan'], keyframe['pose'])

    def integrate_scan(self, scan, sensor_pose):
        """Apply an inverse sensor model to one scan.

        Args:
            scan: LaserScan whose rays are expressed in the laser frame.
            sensor_pose: Corrected ``(x, y, yaw)`` pose in the map frame.

        A ray marks cells before a measured return as free. A finite return
        endpoint is marked occupied. Rays reaching the sensor maximum are
        treated as no-return observations and mark only traversed free cells.
        Log odds are bounded in :meth:`update_cell` to prevent one cell from
        dominating the map numerically.
        """
        sensor_x, sensor_y, sensor_yaw = sensor_pose
        sensor_cell = self.world_to_cell(sensor_x, sensor_y)

        for index in range(len(scan.ranges)):
            measured_range = self.filtered_range(scan, index)
            if measured_range is None:
                continue
            distance = min(
                max(measured_range, scan.range_min),
                scan.range_max,
                self.ray_limit,
            )
            angle = sensor_yaw + scan.angle_min + index * scan.angle_increment
            endpoint = (
                sensor_x + distance * math.cos(angle),
                sensor_y + distance * math.sin(angle),
            )
            endpoint_cell = self.world_to_cell(*endpoint)
            cells = list(self.bresenham(sensor_cell, endpoint_cell))
            if not cells:
                continue

            is_return = (
                scan.range_min <= measured_range < scan.range_max
                and measured_range <= self.ray_limit
            )
            free_cells = cells[:-1] if is_return else cells
            for cell in free_cells:
                self.update_cell(cell, self.free_update)
            if is_return:
                self.update_cell(endpoint_cell, self.occupied_update)

    @staticmethod
    def filtered_range(scan, index):
        """Return a usable range or reject an isolated measurement spike.

        Args:
            scan: LaserScan containing the range array and limits.
            index: Index of the candidate range.

        Returns:
            A finite range within the sensor limits, or ``None``. The simple
            neighbor test rejects a one-ray spike when both adjacent rays
            agree with each other.
        """
        distance = scan.ranges[index]
        if not math.isfinite(distance):
            return None
        if distance < scan.range_min or distance > scan.range_max:
            return None
        if 0 < index < len(scan.ranges) - 1:
            previous = scan.ranges[index - 1]
            following = scan.ranges[index + 1]
            if math.isfinite(previous) and math.isfinite(following):
                neighbours_are_consistent = abs(previous - following) < 0.20
                is_isolated_spike = (
                    abs(distance - previous) > 1.0
                    and abs(distance - following) > 1.0
                )
                if neighbours_are_consistent and is_isolated_spike:
                    return None
        return distance

    def match_scan(self, scan, predicted_pose):
        """Search nearby poses and maximize agreement with occupied cells.

        Args:
            scan: Laser observation to register.
            predicted_pose: Odometry-propagated pose in the map frame.

        Returns:
            ``(pose, score)`` for the best accepted pose. This is a lightweight
            correlative matcher. It uses measured returns
        only, evaluates a coarse grid around the odometry prediction, then a
        finer grid around the best coarse candidate. Unknown cells do not
        influence the score, so the matcher naturally falls back to odometry
        while the map is still empty.
        """
        returns = self.scan_returns(scan, stride=4)
        if not returns or not self.log_odds:
            return predicted_pose, 0.0

        best_pose, best_score = self.search_pose(
            returns,
            predicted_pose,
            self.scan_match_translation,
            self.scan_match_rotation,
            0.10,
        )
        best_pose, best_score = self.search_pose(
            returns,
            best_pose,
            0.10,
            0.10,
            0.04,
        )
        if best_score < self.scan_match_min_score:
            return predicted_pose, best_score
        return best_pose, best_score

    @staticmethod
    def scan_returns(scan, stride=1):
        """Extract finite return measurements for scan matching.

        Args:
            scan: LaserScan to sample.
            stride: Keep every ``stride``-th ray to reduce search cost.

        Returns:
            A list of ``(range, bearing)`` pairs. Maximum-range/no-return
            samples are excluded because they do not identify an obstacle.
        """
        returns = []
        for index in range(0, len(scan.ranges), stride):
            distance = scan.ranges[index]
            if (
                math.isfinite(distance)
                and scan.range_min <= distance < scan.range_max
            ):
                returns.append((distance, scan.angle_min + index * scan.angle_increment))
        return returns

    def search_pose(self, returns, center, translation_radius,
                    rotation_radius, step):
        """Evaluate a regular 3-D pose lattice around ``center``.

        Args:
            returns: ``(range, bearing)`` observations.
            center: Search-center pose ``(x, y, yaw)``.
            translation_radius: Extent of the x/y search in metres.
            rotation_radius: Extent of the yaw search in radians.
            step: Grid spacing used for all three dimensions.

        Returns:
            The candidate pose with the highest :meth:`scan_match_score` and
            its score. The search is exhaustive and therefore easy to study,
            but more expensive than gradient-based registration.
        """
        best_pose = center
        best_score = self.scan_match_score(returns, center)
        translation_steps = range(
            -round(translation_radius / step),
            round(translation_radius / step) + 1,
        )
        rotation_steps = range(
            -round(rotation_radius / step),
            round(rotation_radius / step) + 1,
        )
        for x_step in translation_steps:
            for y_step in translation_steps:
                for yaw_step in rotation_steps:
                    candidate = (
                        center[0] + x_step * step,
                        center[1] + y_step * step,
                        self.normalize_angle(center[2] + yaw_step * step),
                    )
                    score = self.scan_match_score(returns, candidate)
                    if score > best_score:
                        best_pose, best_score = candidate, score
        return best_pose, best_score

    def scan_match_score(self, returns, pose):
        """Score transformed endpoints against occupied map evidence.

        Args:
            returns: ``(range, bearing)`` laser returns.
            pose: Candidate sensor pose in the map frame.

        Returns:
            Sum of positive log-odds values at transformed endpoint cells.
            Unknown cells contribute zero, which prevents an empty map from
            creating an arbitrary pose correction.
        """
        score = 0.0
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        for distance, angle in returns:
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            world_x = pose[0] + cosine * local_x - sine * local_y
            world_y = pose[1] + sine * local_x + cosine * local_y
            value = self.log_odds.get(self.world_to_cell(world_x, world_y))
            if value is not None and value > 0.0:
                score += value
        return score

    def update_cell(self, cell, update):
        """Apply a bounded inverse-sensor-model update to one cell.

        Args:
            cell: Local ``(x, y)`` map index.
            update: Positive occupied or negative free log-odds increment.

        The bounds ``[-5, 5]`` prevent repeated observations from overflowing
        and keep the occupancy classification numerically stable.
        """
        if not self.in_bounds(cell):
            return
        value = self.log_odds.get(cell, 0.0) + update
        self.log_odds[cell] = max(-5.0, min(5.0, value))

    def publish_map(self):
        """Advance the pipeline and publish the current ROS map.

        The timer is intentionally the synchronization point for queued input:
        callbacks only cache messages, while this method performs mapping and
        publishes a coherent grid/TF pair.
        """
        self.process_pending_scans()
        if not self.odom_history:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = [self.cell_value((x, y)) for y in range(self.height) for x in range(self.width)]
        self.map_publisher.publish(message)
        self.publish_map_to_odom_tf(message.header.stamp)
        self.map_sequence += 1

    def publish_map_to_odom_tf(self, stamp):
        """Publish the estimated correction from odometry into the map frame.

        Args:
            stamp: ROS timestamp assigned to the transform.

        The transform lets consumers combine the talker’s ``odom -> base``
        transform with the mapper’s global correction:

        ``map -> odom -> base_footprint -> laser``.
        """
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = self.map_to_odom[0]
        transform.transform.translation.y = self.map_to_odom[1]
        transform.transform.rotation.z = math.sin(self.map_to_odom[2] / 2.0)
        transform.transform.rotation.w = math.cos(self.map_to_odom[2] / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def cell_value(self, cell):
        """Convert log odds to a ROS occupancy-grid value.

        Returns ``100`` for occupied, ``0`` for free, and ``-1`` for unknown.
        The thresholds are the log-odds equivalents of probabilities 0.65 and
        0.196 used by the map-server-style representation.
        """
        value = self.log_odds.get(cell, 0.0)
        if value > 0.619:
            return 100
        if value < -1.408:
            return 0
        return -1

    def world_to_cell(self, x, y):
        """Convert continuous map-frame coordinates into grid indices.

        ``floor`` is important for negative coordinates: it preserves the
        mathematical half-open cell interval ``[origin + i*r,
        origin + (i+1)*r)``.
        """
        return (
            math.floor((x - self.origin_x) / self.resolution),
            math.floor((y - self.origin_y) / self.resolution),
        )

    def in_bounds(self, cell):
        """Return whether a local ``(x, y)`` index lies in the map array."""
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    @staticmethod
    def stamp_to_seconds(stamp):
        """Convert a ROS ``builtin_interfaces/Time`` message to seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    def closest_odom_pose(self, stamp):
        """Return the nearest cached odometry pose for a scan timestamp.

        Args:
            stamp: Scan timestamp in floating-point seconds.

        Returns:
            The closest ``(x, y, yaw)`` odometry pose, or ``None`` when the
            history is empty. This implementation uses nearest-neighbor
            association; a production system would usually interpolate TF.
        """
        if not self.odom_history:
            return None
        _, pose = min(self.odom_history, key=lambda item: abs(item[0] - stamp))
        return pose

    @staticmethod
    def compose_pose(first, second):
        """Compose two planar rigid transforms.

        If ``first`` maps frame A to B and ``second`` maps B to C, the result
        maps A to C. In this node it is used for ``map_to_odom ∘ odom_pose``.
        """
        cosine = math.cos(first[2])
        sine = math.sin(first[2])
        return (
            first[0] + cosine * second[0] - sine * second[1],
            first[1] + sine * second[0] + cosine * second[1],
            ScratchSlam.normalize_angle(first[2] + second[2]),
        )

    @staticmethod
    def inverse_pose(pose):
        """Return the inverse of a planar ``(x, y, yaw)`` transform."""
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        return (
            -cosine * pose[0] - sine * pose[1],
            sine * pose[0] - cosine * pose[1],
            ScratchSlam.normalize_angle(-pose[2]),
        )

    @staticmethod
    def normalize_angle(angle):
        """Wrap an angle to the canonical interval ``[-pi, pi)``."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def quaternion_to_yaw(quaternion):
        """Extract planar yaw from a ROS quaternion.

        The demo is planar, so roll and pitch are ignored. The formula is the
        standard ``atan2(2(wz + xy), 1 - 2(y² + z²))`` conversion.
        """
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def bresenham(start, end):
        """Yield grid cells along a ray using integer Bresenham traversal.

        Args:
            start: Integer sensor-cell ``(x, y)``.
            end: Integer endpoint-cell ``(x, y)``.

        Yields:
            Every discrete cell intersected by the segment, including both
            endpoints. The mapper uses all but the endpoint as free cells for
            a measured return.
        """
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            yield x0, y0
            if (x0, y0) == (x1, y1):
                return
            double_error = 2 * error
            if double_error >= dy:
                error += dy
                x0 += step_x
            if double_error <= dx:
                error += dx
                y0 += step_y


def main(args=None):
    """Initialize ROS, run :class:`ScratchSlam`, and shut down cleanly.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.
    """
    rclpy.init(args=args)
    node = ScratchSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
