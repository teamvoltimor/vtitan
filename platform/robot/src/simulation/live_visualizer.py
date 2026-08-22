"""Publish the headless simulator's ground-truth state to ROS2 for RViz/Gazebo.

Lets the exact same closed-loop scenario the headless test battery runs
(``ScenarioSimulator`` driving the real ``CoreNavigator``) be watched live: the
pose becomes a TF frame + ``nav_msgs/Odometry``, the raycast LIDAR becomes a
``sensor_msgs/LaserScan``, and the track walls become a one-shot
``visualization_msgs/MarkerArray`` built from the same ``TrackModel`` geometry
the collision checks use — so what you see is what was actually tested, not a
static approximation.

Requires the ``robot`` pixi environment (ROS2 Kilted / RoboStack); not
importable from the plain ``uv`` env the headless tests run in.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from shared.config.constants import (
    ParkingLotSpecs,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
)
from shared.config.ros_topics import RosTopicConfig
from shared.domain.models import SignColor
from src.ros2.qos import QOS_STREAM
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState
    from src.simulation.track_model import TrackModel

_MAP_FRAME = "map"
_ROBOT_FRAME = "base_link"


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _pitch_to_quaternion(pitch: float) -> Quaternion:
    return Quaternion(x=0.0, y=math.sin(pitch / 2.0), z=0.0, w=math.cos(pitch / 2.0))


class LiveScenarioVisualizer(Node):
    """Publishes one running scenario's pose/LIDAR/track to ROS2 topics."""

    _TRACK_REPUBLISH_EVERY_N_TICKS = 20
    """Re-send the cached track/robot MarkerArray about once a second (at the default 20Hz
    sim rate) instead of relying on a single one-shot publish. A late-connecting or
    reconnecting RViz subscriber can otherwise miss that first publish entirely and never
    show the markers, since a plain volatile-QoS publish isn't retained for late joiners."""

    def __init__(self, track: TrackModel, node_name: str = "sim_live_visualizer") -> None:
        super().__init__(node_name)
        topics = RosTopicConfig.load_default()
        self._odom_pub = self.create_publisher(Odometry, topics.simulation.odom, QOS_STREAM)
        self._scan_pub = self.create_publisher(
            LaserScan,
            topics.sensors.scan,
            QOS_STREAM,
        )
        self._track_pub = self.create_publisher(MarkerArray, topics.simulation.track, 1)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cached_track_markers: MarkerArray | None = None
        self._tick_count = 0
        self.set_track(track)

    def set_track(
        self,
        track: TrackModel,
        sign_positions: list[dict] | None = None,
        parking_lot: dict | None = None,
    ) -> None:
        """(Re)publish the track walls, signs, and parking lot — call again per scenario."""
        markers = MarkerArray()
        # Clear every previously published marker first. Sign/parking marker IDs are just
        # 0..N-1 within their namespace — switching to a scenario with fewer signs (or no
        # parking lot) would otherwise leave the previous scenario's markers stuck on screen,
        # rendered at positions that belong to a different track layout entirely (e.g. sitting
        # on/through the new track's wall).
        markers.markers.append(Marker(action=Marker.DELETEALL))
        markers.markers.append(self._outer_boundary_marker())
        markers.markers.append(self._inner_block_marker(track))
        for i, sign in enumerate(sign_positions or []):
            markers.markers.append(self._sign_marker(i, sign))
        if parking_lot is not None:
            markers.markers.append(
                self._parking_block_marker(10, parking_lot["block1_position"], parking_lot.get("block1_yaw", 0.0)),
            )
            markers.markers.append(
                self._parking_block_marker(11, parking_lot["block2_position"], parking_lot.get("block2_yaw", 0.0)),
            )
        markers.markers.extend(self._robot_model_markers())
        self._cached_track_markers = markers
        self._track_pub.publish(markers)

    def publish(self, state: AckermannState, scan: LidarScan | None) -> None:
        """Publish one tick's pose (odom + TF) and LIDAR sweep."""
        self._tick_count += 1
        if self._cached_track_markers is not None and (self._tick_count % self._TRACK_REPUBLISH_EVERY_N_TICKS == 0):
            self._track_pub.publish(self._cached_track_markers)

        stamp = self.get_clock().now().to_msg()
        quat = _yaw_to_quaternion(state.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = _MAP_FRAME
        tf.child_frame_id = _ROBOT_FRAME
        tf.transform.translation.x = state.x
        tf.transform.translation.y = state.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quat
        self._tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = _MAP_FRAME
        odom.child_frame_id = _ROBOT_FRAME
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = state.v
        self._odom_pub.publish(odom)

        if scan is not None and scan.ranges_m:
            self._scan_pub.publish(self._build_laserscan(scan, stamp))

    def _build_laserscan(self, scan: LidarScan, stamp: object) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = _ROBOT_FRAME
        angles = scan.angles_rad
        msg.angle_min = angles[0]
        msg.angle_max = angles[-1]
        msg.angle_increment = (angles[-1] - angles[0]) / max(1, len(angles) - 1)
        msg.range_min = RobotSpecs.LIDAR_MIN_RANGE
        msg.range_max = RobotSpecs.LIDAR_MAX_RANGE
        msg.ranges = list(scan.ranges_m)
        return msg

    def _outer_boundary_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = _MAP_FRAME
        m.ns = "track"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = WallSpecs.THICKNESS
        m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.8, 0.8, 1.0
        edge = TrackDimensions.MAX_COORD
        for x, y in [(0.0, 0.0), (edge, 0.0), (edge, edge), (0.0, edge), (0.0, 0.0)]:
            m.points.append(Point(x=x, y=y, z=0.05))
        return m

    def _inner_block_marker(self, track: TrackModel) -> Marker:
        x_min, y_min, x_max, y_max = track.inner_block_visual
        m = Marker()
        m.header.frame_id = _MAP_FRAME
        m.ns = "track"
        m.id = 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (x_min + x_max) / 2.0
        m.pose.position.y = (y_min + y_max) / 2.0
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = x_max - x_min
        m.scale.y = y_max - y_min
        m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.2, 0.2, 1.0
        return m

    def _sign_marker(self, index: int, sign: dict) -> Marker:
        m = Marker()
        m.header.frame_id = _MAP_FRAME
        m.ns = "signs"
        m.id = index
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        # Whole-number scenario-metadata coordinates parse from JSON as Python int, not
        # float. Assigning an int straight to a Point field looks fine in-memory (Python
        # doesn't care), but CDR serialization onto the wire reinterprets its bits as a
        # float64 instead of converting the value — the sign silently jumps to ~0.0.
        m.pose.position.x = float(sign["x"])
        m.pose.position.y = float(sign["y"])
        m.pose.position.z = TrafficSignSpecs.Z_POSITION
        m.pose.orientation.w = 1.0
        m.scale.x = TrafficSignSpecs.WIDTH
        m.scale.y = TrafficSignSpecs.DEPTH
        m.scale.z = TrafficSignSpecs.HEIGHT
        color = TrafficSignSpecs.RED_COLOR if sign["color"] == SignColor.RED else TrafficSignSpecs.GREEN_COLOR
        m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.0
        return m

    def _parking_block_marker(self, index: int, block: dict, yaw: float) -> Marker:
        m = Marker()
        m.header.frame_id = _MAP_FRAME
        m.ns = "parking"
        m.id = index
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # See _sign_marker: JSON-int coordinates must be coerced to float before
        # reaching a Point field, or CDR serialization corrupts them to ~0.0.
        m.pose.position.x = float(block["x"])
        m.pose.position.y = float(block["y"])
        m.pose.position.z = ParkingLotSpecs.Z_POSITION
        m.pose.orientation = _yaw_to_quaternion(yaw)
        m.scale.x = ParkingLotSpecs.LENGTH
        m.scale.y = ParkingLotSpecs.WIDTH
        m.scale.z = ParkingLotSpecs.HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = *ParkingLotSpecs.COLOR, 1.0
        return m

    def _robot_model_markers(self) -> list[Marker]:
        """Chassis/LIDAR/camera geometry, static relative to ``base_link``.

        This sim/RViz path has no URDF or Gazebo mesh — this is the only place these
        mount offsets (LIDAR front-mount, camera-over-LIDAR pitch) are visible without
        launching Gazebo. Positioned directly from ``RobotSpecs`` (not a duplicate copy)
        so it can't itself drift from the values it's meant to let you check.
        """
        cam_quat = _pitch_to_quaternion(math.radians(RobotSpecs.CAMERA_MOUNT_PITCH_DEG))
        return [
            self._chassis_marker(),
            self._robot_lidar_marker(),
            self._camera_marker(cam_quat),
            self._camera_facing_marker(cam_quat),
        ]

    def _chassis_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = _ROBOT_FRAME
        m.ns = "robot"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = RobotSpecs.HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = RobotSpecs.LENGTH
        m.scale.y = RobotSpecs.WIDTH
        m.scale.z = RobotSpecs.HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 0.8, 0.6
        return m

    def _robot_lidar_marker(self) -> Marker:
        # z = HEIGHT + 0.02 = 0.12, matching static_tfs.launch.py / the Go SDF generator.
        m = Marker()
        m.header.frame_id = _ROBOT_FRAME
        m.ns = "robot"
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.LIDAR_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.HEIGHT + 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = RobotSpecs.LIDAR_DIAMETER
        m.scale.y = RobotSpecs.LIDAR_DIAMETER
        m.scale.z = RobotSpecs.LIDAR_HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.1, 0.1, 1.0
        return m

    def _camera_marker(self, orientation: Quaternion) -> Marker:
        m = Marker()
        m.header.frame_id = _ROBOT_FRAME
        m.ns = "robot"
        m.id = 2
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.CAMERA_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.CAMERA_MOUNT_Z_OFFSET
        m.pose.orientation = orientation
        m.scale.x = m.scale.y = m.scale.z = 0.025
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.2, 0.2, 1.0
        return m

    def _camera_facing_marker(self, orientation: Quaternion) -> Marker:
        # A plain cube's own rotation is hard to read at a glance -- this arrow makes the
        # camera's actual look direction (including the downward pitch) unambiguous.
        m = Marker()
        m.header.frame_id = _ROBOT_FRAME
        m.ns = "robot"
        m.id = 3
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.CAMERA_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.CAMERA_MOUNT_Z_OFFSET
        m.pose.orientation = orientation
        m.scale.x = 0.15  # shaft length
        m.scale.y = 0.02  # shaft diameter
        m.scale.z = 0.02  # head diameter
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
        return m


class RealTimePacer:
    """Sleeps between ticks so a headless-speed loop plays back at wall-clock rate."""

    def __init__(self, dt: float, rate: float = 1.0) -> None:
        self._tick_budget = dt / rate if rate > 0 else 0.0
        self._next_tick: float | None = None

    def wait(self) -> None:
        """Block until the next tick's wall-clock deadline (no-op if unthrottled)."""
        if self._tick_budget <= 0.0:
            return
        now = time.monotonic()
        if self._next_tick is None:
            self._next_tick = now
        self._next_tick += self._tick_budget
        delay = self._next_tick - now
        if delay > 0.0:
            time.sleep(delay)
        else:
            self._next_tick = now


def init_rclpy_once() -> None:
    """Initialize rclpy if it hasn't been already (idempotent for script reuse)."""
    if not rclpy.ok():
        rclpy.init()
