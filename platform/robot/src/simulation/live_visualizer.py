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

import rclpy
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs, TrackDimensions
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from src.navigation.ports import LidarScan
from src.simulation.kinematics import AckermannState
from src.simulation.track_model import TrackModel

_MAP_FRAME = "map"
_ROBOT_FRAME = "base_link"
_WALL_THICKNESS_M = 0.10


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class LiveScenarioVisualizer(Node):
    """Publishes one running scenario's pose/LIDAR/track to ROS2 topics."""

    def __init__(self, track: TrackModel, node_name: str = "sim_live_visualizer") -> None:
        super().__init__(node_name)
        self._odom_pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._track_pub = self.create_publisher(MarkerArray, "/sim/track", 1)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.set_track(track)

    def set_track(self, track: TrackModel) -> None:
        """(Re)publish the track walls — call again when switching scenarios."""
        self._publish_track(track)

    def publish(self, state: AckermannState, scan: LidarScan | None) -> None:
        """Publish one tick's pose (odom + TF) and LIDAR sweep."""
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

    def _publish_track(self, track: TrackModel) -> None:
        """Draw the outer boundary + inner block once (matches the collision model)."""
        markers = MarkerArray()
        markers.markers.append(self._outer_boundary_marker())
        markers.markers.append(self._inner_block_marker(track))
        self._track_pub.publish(markers)

    def _outer_boundary_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = _MAP_FRAME
        m.ns = "track"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = _WALL_THICKNESS_M
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


class RealTimePacer:
    """Sleeps between ticks so a headless-speed loop plays back at wall-clock rate."""

    def __init__(self, dt: float, rate: float = 1.0) -> None:
        self._tick_budget = dt / rate if rate > 0 else 0.0
        self._next_tick: float | None = None

    def wait(self) -> None:
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
