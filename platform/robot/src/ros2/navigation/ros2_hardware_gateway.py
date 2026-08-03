"""ROS2-backed HardwareGateway: bridges TrackNavigator to real ROS2 topics.

Subscribes to /scan, /imu/data, /joint_states, /vision/detections and
publishes /ackermann_cmd, implementing the same
:class:`~src.navigation.ports.HardwareGateway` protocol
:class:`~src.simulation.simulated_hardware_gateway.SimulatedHardwareGateway`
implements for the closed-loop simulator, so
:class:`~src.navigation.core_navigator.CoreNavigator` runs unmodified on
either.
"""

from __future__ import annotations

import json
import math

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.constants import RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.enums import Section
from shared.config.navigation_tuning import LocalizationParams, SensorHealthParams
from shared.domain.models import CorridorGeometry, Detection, IMUReading, Pose, TrafficSignObservation
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.hardware.motors.enums import DRIVE_JOINT
from src.navigation.localization import LidarLocalizer
from src.navigation.ports import DriveCommand, HardwareGateway, LidarScan, WheelOdometry
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.state_machine.estimator import StateEstimator

_LIDAR_YAW_OFFSET_RAD = math.radians(
    (180.0 if RobotSpecs.LIDAR_INVERTED else 0.0) + RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG
)
"""Rotates raw /scan bearings into the robot frame (0 rad = forward).

Two independent components, combined here rather than read as one pre-combined
constant: RobotSpecs.LIDAR_INVERTED (mandatory 180 deg when the mount is
upside-down -- the same fact that also has to drive the sllidar_ros2 driver's
own `inverted` launch parameter, see robot.toml's [lidar] section) plus
RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG (any additional residual miscalibration,
independent of the inversion). Getting these out of sync -- e.g. flipping the
driver's `inverted` parameter without updating this rotation, or vice versa --
is exactly the bug found and fixed 2026-08-02: a stale 180 deg applied on top
of already-correctly-oriented raw data rotated the navigator's whole picture
180 deg, front read as back and left read as right.

This constant already drives static_tfs.launch.py's lidar_link TF rotation
(computed the same way there), but nothing reads that TF back -- every
consumer of LidarScan.angles_rad (CollisionAvoidanceController via
core_navigator.py, and estimate_yaw_from_walls) documents and requires 0 rad
= forward, so the correction has to happen here, where angles_rad is actually
built. Invisible in simulation, which synthesizes scan angles already in the
correct robot frame and never models a raw LIDAR mounting frame at all.
"""


def _topic(node: Node, name: str, default: str) -> str:
    """Resolve a topic parameter, declaring it if the host node has not.

    ROS2HardwareGateway is constructed against a host node that is expected to
    have declared its topic parameters, which TrackNavigator does. Anything
    else building a gateway -- the contract tests, in two separate files -- has
    to replicate that list exactly, and adding a topic here broke both of them
    with ParameterNotDeclaredException at construction. Declaring on demand
    makes the gateway responsible for its own inputs, so a new topic cannot
    silently become a required ritual for every caller.
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return str(node.get_parameter(name).get_parameter_value().string_value)


class ROS2HardwareGateway(HardwareGateway):
    """Implementation of HardwareGateway for ROS2 environment.

    Acts as an adapter between the ROS2 Node topics and the pure Python
    CoreNavigator logic.
    """

    def __init__(
        self,
        node: Node,
        start_x: float,
        start_y: float,
        start_yaw: float,
        geometry: CorridorGeometry | dict[Section, float],
        stale_timeout_sec: float = SensorHealthParams().STALE_TIMEOUT_SEC,
        localization: LocalizationParams | None = None,
    ) -> None:
        self._node = node
        self._estimator = StateEstimator(start_x, start_y, start_yaw)
        self._localization_params = localization or LocalizationParams()
        # Accept dict for backward compat (test callers).
        geom = corridor_geometry_from_widths(geometry) if isinstance(geometry, dict) else geometry
        self._localizer = LidarLocalizer(
            TrackWalls(geom),
            search_radius_m=self._localization_params.SEARCH_RADIUS_M,
            passes=self._localization_params.PASSES,
            grid_points=self._localization_params.GRID_POINTS,
            residual_clip_m=self._localization_params.RESIDUAL_CLIP_M,
        )
        self._latest_lidar: LidarScan | None = None
        self._latest_detections: list[Detection] = []
        self._latest_imu: IMUReading | None = None
        self._latest_wheel: WheelOdometry | None = None
        # Receipt timestamp (seconds) for staleness / dropout detection. LIDAR
        # is the position source (no wheel odometry exists on real hardware),
        # so its staleness gates get_current_pose() too.
        self._lidar_stamp: float | None = None
        self._stale_timeout_sec = stale_timeout_sec

        # Publishers
        self._drive_publisher = node.create_publisher(
            AckermannDriveStamped,
            _topic(node, "ackermann_cmd_topic", "/ackermann_cmd"),
            10,
        )

        # Subscribers
        node.create_subscription(
            LaserScan,
            _topic(node, "lidar_topic", "/scan"),
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            String,
            _topic(node, "vision_topic", "/vision/detections"),
            self._vision_callback,
            10,
        )
        node.create_subscription(
            Imu,
            _topic(node, "imu_topic", "/imu/data"),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            JointState,
            _topic(node, "joint_states_topic", "/joint_states"),
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

    def set_believed_walls(self, walls: TrackWalls) -> None:
        """Re-point the localizer at the layout the robot currently believes in.

        Blind operation estimates corridor widths as it drives, so the wall
        model the scans are matched against changes mid-round. Without this the
        localizer keeps matching against the layout assumed at startup, and a
        corrected belief never reaches the position fix.
        """
        self._localizer = LidarLocalizer(
            walls,
            search_radius_m=self._localization_params.SEARCH_RADIUS_M,
            passes=self._localization_params.PASSES,
            grid_points=self._localization_params.GRID_POINTS,
            residual_clip_m=self._localization_params.RESIDUAL_CLIP_M,
        )

    def reset_heading_reference(self) -> None:
        """Re-zero the estimator's heading against the next IMU reading."""
        self._estimator.reset_heading_reference()

    def _joint_state_callback(self, msg: JointState) -> None:
        """Convert the drive wheel's angle and rate into linear travel.

        Indexed by joint name rather than array position: JointState carries an
        arbitrary set of joints in an arbitrary order, and assuming index 0 is
        the drive wheel would break silently the moment another joint is added.
        """
        try:
            i = msg.name.index(DRIVE_JOINT)
        except ValueError:
            return
        if i >= len(msg.position):
            return

        radius = RobotSpecs.WHEEL_RADIUS
        speed = msg.velocity[i] * radius if i < len(msg.velocity) else 0.0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_wheel = WheelOdometry(
            distance_m=msg.position[i] * radius,
            speed_mps=speed,
            stamp_s=stamp,
        )

    def get_wheel_odometry(self) -> WheelOdometry | None:
        """Latest wheel travel, or ``None`` before the first /joint_states message."""
        return self._latest_wheel

    def _imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        reading = IMUReading(yaw=yaw, pitch=0.0, roll=0.0)
        self._latest_imu = reading
        self._estimator.update_imu(reading)

    def _now(self) -> float:
        """Current node clock time in seconds (sim time when enabled)."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _lidar_callback(self, msg: LaserScan) -> None:
        raw = np.array(msg.ranges, dtype=float)
        # Invalid returns (NaN / +-inf, which Slamtec drivers emit for no-return
        # rays) must not survive: NaN silently drops out of every downstream mask
        # and inf reads as "far away", so replace both with the max range before
        # clamping. Zero/near-zero (also emitted for invalid) is filtered later by
        # the collision controller's ``> 0.01`` guard.
        raw[~np.isfinite(raw)] = RobotSpecs.LIDAR_MAX_RANGE
        raw = np.clip(raw, 0.0, RobotSpecs.LIDAR_MAX_RANGE)
        angles = (np.linspace(msg.angle_min, msg.angle_max, len(raw)) + _LIDAR_YAW_OFFSET_RAD).tolist()
        self._latest_lidar = LidarScan(ranges_m=tuple(raw.tolist()), angles_rad=tuple(angles))
        self._lidar_stamp = self._now()

        # Correct heading against the walls before solving for position. The
        # localizer takes yaw as given, so a better yaw yields a better fix --
        # and this is the only thing that bounds heading at all. The IMU has no
        # absolute reference, so without it drift and scale error accumulate
        # for the whole round; measured, 0.1 deg/s of drift costs 15 of 28
        # fixtures without this and none with it.
        measured_yaw = estimate_yaw_from_walls(
            self._latest_lidar.ranges_m,
            self._latest_lidar.angles_rad,
            prior_yaw=self._estimator.estimate_pose().yaw,
        )
        if measured_yaw is not None:
            self._estimator.correct_yaw(measured_yaw)

        # This scan is the position source: match it against the known wall
        # geometry, seeded from the previous estimate.
        #
        # Seeding it with encoder dead reckoning instead was tried and
        # rejected. The robot covers 0.8 cm between scans in simulation and
        # about 1.6 cm on the real C1 at full speed, against 3 cm of LIDAR
        # noise -- so "assume it barely moved" is not an approximation, it is
        # true, and the correction is smaller than the noise on the
        # measurement it would seed. Measured over the 28 fixtures it changed
        # the sighted peak error not at all and made the blind peak error 2.5x
        # worse (20.8 -> 52.2 cm), because blind means the wall model itself is
        # wrong and dead reckoning between poor fixes compounds drift rather
        # than staying anchored to the last one.
        prior_pose = self._estimator.estimate_pose()
        est_x, est_y = self._localizer.estimate_position(
            (prior_pose.x, prior_pose.y),
            prior_pose.yaw,
            raw.tolist(),
            angles,
        )
        self._estimator.update_position(est_x, est_y)

    def _vision_callback(self, msg: String) -> None:
        try:
            from ros2.vision.detection_payload_keys import (
                AREA_KEY,
                BBOX_KEY,
                CLASS_NAME_KEY,
                CONFIDENCE_KEY,
                HEIGHT_KEY,
                WIDTH_KEY,
                X_KEY,
                Y_KEY,
            )

            raw_data = json.loads(msg.data)
            self._latest_detections = []
            for d in raw_data:
                self._latest_detections.append(
                    Detection(
                        class_name=d.get(CLASS_NAME_KEY, ""),
                        confidence=d.get(CONFIDENCE_KEY, 0.0),
                        bbox=d.get(BBOX_KEY, (0.0, 0.0, 0.0, 0.0)),
                        x=d.get(X_KEY, 0.0),
                        y=d.get(Y_KEY, 0.0),
                        width=d.get(WIDTH_KEY, 0.0),
                        height=d.get(HEIGHT_KEY, 0.0),
                        area=d.get(AREA_KEY, 0.0),
                    ),
                )
        except (json.JSONDecodeError, TypeError):
            self._latest_detections = []

    def publish_drive(self, command: DriveCommand) -> None:
        """Publish drive command as an AckermannDriveStamped on the ackermann_cmd topic.

        This is the contract ``ackermann_motor_node`` actually subscribes to:
        ``drive.speed`` in m/s and ``drive.steering_angle`` as a physical angle
        in radians (not the normalised [-1, 1] steer CoreNavigator computes
        internally) — decoded via the same shared mapping the motor node and
        the simulator both use.
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.drive.speed = float(command.speed_mps)
        msg.drive.steering_angle = steering_norm_to_angle_rad(
            command.steering_norm,
            RobotSpecs.MAX_STEERING_ANGLE,
        )
        self._drive_publisher.publish(msg)

    def get_current_pose(self) -> Pose | None:
        """Get the latest fused pose, or None if LIDAR (the position source) has gone stale.

        Before the first LIDAR scan the estimator's seed pose is used
        (startup). Once scans have been seen, a stale feed is reported as None
        so the navigator stops rather than steering on a frozen position.
        """
        if self._lidar_stamp is not None and (self._now() - self._lidar_stamp) > self._stale_timeout_sec:
            return None
        return self._estimator.estimate_pose()

    def get_lidar_scan(self) -> LidarScan | None:
        """Get the latest processed LIDAR scan, or None if it has gone stale."""
        if self._lidar_stamp is None or (self._now() - self._lidar_stamp) > self._stale_timeout_sec:
            return None
        return self._latest_lidar

    def get_imu_reading(self) -> IMUReading | None:
        """Get the latest IMU orientation."""
        return self._latest_imu

    def get_vision_detections(self) -> list[TrafficSignObservation]:
        """Convert latest pixel detections to world-coordinate observations."""
        pose = self.get_current_pose()
        if pose is None or not self._latest_detections:
            return []
        from src.navigation.planning.sign_discovery import detection_to_observation

        result: list[TrafficSignObservation] = []
        for det in self._latest_detections:
            obs = detection_to_observation(det, (pose.x, pose.y), pose.yaw)
            if obs is not None:
                result.append(obs)
        return result
