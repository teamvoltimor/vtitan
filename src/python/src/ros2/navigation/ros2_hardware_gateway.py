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
from collections import deque
from typing import TYPE_CHECKING

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.constants import RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.navigation_tuning import LocalizationParams, SensorHealthParams
from shared.config.ros_topics import RosTopicConfig
from shared.domain.models import (
    CorridorGeometry,
    Detection,
    IMUReading,
    LocalizerHealth,
    LocalizerInputs,
    Pose,
    TrafficSignObservation,
    Waypoint,
)
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.config.tuning_helpers import get_tuning
from src.hardware.motors.enums import DRIVE_JOINT
from src.navigation.localization import make_localizer
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.ports import DriveCommand, HardwareGateway, LidarScan, WheelOdometry, sanitize_lidar_ranges
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths
from src.navigation.utils import clamp
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.ros2.params import declare_and_get_str_param
from src.ros2.qos import QOS_ACKERMANN_CMD, QOS_STREAM
from src.ros2.vision.detection_payload_keys import CAPTURED_AT_KEY, parse_detection
from src.state_machine.estimator import StateEstimator

if TYPE_CHECKING:
    from collections.abc import Sequence

    from rclpy.node import Node
    from shared.domain.enums import Section

_LIDAR_YAW_OFFSET_RAD = RobotSpecs.lidar_yaw_offset_rad()
"""Rotates raw /scan bearings into the robot frame (0 rad = forward).

See RobotSpecs.lidar_yaw_offset_rad()'s docstring for what this combines
(LIDAR_INVERTED's mandatory 180deg + any residual LIDAR_MOUNT_YAW_OFFSET_DEG)
and why it's a shared classmethod rather than recomputed per-consumer --
that duplication is exactly how telemetry_bridge_node.py's own copy of this
formula silently dropped the 180deg term for weeks.

This constant already drives static_tfs.launch.py's lidar_link TF rotation
(same classmethod, called there too), but nothing reads that TF back -- every
consumer of LidarScan.angles_rad (CollisionAvoidanceController via
core_navigator.py, and estimate_yaw_from_walls) documents and requires 0 rad
= forward, so the correction has to happen here, where angles_rad is actually
built. Invisible in simulation, which synthesizes scan angles already in the
correct robot frame and never models a raw LIDAR mounting frame at all.
"""


_POSE_HISTORY_LEN: int = 120
"""Poses retained for camera time-alignment: 6 s at the 20 Hz control rate.

Comfortably longer than the measured 0.85 s capture-to-receipt lag, so a
detection is still matchable after a scheduling hiccup, and small enough that
the whole buffer is a few kilobytes.
"""


def pose_at_time(
    history: Sequence[tuple[float, Pose]],
    target_s: float,
    fallback: Pose,
) -> Pose:
    """The recorded pose nearest ``target_s``, or ``fallback`` if none is old enough.

    Pure so it can be tested without a ROS node. Falls back rather than
    extrapolating: before the buffer covers the camera's lag -- the first
    second or so of a round -- the honest answer is the current pose, and
    inventing one backwards would be worse than the error it replaces.
    """
    if not history:
        return fallback
    if target_s < history[0][0]:
        return fallback
    return min(history, key=lambda entry: abs(entry[0] - target_s))[1]


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
        self._localizer = make_localizer(TrackWalls(geom), self._localization_params)
        self._latest_lidar: LidarScan | None = None
        self._latest_detections: list[Detection] = []
        # When the FRAME behind `_latest_detections` was grabbed, and a short
        # history of poses to look that instant up in. The camera pipeline is
        # not instant: measured 0.85 s from capture to the box arriving here on
        # run_20260906_232408/_232748. Pairing a detection with the pose at
        # RECEIPT put every sign where the robot had already moved to, which was
        # the whole of the bearing residual left after the mirror fix -- the
        # recovered cx-vs-bearing slope read -309 px/rad, a value no real lens
        # can produce, against a physical floor of ~620.
        self._detections_captured_at: float | None = None
        self._pose_history: deque[tuple[float, Pose]] = deque(maxlen=_POSE_HISTORY_LEN)
        # Corridor holding the parking lot, set by the node from the start
        # section. None means "unknown", which keeps the shape gate strict
        # everywhere rather than relaxing it on an unproven belief.
        self._parking_corridor: Section | None = None
        self._latest_imu: IMUReading | None = None
        self._latest_wheel: WheelOdometry | None = None
        self._localizer_inputs: LocalizerInputs | None = None
        # Receipt timestamp (seconds) for staleness / dropout detection. LIDAR
        # is the position source (no wheel odometry exists on real hardware),
        # so its staleness gates get_current_pose() too.
        self._lidar_stamp: float | None = None
        self._stale_timeout_sec = stale_timeout_sec

        topics = RosTopicConfig.load_default()

        # Publishers
        self._drive_publisher = node.create_publisher(
            AckermannDriveStamped,
            declare_and_get_str_param(node, "ackermann_cmd_topic", topics.commands.ackermann_cmd),
            QOS_ACKERMANN_CMD,
        )

        # Subscribers
        node.create_subscription(
            LaserScan,
            declare_and_get_str_param(node, "lidar_topic", topics.sensors.scan),
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            String,
            declare_and_get_str_param(node, "vision_topic", topics.sensors.vision_detections),
            self._vision_callback,
            QOS_STREAM,
        )
        node.create_subscription(
            Imu,
            declare_and_get_str_param(node, "imu_topic", topics.sensors.imu),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            JointState,
            declare_and_get_str_param(node, "joint_states_topic", topics.actuators.joint_states),
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
        self._localizer = make_localizer(walls, self._localization_params)

    def reset_heading_reference(self) -> None:
        """Re-zero the estimator's heading against the next IMU reading."""
        self._estimator.reset_heading_reference()

    def reset_position(self, x: float, y: float) -> None:
        """Re-seed the estimator's position at the start of a new race.

        See ``StateEstimator.reset_position`` -- without this a new race
        inherits wherever the previous one's position estimate last drifted
        to, instead of starting from this race's actual starting pose.

        The localizer's own between-tick state is cleared alongside it: a
        re-seed declares the previous fix void, and leaving the guard's held
        candidate behind lets it confirm the first post-reset estimate against
        a position computed in the frame that was just discarded (see
        ``LidarLocalizer.reset_tracking``).
        """
        self._estimator.reset_position(x, y)
        self._localizer.reset_tracking()

    def correct_heading_for_direction_change(self, delta_rad: float) -> None:
        """Shift the estimator's heading by a known amount, applied in full.

        See ``StateEstimator.apply_yaw_correction`` -- used when blind
        direction inference overturns the direction assumed at construction.
        """
        self._estimator.apply_yaw_correction(delta_rad)

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

        speed = msg.velocity[i] * RobotSpecs.WHEEL_RADIUS if i < len(msg.velocity) else 0.0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_wheel = WheelOdometry(
            distance_m=msg.position[i] * RobotSpecs.WHEEL_RADIUS,
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
        sanitized = sanitize_lidar_ranges(raw)
        angles = (np.linspace(msg.angle_min, msg.angle_max, len(sanitized)) + _LIDAR_YAW_OFFSET_RAD).tolist()
        self._latest_lidar = LidarScan(ranges_m=tuple(sanitized), angles_rad=tuple(angles))
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
        # Recorded before the call, not reconstructed from the pose afterwards:
        # these are the localizer's actual inputs, and the whole point is to be
        # able to tell them apart from the corrected pose the navigator reports.
        self._localizer_inputs = LocalizerInputs(prior_pose.yaw, prior_pose.x, prior_pose.y)
        est = self._localizer.estimate_position(
            Waypoint(prior_pose.x, prior_pose.y),
            prior_pose.yaw,
            sanitized,
            angles,
            now_s=self._now(),
        )
        self._estimator.update_position(est.x, est.y)
        # One entry per SCAN, not per `get_current_pose()` call: the navigator
        # asks several times a tick, and appending there would fill the buffer
        # with a fraction of a second of duplicates instead of the seconds of
        # history the camera lag needs.
        self._pose_history.append((self._lidar_stamp, self._estimator.estimate_pose()))

    def get_localizer_inputs(self) -> LocalizerInputs | None:
        """(yaw, prior_x, prior_y) handed to the localizer on the last scan."""
        return self._localizer_inputs

    def get_localizer_health(self) -> LocalizerHealth:
        """How well the last fix explained its scan, read live off the localizer.

        Not cached alongside ``_localizer_inputs``: ``set_believed_walls``
        replaces the localizer outright, and a cached copy would keep
        reporting the retired one's numbers.
        """
        return LocalizerHealth(
            fit_cost=self._localizer.last_fit_cost,
            relocalization_count=self._localizer.relocalization_count,
        )

    def _vision_callback(self, msg: String) -> None:
        try:
            raw_data = json.loads(msg.data)
            self._latest_detections = [det for d in raw_data if (det := parse_detection(d)) is not None]
            # Every box in one payload comes from one frame, so the first
            # carries the stamp for all of them. Absent on an older vision_node,
            # which `_pose_when_seen` falls back for.
            stamp = raw_data[0].get(CAPTURED_AT_KEY) if raw_data else None
            self._detections_captured_at = float(stamp) if stamp is not None else None
        except (json.JSONDecodeError, TypeError, ValueError, AttributeError):
            self._latest_detections = []
            self._detections_captured_at = None

    def publish_drive(self, command: DriveCommand) -> None:
        """Publish drive command as an AckermannDriveStamped on the ackermann_cmd topic.

        This is the contract ``ackermann_motor_node`` actually subscribes to:
        ``drive.speed`` in m/s and ``drive.steering_angle`` as a physical angle
        in radians (not the normalised [-1, 1] steer CoreNavigator computes
        internally) — decoded via the same shared mapping the motor node and
        the simulator both use.

        Speed is clamped to RobotSpecs.MAX_SPEED_MPS (the drive motor's
        measured top speed under load), matching what AckermannKinematics
        already enforces in the simulator -- previously only the sim clamped,
        so a speed profile asking for e.g. 0.5 m/s published that value
        verbatim here while the real motor physically saturates well below
        it. The clamp changes nothing about real motor behaviour (the PID's
        own output-duty clamp already saturates identically whether the
        setpoint is 0.156 or 0.5), it only stops logging/telemetry
        (commanded_speed_mps) from reporting an unreachable aspirational
        value instead of what was actually asked of the motor.
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        speed = clamp(command.speed_mps, -RobotSpecs.MAX_SPEED_MPS, RobotSpecs.MAX_SPEED_MPS)
        msg.drive.speed = float(speed)
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

    def set_parking_corridor(self, corridor: Section | None) -> None:
        """Tell the ingest path which corridor holds the parking lot.

        There is exactly one lot and it sits in the corridor the robot started
        in. Knowing that is what lets the shape gate stop rejecting wide RED
        boxes everywhere else, where no barrier can be.
        """
        self._parking_corridor = corridor

    def _pose_when_seen(self) -> Pose | None:
        """The pose the CAMERA saw from, not the pose by the time a box arrives.

        A detection is only useful as a bearing plus a range; turning that into a
        world position needs the pose at CAPTURE. The pipeline takes time --
        measured 0.85 s end to end on run_20260906_232408/_232748 -- and at
        0.3 m/s through a corner that is most of a sign's lateral offset.

        Uses the frame's own ``captured_at`` when the vision node supplied one,
        and otherwise steps back by ``VISION_LATENCY_S``, so an older vision_node
        still gets the correction rather than silently reverting to the receipt
        pose. Falls back to the current pose when the history has nothing old
        enough, which is only true in the first seconds of a round.
        """
        current = self.get_current_pose()
        if current is None or not self._pose_history:
            return current
        target = self._detections_captured_at
        if target is None:
            target = self._now() - get_tuning(None).sign_discovery.VISION_LATENCY_S
        return pose_at_time(self._pose_history, target, current)

    def get_vision_detections(self, current_corridor: Section | None = None) -> list[TrafficSignObservation]:
        """Convert latest pixel detections to world-coordinate observations.

        Fuses in the same tick's LIDAR sweep for range -- the camera alone
        is required for colour, but the LIDAR measures range at any distance
        far more precisely than depth-from-bbox-height does. See
        ``_detection_to_world``'s docstring.
        """
        pose = self._pose_when_seen()
        if pose is None or not self._latest_detections:
            return []

        scan = self.get_lidar_scan()
        lidar_ranges = scan.ranges_m if scan is not None else None
        lidar_angles = scan.angles_rad if scan is not None else None

        result: list[TrafficSignObservation] = []
        for det in self._latest_detections:
            # An unknown corridor counts as "the barrier could be here": the
            # label is unset at the start, in and around the bay, which is
            # exactly where the barrier is. Measured: 72% of wall-shaped red
            # detections carry no corridor label.
            #
            # The lot's NEIGHBOURS count too, because a corridor boundary is not
            # a sight line: the camera spans 102 deg and sees down the next
            # straight from a corner. Measured 2026-09-11 over three rounds, the
            # shape gate admitted 293 wall-shaped reds on the strength of this
            # test and 257 of them (88%) were taken from the corridor NEXT to
            # the lot's -- which is also where 55 of the barrier's own 220
            # magenta detections come from, so the barrier demonstrably IS
            # visible from there.
            barrier_possible = (
                self._parking_corridor is None
                or current_corridor is None
                or current_corridor == self._parking_corridor
                or current_corridor in self._parking_corridor.neighbours
            )
            obs = detection_to_observation(
                det,
                pose,
                lidar_ranges_m=lidar_ranges,
                lidar_angles_rad=lidar_angles,
                barrier_possible=barrier_possible,
            )
            if obs is not None:
                result.append(obs)
        return result
