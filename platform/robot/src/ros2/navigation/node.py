"""WRO Track Navigator — Refactored to use focused controllers.

Orchestrates navigation by wiring the CoreNavigator (pure Python)
with the ROS2 ecosystem via the HardwareGateway protocol.

Usage:
    python main.py --metadata scenario_0000_metadata.json --laps 3
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Any, cast

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.constants import DictKeys, RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning, SensorHealthParams
from shared.domain.enums import RobotState
from shared.domain.models import Detection, IMUReading, Pose
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import follow_corridor
from src.navigation.direction_estimator import DirectionEstimator
from src.navigation.localization import LidarLocalizer
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand, HardwareGateway, LidarScan, WheelOdometry
from src.navigation.race_tracker import LapDetector
from src.navigation.start_conditions import assumed_start_conditions
from src.navigation.track_geometry import TrackWalls, corridor_widths_from_metadata
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.state_machine.estimator import StateEstimator

logger = logging.getLogger(__name__)

_DRIVE_JOINT = "drive_wheel"
"""Drive-wheel joint name on /joint_states.

Must match ackermann_motor_node's ``_DRIVE_JOINT``; the topic-contract test
pins the two together, since a rename on either side would otherwise just stop
producing odometry with no error anywhere.
"""


def _load_json(path: str | Path) -> dict[str, Any]:
    """Load JSON file."""
    p = Path(path) if isinstance(path, str) else path
    with p.open(encoding="utf-8") as f:
        return cast("dict[str, Any]", json.load(f))


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
    return node.get_parameter(name).get_parameter_value().string_value


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
        corridor_widths_m: dict[Section, float],
        stale_timeout_sec: float = SensorHealthParams().STALE_TIMEOUT_SEC,
    ) -> None:
        self._node = node
        self._estimator = StateEstimator(start_x, start_y, start_yaw)
        self._localizer = LidarLocalizer(TrackWalls(corridor_widths_m))
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
        self._localizer = LidarLocalizer(walls)

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
            i = msg.name.index(_DRIVE_JOINT)
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
        angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)).tolist()
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
            raw_data = json.loads(msg.data)
            self._latest_detections = []
            for d in raw_data:
                # Need to map detection dict to Detection dataclass
                self._latest_detections.append(
                    Detection(
                        class_name=d.get("class_name", ""),
                        confidence=d.get("confidence", 0.0),
                        bbox=d.get("bbox", (0.0, 0.0, 0.0, 0.0)),
                        x=d.get("x", 0.0),
                        y=d.get("y", 0.0),
                        width=d.get("width", 0.0),
                        height=d.get("height", 0.0),
                        area=d.get("area", 0.0),
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

    def get_vision_detections(self) -> list[Detection]:
        """Get the latest parsed vision detections."""
        return self._latest_detections


class TrackNavigator(Node):
    """ROS2 node wrapping the pure Python CoreNavigator."""

    def __init__(
        self,
        metadata_path: str | Path | None = None,
        num_laps: int = 3,
        params_path: str | Path | None = None,
        tuning_path: str | Path | None = None,
        blind: bool = False,
        direction: Direction = Direction.CLOCKWISE,
    ) -> None:
        """Drive the track.

        Args:
            metadata_path: Scenario metadata. Omit it entirely to run with no
                scenario file at all, which is what competition requires: the
                robot knows the corridor width *possibilities* (60 or 100 cm)
                but not which corridor is which, and no file describing the
                round exists on the day. Omitting it implies ``blind``.
            num_laps: Laps to complete.
            params_path: Optional JSON of ROS parameter overrides.
            tuning_path: Optional navigation tuning YAML.
            blind: Estimate the corridor layout from LIDAR instead of being
                told it. Implied when ``metadata_path`` is omitted, since there
                is then nothing to be told.
            direction: Travel direction for the round. This is the one starting
                condition that cannot be assumed -- the section can, because
                assuming it only rotates the robot's private world frame, but
                the direction is a reflection and no amount of width learning
                recovers from getting it wrong. See
                :mod:`src.navigation.start_conditions`. Ignored when
                ``metadata_path`` supplies one.
        """
        super().__init__("track_navigator")

        # No file means nothing to be sighted with.
        self._blind = blind or metadata_path is None
        self._metadata = (
            _load_json(metadata_path)
            if metadata_path is not None
            else {DictKeys.STARTING_CONDITIONS: assumed_start_conditions(direction)}
        )
        self._is_open_challenge = self._metadata.get(DictKeys.CHALLENGE_TYPE, ScenarioType.OPEN) == ScenarioType.OPEN

        # Start conditions
        start_cond = self._metadata[DictKeys.STARTING_CONDITIONS]
        start_x = start_cond[DictKeys.POSITION][DictKeys.X]
        start_y = start_cond[DictKeys.POSITION][DictKeys.Y]
        start_yaw = start_cond[DictKeys.YAW]

        # Parameters. Defaults match the topics the deployed nodes actually
        # use (ackermann_motor_node's /ackermann_cmd, sllidar_ros2's /scan) —
        # not the pre-Ackermann-migration /wro_robot/cmd_vel and /lidar names
        # this node previously assumed. There is no odom_topic: no node
        # publishes nav_msgs/Odometry on real hardware, so position comes
        # from LIDAR localization instead (see ROS2HardwareGateway).
        self.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("vision_topic", "/vision/detections")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("is_simulation", value=False)

        if params_path is not None:
            self._apply_param_overrides(params_path)

        # Setup Tuning
        tuning = NavigationTuning.load_from_yaml(tuning_path) if tuning_path else NavigationTuning()

        start_section = Section.from_string(start_cond[DictKeys.SECTION])
        start_direction = Direction.from_string(start_cond[DictKeys.DIRECTION])

        # What the robot is allowed to believe about the layout. Sighted runs
        # read it from the metadata; blind runs start with every corridor
        # assumed narrow and correct it from LIDAR as they drive.
        #
        # Narrow is the safe prior: planning a 1.0 m corridor as if it were
        # 0.6 m puts the path nearer the outer wall, which is still inside it.
        # The converse puts the path 0.15 m from the inner block face, inside
        # the chassis half-diagonal, and clips it mid-turn.
        self._arc_radius = tuning.waypoints.ARC_RADIUS
        self._direction = start_direction
        self._start_xy = (start_x, start_y)
        self._start_section = start_section
        self._width_estimator = CorridorWidthEstimator() if self._blind else None
        # Blind implies inferring the direction: it is drawn at random on the
        # day, so a blind robot cannot be handed it either. ``direction`` is
        # only the provisional the first path is built from, and is replaced
        # the moment the inference settles.
        self._direction_estimator = DirectionEstimator() if self._blind else None
        self._creep_widths: list[tuple[float, float]] = []
        self._creep_speed = tuning.speed.SLOW_SPEED
        self._told_widths = corridor_widths_from_metadata(self._metadata) if not self._blind else {}
        corridor_widths_m = self._width_estimator.widths if self._width_estimator else self._told_widths

        self._gateway = ROS2HardwareGateway(
            self,
            start_x,
            start_y,
            start_yaw,
            corridor_widths_m,
            stale_timeout_sec=tuning.sensor.STALE_TIMEOUT_SEC,
        )
        waypoints = self._plan(corridor_widths_m)

        sign_router: SignRouter | None = None
        if not self._is_open_challenge:
            signs = signs_from_metadata(self._metadata)
            if signs:
                sign_router = SignRouter(signs, direction=start_direction)

        lap_detector = LapDetector(
            start_pos=(start_x, start_y),
            start_section=start_section,
            direction=start_direction,
        )

        park_controller: ParkController | None = None
        if not self._is_open_challenge:
            park_controller = park_controller_from_metadata(
                self._metadata,
                start_section,
                start_direction,
            )

        self._core_navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=waypoints,
            num_laps=num_laps,
            tuning=tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=park_controller,
        )

        # Race-state gate. Without this the navigator drives the moment it has a
        # pose -- before the start button is pressed, and straight through an
        # E-STOP, since stopping the state machine does not stop this node. The
        # button is the operator's only physical control, so it has to gate the
        # thing that actually moves the robot.
        #
        # TRANSIENT_LOCAL matches state_machine_node's /robot_state publisher, so
        # the current state arrives immediately rather than only on the next
        # transition -- otherwise launching mid-race would sit idle until the
        # state happened to change.
        self._racing = False
        self.create_subscription(
            String,
            "/robot_state",
            self._on_robot_state,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Control Loop
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info(
            f"Navigator ready: {len(waypoints)} waypoints, {num_laps} lap(s) - "
            "holding until /robot_state reports racing",
        )

    def _resolve_direction(self) -> bool:
        """Creep along the corridor until the travel direction is inferable.

        Returns:
            ``True`` while the direction is still unknown, meaning this tick
            was driven by the corridor follower and there is no plan to step.
        """
        estimator = self._direction_estimator
        if estimator is None or estimator.is_settled:
            return False

        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            return True

        # Width readings taken now cannot be filed under a corridor yet -- that
        # needs the direction -- but they are the cleanest of the round, taken
        # driving straight down a corridor. Buffer and replay them, or the
        # first surviving readings are taken at a corner where the side rays
        # span the *next* corridor and get attributed to this one.
        if self._width_estimator is not None:
            measured = measure_corridor_width(scan.ranges_m, scan.angles_rad, pose.yaw)
            if measured is not None:
                self._creep_widths.append((pose.yaw, measured))

        if estimator.observe(scan.ranges_m, scan.angles_rad, pose.yaw):
            inferred = estimator.direction
            if inferred is not None:
                self._commit_direction(inferred, pose)
            return False

        self._gateway.publish_drive(follow_corridor(scan.ranges_m, scan.angles_rad, self._creep_speed))
        return True

    def _commit_direction(self, inferred: Direction, pose: Pose) -> None:
        """Adopt the inferred direction and rebuild everything derived from it."""
        changed = inferred is not self._direction
        self._direction = inferred
        if self._width_estimator is not None:
            for buffered_yaw, buffered_width in self._creep_widths:
                self._width_estimator.observe_measurement(
                    section_from_heading(buffered_yaw, inferred),
                    buffered_width,
                )
            self._creep_widths.clear()
            self._gateway.set_believed_walls(TrackWalls(self._width_estimator.widths))
        if changed:
            # The finish line's normal is the travel direction, so a detector
            # built for the provisional one counts crossings inverted.
            self._core_navigator.replace_lap_detector(
                LapDetector(
                    start_pos=(self._start_xy),
                    start_section=self._start_section,
                    direction=inferred,
                ),
            )
        widths = self._width_estimator.widths if self._width_estimator else self._told_widths
        # Resync unconditionally: the navigator did not step during the creep,
        # so its waypoint index is still 0 while the robot has driven a metre
        # past it, and it would resume by chasing a waypoint behind itself.
        self._core_navigator.replace_path(self._plan(widths), (pose.x, pose.y))
        self.get_logger().info(f"Travel direction inferred from LIDAR: {inferred}")

    def _plan(self, widths: dict[Section, float]) -> list[tuple[float, float]]:
        """Build a one-lap path for the layout the robot believes it is on.

        num_laps=1 is intentional: calculate_waypoints bakes the lap count into
        the list, but CoreNavigator already cycles one canonical lap `num_laps`
        times (see step() waypoint-wrap). Passing the real count would multiply
        laps (e.g. 3 -> 9). Keep this at 1.
        """
        planning_metadata = dict(self._metadata)
        planning_metadata[DictKeys.CORRIDOR_WIDTHS] = {
            section.value: {DictKeys.WIDTH_MM: round(width * 1000)} for section, width in widths.items()
        }
        # The travel direction has to be overridden too, not just the widths.
        # calculate_waypoints reads it from starting_conditions, so a path
        # replanned after the direction was inferred would otherwise still run
        # the provisional way round the loop -- and the robot would drive it.
        start_conditions = dict(planning_metadata[DictKeys.STARTING_CONDITIONS])
        start_conditions[DictKeys.DIRECTION] = str(self._direction)
        planning_metadata[DictKeys.STARTING_CONDITIONS] = start_conditions
        return calculate_waypoints(planning_metadata, num_laps=1, arc_radius=self._arc_radius)

    def _update_layout_belief(self) -> bool:
        """Fold the latest scan into the width estimate; replan if it moved.

        Returns:
            ``True`` if the belief changed and the path was rebuilt.
        """
        estimator = self._width_estimator
        if estimator is None:
            return False
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return False

        # Attribute the reading by HEADING, not position. Position would be
        # circular -- it comes from matching against a wall model built from the
        # very widths being estimated, so a wrong belief mis-attributes the
        # reading that would have corrected it and the error locks in. Heading
        # comes from the IMU and owes nothing to the map.
        section = section_from_heading(pose.yaw, self._direction)
        if not estimator.observe(section, scan.ranges_m, scan.angles_rad, pose.yaw):
            return False

        believed = estimator.widths
        self._gateway.set_believed_walls(TrackWalls(believed))
        self._core_navigator.replace_path(self._plan(believed), (pose.x, pose.y))
        self.get_logger().info(
            "Layout belief updated: "
            + ", ".join(f"{s.value}={w * 100:.0f}cm" for s, w in sorted(believed.items(), key=lambda kv: kv[0].value)),
        )
        return True

    def _on_robot_state(self, msg: String) -> None:
        """Track whether the state machine says we are racing."""
        was_racing = self._racing
        self._racing = msg.data.strip().lower() == RobotState.RACING.value
        if was_racing and not self._racing:
            # Left RACING (finished, or E-STOP). Command a stop immediately
            # rather than waiting for the next control tick.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self.get_logger().info(f"Race state '{msg.data}' - navigator holding, motors stopped")
        elif not was_racing and self._racing:
            # Re-zero heading here, not at startup. RVC yaw is relative to
            # power-on, and the robot is carried to the track after that, so the
            # offset latched by the first IMU reading refers to whatever
            # orientation it happened to be held in. This is the one instant the
            # robot is known to be in its starting pose.
            self._gateway.reset_heading_reference()
            self.get_logger().info("Race started - heading reference zeroed, navigator driving")

    def _control_loop(self) -> None:
        """Execute one control step, or hold the robot stopped when not racing."""
        if not self._racing:
            # Keep publishing zeros rather than going silent: ackermann_motor_node
            # has a 1 s command watchdog, and silence would let it latch a stop
            # only after that delay.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            return
        try:
            if self._resolve_direction():
                # Direction unknown: the corridor follower drove this tick and
                # there is no usable plan to step yet.
                return
            if self._blind:
                self._update_layout_belief()
            self._core_navigator.step()
        except RuntimeError as e:
            self.get_logger().error(f"Runtime error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        except ValueError as e:
            self.get_logger().error(f"Value error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))

    def _apply_param_overrides(self, params_path: str | Path) -> None:
        """Load a JSON file of {param_name: value} overrides and apply to this node."""
        import rclpy.parameter as rp  # noqa: PLC0415

        try:
            data = _load_json(params_path)
        except FileNotFoundError as e:
            self.get_logger().error(f"Param file not found: {e}")
            return
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode params JSON: {e}")
            return

        params = []
        for name, value in data.items():
            if isinstance(value, bool):
                params.append(rp.Parameter(name, rp.Parameter.Type.BOOL, value))
            elif isinstance(value, int):
                # rclpy's Parameter stub only resolves the bool overload;
                # runtime dispatch is on the Type enum, not the stub's
                # positional-arg overload, so this is a stub limitation.
                params.append(rp.Parameter(name, rp.Parameter.Type.INTEGER, value))  # type: ignore[arg-type]
            elif isinstance(value, float):
                params.append(rp.Parameter(name, rp.Parameter.Type.DOUBLE, value))  # type: ignore[arg-type]
            elif isinstance(value, str):
                params.append(rp.Parameter(name, rp.Parameter.Type.STRING, value))  # type: ignore[arg-type]
            else:
                self.get_logger().warning(f"Skipping param '{name}': unsupported type {type(value)}")

        if params:
            self.set_parameters(params)
            self.get_logger().info(f"Applied {len(params)} param override(s) from {params_path}")


def main(args: list[str] | None = None) -> None:
    """Run the ROS2 track navigator node (``ros2 run voldemorbot_navigation track_navigator_node``)."""
    parser = argparse.ArgumentParser(description="WRO 2026 track navigator ROS2 node.")
    parser.add_argument(
        "--metadata",
        help="Path to scenario metadata JSON. Omit to run with no scenario file at "
        "all, which is what competition requires -- the layout is then estimated "
        "from LIDAR and --direction supplies the only start condition that cannot "
        "be assumed.",
    )
    parser.add_argument("--laps", type=int, default=3, help="Laps to complete (default: 3).")
    parser.add_argument("--params", help="Optional navigator_params.json for runtime overrides.")
    parser.add_argument("--tuning", help="Optional navigation tuning YAML.")
    parser.add_argument(
        "--blind",
        action="store_true",
        help="Estimate the corridor layout from LIDAR instead of reading it from "
        "--metadata, which is what competition requires: WRO randomises the inner "
        "walls before each round, so the widths in a file cannot be known on the "
        "day. Only the start conditions are then read from --metadata.",
    )
    parser.add_argument(
        "--direction",
        choices=["cw", "ccw"],
        default="cw",
        help="Travel direction for the round (default: cw). Used only when --metadata "
        "is omitted. The starting section does not need one of these because assuming "
        "it merely rotates the robot's own world frame; the direction is a reflection "
        "and has to be right.",
    )
    parsed, _ = parser.parse_known_args(args)

    metadata_path: Path | None = None
    if parsed.metadata:
        metadata_path = Path(parsed.metadata)
        if not metadata_path.exists():
            logger.error("Metadata file not found: %s", metadata_path)
            raise SystemExit(1)

    rclpy.init(args=args)
    navigator: TrackNavigator | None = None
    try:
        navigator = TrackNavigator(
            metadata_path=metadata_path,
            num_laps=parsed.laps,
            params_path=parsed.params,
            tuning_path=parsed.tuning,
            blind=parsed.blind,
            direction=Direction.CLOCKWISE if parsed.direction == "cw" else Direction.COUNTERCLOCKWISE,
        )
        while rclpy.ok() and not getattr(navigator, "shutdown_requested", False):
            rclpy.spin_once(navigator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
