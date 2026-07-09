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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from shared.config.constants import DictKeys, RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning, SensorHealthParams
from shared.domain.models import Detection, IMUReading, Pose
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.navigation.core_navigator import CoreNavigator
from src.navigation.localization import LidarLocalizer
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand, HardwareGateway, LidarScan
from src.navigation.race_tracker import LapDetector
from src.navigation.track_geometry import TrackWalls, corridor_widths_from_metadata
from src.state_machine.estimator import StateEstimator

logger = logging.getLogger(__name__)


def _load_json(path: str | Path) -> dict[str, Any]:
    """Load JSON file."""
    p = Path(path) if isinstance(path, str) else path
    with p.open(encoding="utf-8") as f:
        return cast("dict[str, Any]", json.load(f))


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
        # Receipt timestamp (seconds) for staleness / dropout detection. LIDAR
        # is the position source (no wheel odometry exists on real hardware),
        # so its staleness gates get_current_pose() too.
        self._lidar_stamp: float | None = None
        self._stale_timeout_sec = stale_timeout_sec

        # Publishers
        self._drive_publisher = node.create_publisher(
            AckermannDriveStamped,
            node.get_parameter("ackermann_cmd_topic").get_parameter_value().string_value,
            10,
        )

        # Subscribers
        node.create_subscription(
            LaserScan,
            node.get_parameter("lidar_topic").get_parameter_value().string_value,
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            String,
            node.get_parameter("vision_topic").get_parameter_value().string_value,
            self._vision_callback,
            10,
        )
        node.create_subscription(
            Imu,
            node.get_parameter("imu_topic").get_parameter_value().string_value,
            self._imu_callback,
            qos_profile_sensor_data,
        )

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

        # No wheel odometry exists on real hardware, so this scan is also the
        # position source: match it against the known wall geometry, seeded
        # from the previous estimate (the robot moves only centimetres between
        # scans, so that prior is always a tight, reliable search start).
        prior_pose = self._estimator.estimate_pose()
        est_x, est_y = self._localizer.estimate_position(
            (prior_pose.x, prior_pose.y), prior_pose.yaw, raw.tolist(), angles,
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
            command.steering_norm, RobotSpecs.MAX_STEERING_ANGLE,
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
        metadata_path: str | Path,
        num_laps: int = 3,
        params_path: str | Path | None = None,
        tuning_path: str | Path | None = None,
    ) -> None:
        super().__init__("track_navigator")

        self._metadata = _load_json(metadata_path)
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
        self.declare_parameter("is_simulation", value=False)

        if params_path is not None:
            self._apply_param_overrides(params_path)

        # Setup Tuning
        tuning = NavigationTuning.load_from_yaml(tuning_path) if tuning_path else NavigationTuning()

        # Gateway & Core Logic
        corridor_widths_m = corridor_widths_from_metadata(self._metadata)
        self._gateway = ROS2HardwareGateway(
            self,
            start_x,
            start_y,
            start_yaw,
            corridor_widths_m,
            stale_timeout_sec=tuning.sensor.STALE_TIMEOUT_SEC,
        )
        # num_laps=1 is intentional: calculate_waypoints bakes the lap count into
        # the list, but CoreNavigator already cycles one canonical lap `num_laps`
        # times (see step() waypoint-wrap). Passing the real count would multiply
        # laps (e.g. 3 -> 9). Keep this at 1.
        waypoints = calculate_waypoints(self._metadata, num_laps=1, arc_radius=tuning.waypoints.ARC_RADIUS)

        start_section = Section.from_string(start_cond[DictKeys.SECTION])
        start_direction = Direction.from_string(start_cond[DictKeys.DIRECTION])

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
            park_controller = park_controller_from_metadata(self._metadata, start_section)

        self._core_navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=waypoints,
            num_laps=num_laps,
            tuning=tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=park_controller,
        )

        # Control Loop
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info(f"Navigator ready: {len(waypoints)} waypoints, {num_laps} lap(s)")

    def _control_loop(self) -> None:
        """Execute one control step."""
        try:
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
    parser.add_argument("--metadata", required=True, help="Path to scenario metadata JSON.")
    parser.add_argument("--laps", type=int, default=3, help="Laps to complete (default: 3).")
    parser.add_argument("--params", help="Optional navigator_params.json for runtime overrides.")
    parser.add_argument("--tuning", help="Optional navigation tuning YAML.")
    parsed, _ = parser.parse_known_args(args)

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
