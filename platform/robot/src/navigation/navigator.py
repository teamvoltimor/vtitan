"""WRO Track Navigator — Refactored to use focused controllers.

Orchestrates navigation by wiring the CoreNavigator (pure Python)
with the ROS2 ecosystem via the HardwareGateway protocol.

Usage:
    python main.py --metadata scenario_0000_metadata.json --laps 3
"""

from __future__ import annotations

import json
import logging
import math
from typing import TYPE_CHECKING, Any

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from shared.config.constants import DictKeys, RobotSpecs
from shared.config.enums import ScenarioType
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose, Velocity
from std_msgs.msg import String

from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.enums import Direction, Section
from src.hardware.gateway import HardwareGateway
from src.navigation.core_navigator import CoreNavigator
from src.navigation.parking import ParkController, park_controller_from_metadata
from src.navigation.race_tracker import LapDetector
from src.navigation.sign_router import SignRouter, signs_from_metadata
from src.navigation.waypoints import calculate_waypoints
from src.state_machine.estimator import StateEstimator

if TYPE_CHECKING:
    from pathlib import Path

logger = logging.getLogger(__name__)


def _load_json(path: str | Path) -> dict[str, Any]:
    """Load JSON file."""
    if isinstance(path, str):
        from pathlib import Path

        path = Path(path)
    with path.open(encoding="utf-8") as f:
        return json.load(f)


class ROS2HardwareGateway(HardwareGateway):
    """Implementation of HardwareGateway for ROS2 environment.

    Acts as an adapter between the ROS2 Node topics and the pure Python
    CoreNavigator logic.
    """

    def __init__(self, node: Node, start_x: float, start_y: float, start_yaw: float) -> None:
        self._node = node
        self._estimator = StateEstimator(start_x, start_y, start_yaw)
        self._latest_lidar: tuple[list[float], list[float]] | None = None
        self._latest_detections: list[Detection] = []
        self._latest_imu: IMUReading | None = None

        # Publishers
        self._vel_publisher = node.create_publisher(
            Twist,
            node.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10,
        )

        # Subscribers
        node.create_subscription(
            Odometry,
            node.get_parameter("odom_topic").get_parameter_value().string_value,
            self._odom_callback,
            10,
        )
        node.create_subscription(
            LaserScan,
            node.get_parameter("lidar_topic").get_parameter_value().string_value,
            self._lidar_callback,
            10,
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
            10,
        )

    def _imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        reading = IMUReading(yaw=yaw, pitch=0.0, roll=0.0)
        self._latest_imu = reading
        self._estimator.update_imu(reading)

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._estimator.update_odom(x, y, yaw)

    def _lidar_callback(self, msg: LaserScan) -> None:
        raw = np.array(msg.ranges)
        # Assuming clamp_lidar_scan is no longer needed since controller handles it
        # or we clamp it here
        raw = np.clip(raw, 0.0, RobotSpecs.LIDAR_MAX_RANGE)
        raw[np.isinf(raw)] = RobotSpecs.LIDAR_MAX_RANGE
        angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)).tolist()
        self._latest_lidar = (raw.tolist(), angles)

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

    def publish_velocity(self, velocity: Velocity) -> None:
        """Publish velocity command to ROS2 cmd_vel topic."""
        msg = Twist()
        msg.linear.x = float(velocity.linear)
        msg.angular.z = float(velocity.angular)
        self._vel_publisher.publish(msg)

    def get_current_pose(self) -> Pose | None:
        """Get the latest fused pose."""
        return self._estimator.estimate_pose()

    def get_lidar_scan(self) -> tuple[list[float], list[float]] | None:
        """Get the latest processed LIDAR scan."""
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

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/wro_robot/cmd_vel")
        self.declare_parameter("odom_topic", "/wro_robot/odom")
        self.declare_parameter("lidar_topic", "/lidar")
        self.declare_parameter("vision_topic", "/vision/detections")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("is_simulation", value=False)

        if params_path is not None:
            self._apply_param_overrides(params_path)

        # Setup Tuning
        tuning = NavigationTuning.load_from_yaml(tuning_path) if tuning_path else NavigationTuning()

        # Gateway & Core Logic
        self._gateway = ROS2HardwareGateway(self, start_x, start_y, start_yaw)
        waypoints = calculate_waypoints(self._metadata, num_laps=1)

        sign_router: SignRouter | None = None
        if not self._is_open_challenge:
            signs = signs_from_metadata(self._metadata)
            if signs:
                sign_router = SignRouter(signs)

        start_section = Section.from_string(start_cond[DictKeys.SECTION])
        start_direction = Direction.from_string(start_cond[DictKeys.DIRECTION])
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
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))
        except ValueError as e:
            self.get_logger().error(f"Value error in control loop: {e}")
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))

    def _apply_param_overrides(self, params_path: str | Path) -> None:
        """Load a JSON file of {param_name: value} overrides and apply to this node."""
        import rclpy.parameter as rp

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
                params.append(rp.Parameter(name, rp.Parameter.Type.INTEGER, value))
            elif isinstance(value, float):
                params.append(rp.Parameter(name, rp.Parameter.Type.DOUBLE, value))
            elif isinstance(value, str):
                params.append(rp.Parameter(name, rp.Parameter.Type.STRING, value))
            else:
                self.get_logger().warning(f"Skipping param '{name}': unsupported type {type(value)}")

        if params:
            self.set_parameters(params)
            self.get_logger().info(f"Applied {len(params)} param override(s) from {params_path}")
