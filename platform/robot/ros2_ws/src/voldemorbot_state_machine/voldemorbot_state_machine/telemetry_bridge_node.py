"""ROS2 Telemetry Bridge — Publishes robot data to FastAPI backend."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from typing import Any

import rclpy
import requests
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.coordinate_transform import quaternion_to_yaw
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

_BACKOFF_INITIAL = 1.0
_BACKOFF_MAX = 60.0

_MIN_TIMESTAMPS_FOR_RATE = 2
"""Minimum tracked timestamps needed to compute a topic update rate."""

_MIN_POINTS_FOR_FORWARD_WINDOW = 20
"""Minimum LIDAR points needed to safely slice the +/-10-index forward window."""

_MIN_POINTS_FOR_SIDE_WINDOW = 4
"""Minimum LIDAR points needed to safely slice the quarter-arc left/right windows."""

_MIN_POINTS_FOR_BACK_WINDOW = 16
"""Minimum LIDAR points needed to safely slice the n/8 back window."""

_MIN_VALID_LIDAR_RANGE_M = 0.01
"""LIDAR ranges at or below this are treated as invalid (no-return) readings.

Relocated verbatim from oled_display_node.py, which used to compute this
itself from the raw /scan topic. That direct subscription (along with
/imu/data and /hailo/detections) got replaced by this node publishing a
single low-rate /ui/telemetry_summary instead -- the Pi Zero's USB-gadget
link and single weak core were paying for the full sensor-rate traffic to
feed a 128x64 display that only ever samples it a couple times a second.
"""


@dataclass(frozen=True, slots=True)
class _IMUPayload:
    """IMU section of a RobotSnapshot."""

    linearAcceleration: list[float]
    angularVelocity: list[float]
    orientationQuaternion: list[float]


@dataclass(frozen=True, slots=True)
class _MotorStatePayload:
    """Motor-state section of a RobotSnapshot."""

    steeringAngle: float
    driveSpeed: float
    encoderPosition: int


@dataclass(frozen=True, slots=True)
class _VisionDetectionPayload:
    """A single Hailo detection, as reported in a RobotSnapshot."""

    className: int | str
    confidence: float
    bbox: list[float]


@dataclass(frozen=True, slots=True)
class TelemetryMetrics:
    """Backend-facing telemetry metrics payload (POSTed as part of RobotSnapshot)."""

    timestamp: float
    nodeHealth: str
    pointsCaptured: int
    rangeMin: float | None
    rangeMax: float | None
    rangeMean: float | None
    forward: float | None
    left: float | None
    right: float | None
    back: float | None
    speed: float | None
    stage: str
    lidarAvailable: bool
    imuAvailable: bool
    cameraAvailable: bool
    odometryAvailable: bool


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """Backend-facing telemetry snapshot POSTed to /telemetry/record."""

    timestamp: float
    missionName: str
    robotPosition: list[float] | None
    robotOrientation: float | None
    lidarPoints: list[list[float]]
    pathHistory: list[list[float]]
    logs: list[str]
    metrics: TelemetryMetrics
    imuData: _IMUPayload | None
    visionDetections: list[_VisionDetectionPayload] | None
    motorState: _MotorStatePayload | None


@dataclass(frozen=True, slots=True)
class _TopicUpdatePayload:
    """Per-topic diagnostic entry in a TopicsSnapshot.

    "data" is a recursive reflection of an arbitrary ROS2 message's __slots__
    (see _msg_to_dict) -- its shape depends on which message type was passed,
    so it has no fixed schema and stays a plain dict.
    """

    topicName: str
    messageType: str
    timestamp: float
    updateRateHz: float
    data: dict[str, Any]


@dataclass(frozen=True, slots=True)
class TopicsSnapshot:
    """Backend-facing topic-health snapshot POSTed to /telemetry/topics/update."""

    timestamp: float
    topics: list[_TopicUpdatePayload]


class RosTopic:
    """ROS2 topic name constants used by the telemetry bridge."""

    SCAN = "/scan"
    ODOM = "/odom"
    IMU = "/imu/data"
    STATE = "/state_machine/state"
    CMD_VEL = "/cmd_vel"
    JOINT_STATES = "/joint_states"
    HAILO_DETECTIONS = "/hailo/detections"


class RosMsgType:
    """ROS2 message type string constants."""

    LASER_SCAN = "sensor_msgs/LaserScan"
    ODOMETRY = "nav_msgs/Odometry"
    IMU = "sensor_msgs/Imu"
    STRING = "std_msgs/String"
    TWIST = "geometry_msgs/Twist"
    JOINT_STATE = "sensor_msgs/JointState"
    DETECTION_2D_ARRAY = "vision_msgs/Detection2DArray"


def _lidar_clearances_cm(ranges: list[float]) -> tuple[float, float, float]:
    """Front/left/right clearances in cm for the OLED's RACING page.

    Relocated verbatim from oled_display_node.py's old _lidar_callback --
    deliberately a different windowing scheme than _build_metrics' own
    forward/left/right/back calc below (different indices, meters not cm,
    different consumer). Don't merge the two.
    """
    num_points = len(ranges)
    if num_points == 0:
        return 0.0, 0.0, 0.0

    front_indices = range(num_points // 2 - 20, num_points // 2 + 20)
    front_ranges = [ranges[i] for i in front_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M]
    front = min(front_ranges) * 100 if front_ranges else 0.0

    left_indices = range(num_points // 4, num_points // 3)
    left_ranges = [ranges[i] for i in left_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M]
    left = min(left_ranges) * 100 if left_ranges else 0.0

    right_indices = range(2 * num_points // 3, 3 * num_points // 4)
    right_ranges = [ranges[i] for i in right_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M]
    right = min(right_ranges) * 100 if right_ranges else 0.0

    return front, left, right


def _best_detection(msg: Detection2DArray) -> tuple[str, float] | None:
    """Track the single most salient detection for the OLED's RACING page.

    Relocated verbatim from oled_display_node.py's old _detections_callback.
    Ranked by confidence x bbox area rather than confidence alone: a small,
    high-confidence false positive and a large, low-confidence smear are
    both less trustworthy than one detection that scores well on both axes.
    """
    best: tuple[str, float] | None = None
    best_score = -1.0
    for det in msg.detections:
        if not det.results:
            continue
        hyp = det.results[0]
        if hasattr(hyp, "hypothesis"):
            class_id, confidence = hyp.hypothesis.class_id, hyp.hypothesis.score
        else:
            class_id, confidence = hyp.id, hyp.score
        area = det.bbox.size_x * det.bbox.size_y
        score = confidence * area
        if score > best_score:
            best_score = score
            best = (class_id, confidence)
    return best


class TelemetryBridgeNode(Node):
    """Subscribes to robot topics and POSTs telemetry to backend API."""

    def __init__(self) -> None:
        super().__init__("telemetry_bridge")

        # Configuration
        self.declare_parameter("backend_url", "http://localhost:8010")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_path_history", 120)
        # Independent from publish_rate_hz above: that one drives the HTTP
        # POST to the backend over WiFi/LAN. This one drives a small JSON
        # blob to the Pi Zero over the USB-gadget link -- a 128x64 display
        # doesn't need fresher-than-2Hz numbers, and keeping it decoupled
        # means tuning one doesn't silently affect the other.
        self.declare_parameter("ui_summary_rate_hz", 2.0)

        self._backend_url = self.get_parameter("backend_url").value
        self._rate = self.get_parameter("publish_rate_hz").value
        self._max_history = self.get_parameter("max_path_history").value
        self._ui_summary_rate = self.get_parameter("ui_summary_rate_hz").value

        # Subscriptions — sensor topics use qos_profile_sensor_data to match
        # the BEST_EFFORT QoS that hardware drivers publish with.
        self.create_subscription(LaserScan, RosTopic.SCAN, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, RosTopic.ODOM, self._odom_callback, 10)
        self.create_subscription(Imu, RosTopic.IMU, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(String, RosTopic.STATE, self._state_callback, 10)
        self.create_subscription(Twist, RosTopic.CMD_VEL, self._cmd_vel_callback, 10)
        self.create_subscription(JointState, RosTopic.JOINT_STATES, self._joint_callback, 10)
        self.create_subscription(
            Detection2DArray,
            RosTopic.HAILO_DETECTIONS,
            self._vision_callback,
            qos_profile_sensor_data,
        )

        # Latest data cache
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._latest_imu: Imu | None = None
        self._latest_state: str = "unknown"
        self._latest_cmd_vel: Twist | None = None
        self._latest_joints: JointState | None = None
        self._latest_vision: Detection2DArray | None = None

        self._topic_updates: dict[str, _TopicUpdatePayload] = {}
        self._topic_timestamps: dict[str, deque] = {}

        self._path_history: deque = deque(maxlen=self._max_history)
        self._logs: deque = deque(maxlen=10)

        # Backend-status publisher (reuses system_status topic).
        self._system_status_pub = self.create_publisher(DiagnosticArray, "/system_status", 10)

        # Low-rate lidar/yaw/detection summary for the Pi Zero's OLED --
        # the only sensor telemetry it needs, so it doesn't have to
        # subscribe to /scan, /imu/data and /hailo/detections directly.
        self._ui_summary_pub = self.create_publisher(String, "/ui/telemetry_summary", 10)

        # HTTP: single session for connection pooling; backoff state.
        self._session = requests.Session()
        self._backend_down = False
        self._next_retry_time: float = 0.0
        self._backoff_delay: float = _BACKOFF_INITIAL

        # Timer for publishing
        self.create_timer(1.0 / self._rate, self._publish_telemetry)
        self.create_timer(1.0 / self._ui_summary_rate, self._publish_ui_summary)

        self.get_logger().info(f"Telemetry bridge started → {self._backend_url}")

    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._update_raw_topic(RosTopic.SCAN, RosMsgType.LASER_SCAN, msg)

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._update_raw_topic(RosTopic.ODOM, RosMsgType.ODOMETRY, msg)
        pos = msg.pose.pose.position
        self._path_history.append([pos.x, pos.y, pos.z])

    def _imu_callback(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._update_raw_topic(RosTopic.IMU, RosMsgType.IMU, msg)

    def _state_callback(self, msg: String) -> None:
        self._latest_state = msg.data
        self._update_raw_topic(RosTopic.STATE, RosMsgType.STRING, msg)
        self._logs.append(f"State: {msg.data}")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_cmd_vel = msg
        self._update_raw_topic(RosTopic.CMD_VEL, RosMsgType.TWIST, msg)

    def _joint_callback(self, msg: JointState) -> None:
        self._latest_joints = msg
        self._update_raw_topic(RosTopic.JOINT_STATES, RosMsgType.JOINT_STATE, msg)

    def _vision_callback(self, msg: Detection2DArray) -> None:
        self._latest_vision = msg
        self._update_raw_topic(RosTopic.HAILO_DETECTIONS, RosMsgType.DETECTION_2D_ARRAY, msg)

    def _update_raw_topic(self, topic_name: str, msg_type: str, msg: object) -> None:
        current_time = time.time()

        if topic_name not in self._topic_timestamps:
            self._topic_timestamps[topic_name] = deque(maxlen=10)

        self._topic_timestamps[topic_name].append(current_time)
        timestamps = list(self._topic_timestamps[topic_name])

        if len(timestamps) >= _MIN_TIMESTAMPS_FOR_RATE:
            time_diff = timestamps[-1] - timestamps[0]
            update_rate = (len(timestamps) - 1) / time_diff if time_diff > 0 else 0.0
        else:
            update_rate = 0.0

        msg_dict = self._msg_to_dict(msg)

        self._topic_updates[topic_name] = _TopicUpdatePayload(
            topicName=topic_name,
            messageType=msg_type,
            timestamp=current_time,
            updateRateHz=round(update_rate, 2),
            data=msg_dict,
        )

    def _msg_to_dict(self, msg: object) -> dict[str, Any]:
        """Recursively reflect an arbitrary ROS2 message's __slots__ into a dict.

        Shape depends on the message type passed in, so it has no fixed schema
        -- this is intentionally dict[str, Any], not a TypedDict.
        """
        result: dict[str, Any] = {}
        if hasattr(msg, "__slots__"):
            for field in msg.__slots__:
                value = getattr(msg, field, None)
                if hasattr(value, "__slots__"):
                    result[field] = self._msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    if len(value) > 0 and hasattr(value[0], "__slots__"):
                        result[field] = [self._msg_to_dict(item) for item in value]
                    else:
                        result[field] = list(value)
                else:
                    result[field] = value
        return result

    def _build_topics_snapshot(self) -> TopicsSnapshot:
        return TopicsSnapshot(
            timestamp=time.time(),
            topics=list(self._topic_updates.values()),
        )

    def _publish_telemetry(self) -> None:
        """Aggregate data and POST to backend with exponential backoff."""
        now = time.monotonic()
        if self._backend_down and now < self._next_retry_time:
            return

        snapshot = self._build_snapshot()
        topics_snapshot = self._build_topics_snapshot()

        try:
            r = self._session.post(
                f"{self._backend_url}/telemetry/record",
                json=asdict(snapshot),
                timeout=1.0,
            )
            if r.status_code != HTTPStatus.OK:
                self.get_logger().warning(f"Backend returned {r.status_code}")

            self._session.post(
                f"{self._backend_url}/telemetry/topics/update",
                json=asdict(topics_snapshot),
                timeout=1.0,
            )

            if self._backend_down:
                self.get_logger().info("Backend reconnected — telemetry resumed")
                self._publish_backend_status("connected")
            self._backend_down = False
            self._backoff_delay = _BACKOFF_INITIAL

        except requests.exceptions.RequestException as exc:
            if not self._backend_down:
                self.get_logger().warning(f"Backend unreachable: {exc}")
                self._publish_backend_status("disconnected")
            self._backend_down = True
            self._next_retry_time = now + self._backoff_delay
            self._backoff_delay = min(self._backoff_delay * 2, _BACKOFF_MAX)

    def _publish_backend_status(self, status: str) -> None:
        """Publish backend connectivity state to /system_status."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        entry = DiagnosticStatus()
        entry.name = "TelemetryBackend"
        entry.level = DiagnosticStatus.OK if status == "connected" else DiagnosticStatus.ERROR
        entry.message = f"backend {status}"
        entry.values.append(KeyValue(key="url", value=self._backend_url))
        msg.status.append(entry)
        if hasattr(self, "_system_status_pub"):
            self._system_status_pub.publish(msg)

    def _publish_ui_summary(self) -> None:
        """Publish the low-rate lidar/yaw/detection summary for the OLED.

        Always publishes, regardless of /robot_state -- simpler than gating
        like /race_metrics, and the bandwidth cost of a ~150-byte JSON blob
        at 2Hz is negligible even while idle.
        """
        front = left = right = 0.0
        if self._latest_scan is not None:
            front, left, right = _lidar_clearances_cm(self._latest_scan.ranges)

        yaw = 0.0
        if self._latest_imu is not None:
            q = self._latest_imu.orientation
            yaw = math.degrees(self._quaternion_to_yaw(q.x, q.y, q.z, q.w))

        detection = _best_detection(self._latest_vision) if self._latest_vision is not None else None
        class_id, confidence = detection if detection is not None else (None, None)

        msg = String()
        msg.data = json.dumps(
            {
                "lidar_front_cm": front,
                "lidar_left_cm": left,
                "lidar_right_cm": right,
                "gyro_yaw_deg": yaw,
                "best_detection_class_id": class_id,
                "best_detection_confidence": confidence,
            },
        )
        self._ui_summary_pub.publish(msg)

    def _build_snapshot(self) -> RobotSnapshot:
        """Build RobotSnapshot dict from latest sensor data."""
        timestamp = time.time()

        # Robot position (from odometry)
        robot_position: list[float] | None = None
        robot_orientation: float | None = None
        if self._latest_odom:
            pos = self._latest_odom.pose.pose.position
            robot_position = [pos.x, pos.y, pos.z]
            # Extract yaw from quaternion
            q = self._latest_odom.pose.pose.orientation
            robot_orientation = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # LiDAR points (convert to world frame)
        lidar_points: list[list[float]] = []
        if self._latest_scan and robot_position and robot_orientation is not None:
            lidar_points = self._scan_to_points(self._latest_scan, robot_position, robot_orientation)

        # Metrics
        metrics = self._build_metrics(timestamp)

        # IMU data
        imu_data: _IMUPayload | None = None
        if self._latest_imu:
            imu_data = _IMUPayload(
                linearAcceleration=[
                    self._latest_imu.linear_acceleration.x,
                    self._latest_imu.linear_acceleration.y,
                    self._latest_imu.linear_acceleration.z,
                ],
                angularVelocity=[
                    self._latest_imu.angular_velocity.x,
                    self._latest_imu.angular_velocity.y,
                    self._latest_imu.angular_velocity.z,
                ],
                orientationQuaternion=[
                    self._latest_imu.orientation.x,
                    self._latest_imu.orientation.y,
                    self._latest_imu.orientation.z,
                    self._latest_imu.orientation.w,
                ],
            )

        # Motor state
        motor_state: _MotorStatePayload | None = None
        if self._latest_joints and self._latest_cmd_vel:
            motor_state = _MotorStatePayload(
                steeringAngle=self._latest_joints.position[0] if len(self._latest_joints.position) > 0 else 0.0,
                driveSpeed=self._latest_cmd_vel.linear.x,
                encoderPosition=int(self._latest_joints.position[1]) if len(self._latest_joints.position) > 1 else 0,
            )

        # Vision detections
        vision_detections: list[_VisionDetectionPayload] | None = None
        if self._latest_vision:
            vision_detections = []
            for det in self._latest_vision.detections:
                # Handle varying ROS2 versions
                if hasattr(det.results[0], "hypothesis"):
                    class_id = det.results[0].hypothesis.class_id
                    score = det.results[0].hypothesis.score
                else:
                    class_id = det.results[0].id
                    score = det.results[0].score

                vision_detections.append(
                    _VisionDetectionPayload(
                        className=class_id,
                        confidence=score,
                        bbox=[
                            det.bbox.center.position.x,
                            det.bbox.center.position.y,
                            det.bbox.size_x,
                            det.bbox.size_y,
                        ],
                    ),
                )

        return RobotSnapshot(
            timestamp=timestamp,
            missionName="WRO 2026 Robot",
            robotPosition=robot_position,
            robotOrientation=robot_orientation,
            lidarPoints=lidar_points,
            pathHistory=list(self._path_history),
            logs=list(self._logs),
            metrics=metrics,
            imuData=imu_data,
            visionDetections=vision_detections,
            motorState=motor_state,
        )

    def _build_metrics(self, timestamp: float) -> TelemetryMetrics:
        """Build TelemetryMetrics dict."""
        lidar_available = self._latest_scan is not None
        imu_available = self._latest_imu is not None
        odometry_available = self._latest_odom is not None
        camera_available = self._latest_vision is not None

        points_captured = 0
        range_min: float | None = None
        range_max: float | None = None
        range_mean: float | None = None
        forward: float | None = None
        left: float | None = None
        right: float | None = None
        back: float | None = None

        if self._latest_scan:
            valid_ranges = [
                r for r in self._latest_scan.ranges if self._latest_scan.range_min < r < self._latest_scan.range_max
            ]
            if valid_ranges:
                points_captured = len(valid_ranges)
                range_min = min(valid_ranges)
                range_max = max(valid_ranges)
                range_mean = sum(valid_ranges) / len(valid_ranges)

                n = len(self._latest_scan.ranges)
                # Ensure we have enough points before indexing
                if n > _MIN_POINTS_FOR_FORWARD_WINDOW:
                    forward_scan = self._latest_scan.ranges[n // 2 - 10 : n // 2 + 10]
                    valid_fwd = [
                        r for r in forward_scan if self._latest_scan.range_min < r < self._latest_scan.range_max
                    ]
                    forward = min(valid_fwd) if valid_fwd else None
                if n > _MIN_POINTS_FOR_SIDE_WINDOW:
                    left_scan = self._latest_scan.ranges[: n // 4]
                    valid_l = [r for r in left_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    left = min(valid_l) if valid_l else None

                    right_scan = self._latest_scan.ranges[3 * n // 4 :]
                    valid_r = [r for r in right_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    right = min(valid_r) if valid_r else None
                if n > _MIN_POINTS_FOR_BACK_WINDOW:
                    back_scan = self._latest_scan.ranges[n // 2 - n // 8 : n // 2 + n // 8]
                    valid_b = [r for r in back_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    back = min(valid_b) if valid_b else None

        speed: float | None = None
        if self._latest_cmd_vel:
            speed = self._latest_cmd_vel.linear.x

        return TelemetryMetrics(
            timestamp=timestamp,
            nodeHealth="nominal",
            pointsCaptured=points_captured,
            rangeMin=range_min,
            rangeMax=range_max,
            rangeMean=range_mean,
            forward=forward,
            left=left,
            right=right,
            back=back,
            speed=speed,
            stage=self._latest_state,
            lidarAvailable=lidar_available,
            imuAvailable=imu_available,
            cameraAvailable=camera_available,
            odometryAvailable=odometry_available,
        )

    def _scan_to_points(self, scan: LaserScan, robot_pos: list[float], robot_yaw: float) -> list[list[float]]:
        """Convert LaserScan to world-frame point cloud."""
        points = []
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)
            x_world = robot_pos[0] + x_local * math.cos(robot_yaw) - y_local * math.sin(robot_yaw)
            y_world = robot_pos[1] + x_local * math.sin(robot_yaw) + y_local * math.cos(robot_yaw)
            points.append([x_world, y_world, 0.05])
        return points

    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        return quaternion_to_yaw(x, y, z, w)


def main(args: list[str] | None = None) -> None:
    """Main entry point for telemetry bridge node."""
    rclpy.init(args=args)
    node = TelemetryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
