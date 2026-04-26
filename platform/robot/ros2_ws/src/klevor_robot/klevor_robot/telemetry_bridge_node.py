"""ROS2 Telemetry Bridge — Publishes robot data to FastAPI backend."""

from __future__ import annotations

import json
import math
import time
import requests
from collections import deque
from typing import Optional

_BACKOFF_INITIAL = 1.0    # seconds
_BACKOFF_MAX = 30.0       # seconds

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from shared.config.coordinate_transform import quaternion_to_yaw
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray


class RosTopic:
    SCAN = "/scan"
    ODOM = "/odom"
    IMU = "/imu/data"
    STATE = "/state_machine/state"
    CMD_VEL = "/cmd_vel"
    JOINT_STATES = "/joint_states"
    HAILO_DETECTIONS = "/hailo/detections"


class RosMsgType:
    LASER_SCAN = "sensor_msgs/LaserScan"
    ODOMETRY = "nav_msgs/Odometry"
    IMU = "sensor_msgs/Imu"
    STRING = "std_msgs/String"
    TWIST = "geometry_msgs/Twist"
    JOINT_STATE = "sensor_msgs/JointState"
    DETECTION_2D_ARRAY = "vision_msgs/Detection2DArray"


class TelemetryBridgeNode(Node):
    """Subscribes to robot topics and POSTs telemetry to backend API."""

    def __init__(self):
        super().__init__("telemetry_bridge")

        # Configuration
        self.declare_parameter("backend_url", "http://localhost:8010")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_path_history", 120)

        self._backend_url = self.get_parameter("backend_url").value
        self._rate = self.get_parameter("publish_rate_hz").value
        self._max_history = self.get_parameter("max_path_history").value

        # Subscriptions — sensor topics use qos_profile_sensor_data to match
        # the BEST_EFFORT QoS that hardware drivers publish with.
        self.create_subscription(LaserScan, RosTopic.SCAN, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, RosTopic.ODOM, self._odom_callback, 10)
        self.create_subscription(Imu, RosTopic.IMU, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(String, RosTopic.STATE, self._state_callback, 10)
        self.create_subscription(Twist, RosTopic.CMD_VEL, self._cmd_vel_callback, 10)
        self.create_subscription(JointState, RosTopic.JOINT_STATES, self._joint_callback, 10)
        self.create_subscription(
            Detection2DArray, RosTopic.HAILO_DETECTIONS, self._vision_callback, qos_profile_sensor_data
        )

        # Latest data cache
        self._latest_scan: Optional[LaserScan] = None
        self._latest_odom: Optional[Odometry] = None
        self._latest_imu: Optional[Imu] = None
        self._latest_state: str = "unknown"
        self._latest_cmd_vel: Optional[Twist] = None
        self._latest_joints: Optional[JointState] = None
        self._latest_vision: Optional[Detection2DArray] = None

        self._topic_updates: dict[str, dict] = {}
        self._topic_timestamps: dict[str, deque] = {}

        self._path_history: deque = deque(maxlen=self._max_history)
        self._logs: deque = deque(maxlen=10)

        # Backend-status publisher (reuses system_status topic).
        from diagnostic_msgs.msg import DiagnosticArray
        self._system_status_pub = self.create_publisher(DiagnosticArray, "/system_status", 10)

        # HTTP: single session for connection pooling; backoff state.
        self._session = requests.Session()
        self._backend_down = False
        self._next_retry_time: float = 0.0
        self._backoff_delay: float = _BACKOFF_INITIAL

        # Timer for publishing
        self.create_timer(1.0 / self._rate, self._publish_telemetry)

        self.get_logger().info(f"Telemetry bridge started → {self._backend_url}")

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg
        self._update_raw_topic(RosTopic.SCAN, RosMsgType.LASER_SCAN, msg)

    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg
        self._update_raw_topic(RosTopic.ODOM, RosMsgType.ODOMETRY, msg)
        pos = msg.pose.pose.position
        self._path_history.append([pos.x, pos.y, pos.z])

    def _imu_callback(self, msg: Imu):
        self._latest_imu = msg
        self._update_raw_topic(RosTopic.IMU, RosMsgType.IMU, msg)

    def _state_callback(self, msg: String):
        self._latest_state = msg.data
        self._update_raw_topic(RosTopic.STATE, RosMsgType.STRING, msg)
        self._logs.append(f"State: {msg.data}")

    def _cmd_vel_callback(self, msg: Twist):
        self._latest_cmd_vel = msg
        self._update_raw_topic(RosTopic.CMD_VEL, RosMsgType.TWIST, msg)

    def _joint_callback(self, msg: JointState):
        self._latest_joints = msg
        self._update_raw_topic(RosTopic.JOINT_STATES, RosMsgType.JOINT_STATE, msg)

    def _vision_callback(self, msg: Detection2DArray):
        self._latest_vision = msg
        self._update_raw_topic(RosTopic.HAILO_DETECTIONS, RosMsgType.DETECTION_2D_ARRAY, msg)

    def _update_raw_topic(self, topic_name: str, msg_type: str, msg):
        import time
        from collections import deque

        current_time = time.time()

        if topic_name not in self._topic_timestamps:
            self._topic_timestamps[topic_name] = deque(maxlen=10)

        self._topic_timestamps[topic_name].append(current_time)
        timestamps = list(self._topic_timestamps[topic_name])

        if len(timestamps) >= 2:
            time_diff = timestamps[-1] - timestamps[0]
            update_rate = (len(timestamps) - 1) / time_diff if time_diff > 0 else 0.0
        else:
            update_rate = 0.0

        msg_dict = self._msg_to_dict(msg)

        self._topic_updates[topic_name] = {
            "topicName": topic_name,
            "messageType": msg_type,
            "timestamp": current_time,
            "updateRateHz": round(update_rate, 2),
            "data": msg_dict,
        }

    def _msg_to_dict(self, msg) -> dict:
        result = {}
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

    def _build_topics_snapshot(self) -> dict:
        import time

        return {
            "timestamp": time.time(),
            "topics": list(self._topic_updates.values()),
        }

    def _publish_telemetry(self):
        """Aggregate data and POST to backend with exponential backoff."""
        now = time.monotonic()
        if self._backend_down and now < self._next_retry_time:
            return

        snapshot = self._build_snapshot()
        topics_snapshot = self._build_topics_snapshot()

        try:
            r = self._session.post(
                f"{self._backend_url}/telemetry/record",
                json=snapshot,
                timeout=1.0,
            )
            if r.status_code != 200:
                self.get_logger().warning(f"Backend returned {r.status_code}")

            self._session.post(
                f"{self._backend_url}/telemetry/topics/update",
                json=topics_snapshot,
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
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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

    def _build_snapshot(self) -> dict:
        """Build RobotSnapshot dict from latest sensor data."""
        timestamp = time.time()

        # Robot position (from odometry)
        robot_position = None
        robot_orientation = None
        if self._latest_odom:
            pos = self._latest_odom.pose.pose.position
            robot_position = [pos.x, pos.y, pos.z]
            # Extract yaw from quaternion
            q = self._latest_odom.pose.pose.orientation
            robot_orientation = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # LiDAR points (convert to world frame)
        lidar_points = []
        if self._latest_scan and robot_position and robot_orientation is not None:
            lidar_points = self._scan_to_points(self._latest_scan, robot_position, robot_orientation)

        # Metrics
        metrics = self._build_metrics(timestamp)

        # IMU data
        imu_data = None
        if self._latest_imu:
            imu_data = {
                "linearAcceleration": [
                    self._latest_imu.linear_acceleration.x,
                    self._latest_imu.linear_acceleration.y,
                    self._latest_imu.linear_acceleration.z,
                ],
                "angularVelocity": [
                    self._latest_imu.angular_velocity.x,
                    self._latest_imu.angular_velocity.y,
                    self._latest_imu.angular_velocity.z,
                ],
                "orientationQuaternion": [
                    self._latest_imu.orientation.x,
                    self._latest_imu.orientation.y,
                    self._latest_imu.orientation.z,
                    self._latest_imu.orientation.w,
                ],
            }

        # Motor state
        motor_state = None
        if self._latest_joints and self._latest_cmd_vel:
            motor_state = {
                "steeringAngle": self._latest_joints.position[0] if len(self._latest_joints.position) > 0 else 0.0,
                "driveSpeed": self._latest_cmd_vel.linear.x,
                "encoderPosition": int(self._latest_joints.position[1]) if len(self._latest_joints.position) > 1 else 0,
            }

        # Vision detections
        vision_detections = None
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
                    {
                        "className": class_id,
                        "confidence": score,
                        "bbox": [
                            det.bbox.center.position.x,
                            det.bbox.center.position.y,
                            det.bbox.size_x,
                            det.bbox.size_y,
                        ],
                    }
                )

        return {
            "timestamp": timestamp,
            "missionName": "WRO 2026 Robot",
            "robotPosition": robot_position,
            "robotOrientation": robot_orientation,
            "lidarPoints": lidar_points,
            "pathHistory": list(self._path_history),
            "logs": list(self._logs),
            "metrics": metrics,
            "imuData": imu_data,
            "visionDetections": vision_detections,
            "motorState": motor_state,
        }

    def _build_metrics(self, timestamp: float) -> dict:
        """Build TelemetryMetrics dict."""
        lidar_available = self._latest_scan is not None
        imu_available = self._latest_imu is not None
        odometry_available = self._latest_odom is not None
        camera_available = self._latest_vision is not None

        points_captured = 0
        range_min = None
        range_max = None
        range_mean = None
        forward = None
        left = None
        right = None
        back = None

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
                if n > 20:
                    forward_scan = self._latest_scan.ranges[n // 2 - 10 : n // 2 + 10]
                    valid_fwd = [
                        r for r in forward_scan if self._latest_scan.range_min < r < self._latest_scan.range_max
                    ]
                    forward = min(valid_fwd) if valid_fwd else None
                if n > 4:
                    left_scan = self._latest_scan.ranges[: n // 4]
                    valid_l = [r for r in left_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    left = min(valid_l) if valid_l else None

                    right_scan = self._latest_scan.ranges[3 * n // 4 :]
                    valid_r = [r for r in right_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    right = min(valid_r) if valid_r else None
                if n > 16:
                    back_scan = self._latest_scan.ranges[n // 2 - n // 8 : n // 2 + n // 8]
                    valid_b = [r for r in back_scan if self._latest_scan.range_min < r < self._latest_scan.range_max]
                    back = min(valid_b) if valid_b else None

        speed = None
        if self._latest_cmd_vel:
            speed = self._latest_cmd_vel.linear.x

        return {
            "timestamp": timestamp,
            "nodeHealth": "nominal",
            "pointsCaptured": points_captured,
            "rangeMin": range_min,
            "rangeMax": range_max,
            "rangeMean": range_mean,
            "forward": forward,
            "left": left,
            "right": right,
            "back": back,
            "speed": speed,
            "stage": self._latest_state,
            "lidarAvailable": lidar_available,
            "imuAvailable": imu_available,
            "cameraAvailable": camera_available,
            "odometryAvailable": odometry_available,
        }

    def _scan_to_points(self, scan: LaserScan, robot_pos: list, robot_yaw: float) -> list:
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


def main(args=None):
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
