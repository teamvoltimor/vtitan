"""ROS2 Telemetry Bridge — Publishes robot data to FastAPI backend."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass
from http import HTTPStatus
from typing import Any, override

import numpy as np
import rclpy
import requests
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.constants import RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.domain.models import LidarClearances, MotorStateSnapshot
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from src.navigation.control.controllers.collision_avoidance_controller import CollisionAvoidanceController
from vtitan_state_machine.command_channel import CommandChannel

_BACKOFF_INITIAL = 1.0
_BACKOFF_MAX = 60.0

_MIN_TIMESTAMPS_FOR_RATE = 2
"""Minimum tracked timestamps needed to compute a topic update rate."""

# Matches state_machine_node's _QOS_TRANSIENT: both nodes publish to
# /system_status, so a subscriber (the OLED) needs both durability-compatible
# to receive from either -- a VOLATILE publisher on this side previously
# forced the subscriber to also stay VOLATILE, which meant it could never get
# the latched current value from a fresh state_machine_node instance after a
# Pi 5 restart until the next periodic publish (if the restarted node's
# instance even re-matched the long-running subscriber at all).
#
# BEST_EFFORT (not RELIABLE), same reasoning as state_machine_node's
# _QOS_TRANSIENT: a RELIABLE publish() blocks on a slow/overloaded reader,
# which the Pi Zero's oled_display_node was measured doing for 30+ seconds
# at a time. This publisher's own update (backend connect/disconnect) isn't
# periodic like state_machine_node's diagnostics, so a dropped sample here
# persists until the next connect/disconnect event -- acceptable for a
# secondary status field, and the subscriber has to match this publisher's
# reliability regardless since both write to the same topic.
_QOS_SYSTEM_STATUS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# BEST_EFFORT so a slow/overloaded subscriber (the Pi Zero) can never make
# this publisher's own .publish() call block. RELIABLE's flow control will
# hold a writer's publish() until the reader acks or drops out -- confirmed
# on hardware: with the Pi Zero's oled_display_node occasionally taking
# 30+ seconds to keep up (I2C write stalls under CPU/memory contention on
# that board), this node's own publish() blocked for the same duration,
# stalling its entire single-threaded executor (every sensor callback and
# the backend-telemetry timer) right along with it. A dropped summary
# frame just means the OLED holds its last value one tick longer -- far
# better than dragging this node's whole pipeline down with it.
_QOS_UI_SUMMARY = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

_MIN_POINTS_FOR_FORWARD_WINDOW = 20
"""Minimum LIDAR points needed to safely slice the +/-10-index forward window."""

_MIN_POINTS_FOR_SIDE_WINDOW = 4
"""Minimum LIDAR points needed to safely slice the quarter-arc left/right windows."""

_MIN_POINTS_FOR_BACK_WINDOW = 16
"""Minimum LIDAR points needed to safely slice the n/8 back window."""

_OLED_SECTOR_HALF_FOV_RAD = math.radians(30)
"""Half-width of each front/left/right sector fed to the OLED summary.

Matches compute_forward_clearance's own forward cone width, for consistency
across the two sectors that both use a mean (this is a display readout, not
a threat gate like detect_threat_direction's narrower, min-based +/-45 deg
sectors)."""


@dataclass(frozen=True, slots=True)
class _IMUPayload:
    """IMU section of a RobotSnapshot."""

    linearAcceleration: list[float]
    angularVelocity: list[float]
    orientationQuaternion: list[float]


@dataclass(frozen=True, slots=True)
class _MotorStatePayload:
    """Serialization DTO matching the backend camelCase contract."""

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


_LIDAR_YAW_OFFSET_RAD = math.radians(RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG)
"""Rotates raw /scan bearings into the robot frame (0 rad = forward).

The C1 is mounted inverted, so its raw angle-zero points opposite
robot-front (confirmed empirically: the open-space/robot-front sector
lands at +-180 deg in raw /scan data, not 0 deg). Matches the
correction ros2/navigation/node.py applies when building LidarScan for
the real collision-avoidance path -- see that module's docstring for
the same constant.
"""


def _lidar_clearances(ranges: list[float]) -> LidarClearances:
    """Directional LIDAR clearances in meters for the OLED's RACING page.

    Reuses CollisionAvoidanceController's real angle-based sector logic
    (the same one detect_threat_direction/compute_forward_clearance use for
    actual collision avoidance) instead of the old min-of-raw-index-window
    approach -- that one had no self-detection filtering and took the single
    minimum reading in each window, so one stray noisy return (dust, an edge
    reflection) could dominate the whole sector and made the display jump to
    a nonsense 2-3cm reading. Mean-over-sector, like
    compute_forward_clearance, is far less sensitive to a single outlier.

    Passes explicit robot-frame angles (raw sweep + _LIDAR_YAW_OFFSET_RAD)
    rather than lidar_angles=None: the naive synthesized sweep assumed
    raw index 0 was already robot-front, but the C1's mount offset means
    it isn't -- previously left the display's front/left/right all
    rotated 180 deg out from reality (front showing rear's clearance,
    left and right swapped).
    """
    if len(ranges) == 0:
        return LidarClearances(front_m=0.0, left_m=0.0, right_m=0.0)

    ranges_t = tuple(ranges)
    angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False) + _LIDAR_YAW_OFFSET_RAD

    front = CollisionAvoidanceController._sector_ranges(ranges_t, angles, 0.0, _OLED_SECTOR_HALF_FOV_RAD)
    left = CollisionAvoidanceController._sector_ranges(
        ranges_t, angles, math.pi / 2, _OLED_SECTOR_HALF_FOV_RAD, filter_self_detection=True,
    )
    right = CollisionAvoidanceController._sector_ranges(
        ranges_t, angles, -math.pi / 2, _OLED_SECTOR_HALF_FOV_RAD, filter_self_detection=True,
    )

    return LidarClearances(
        front_m=float(np.mean(front)) if front.size else 0.0,
        left_m=float(np.mean(left)) if left.size else 0.0,
        right_m=float(np.mean(right)) if right.size else 0.0,
    )


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
        # blob to the Pi Zero over the USB-gadget link, kept decoupled so
        # tuning one doesn't silently affect the other. Matched to the OLED's
        # own 10Hz redraw rate -- the USB-gadget link and message size (a few
        # hundred bytes) have plenty of headroom at 10Hz; a lower rate here
        # was just making every other redraw show stale numbers.
        self.declare_parameter("ui_summary_rate_hz", 10.0)

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
        self._system_status_pub = self.create_publisher(DiagnosticArray, "/system_status", _QOS_SYSTEM_STATUS)

        # Low-rate lidar/yaw/detection summary for the Pi Zero's OLED --
        # the only sensor telemetry it needs, so it doesn't have to
        # subscribe to /scan, /imu/data and /hailo/detections directly.
        self._ui_summary_pub = self.create_publisher(String, "/ui/telemetry_summary", _QOS_UI_SUMMARY)

        # HTTP: single session for connection pooling; backoff state.
        self._session = requests.Session()
        self._backend_down = False
        self._next_retry_time: float = 0.0
        self._backoff_delay: float = _BACKOFF_INITIAL
        # POSTs run here, off the executor thread that also processes
        # /scan, /imu/data and the ui-summary timer -- two sequential
        # requests.post() calls at timeout=1.0 each could otherwise block
        # this node's entire callback processing for up to 2s on any
        # backend hang/slow-fail (a normal ECONNREFUSED is near-instant,
        # but not every failure mode is), which stalls
        # /ui/telemetry_summary right along with it -- the OLED's
        # multi-second freezes traced back to this.
        self._http_executor = ThreadPoolExecutor(max_workers=1)
        self._telemetry_post_in_flight = False

        # TEMP DIAGNOSTIC (2026-07-28): see _publish_ui_summary.
        self._last_ui_summary_publish_time: float | None = None

        # Backend->robot command channel (gRPC RobotCommandService). host:port,
        # not a URL -- gRPC channels don't take a scheme, unlike backend_url.
        self.declare_parameter("command_channel_target", "localhost:9010")
        command_channel_target = self.get_parameter("command_channel_target").value

        # START_RACE/STOP_RACE/EMERGENCY_STOP are applied by publishing the
        # same synthetic /button/event the physical button already produces
        # -- state_machine_node has no service/topic of its own for commands,
        # and reusing its one existing trigger path is simpler than adding a
        # second one. PAUSE/RESUME/RETURN_TO_START/REBOOT/SHUTDOWN have no
        # backing implementation anywhere in this codebase yet (no paused
        # state, no reboot/shutdown handler) -- those are acked FAILED, not
        # silently dropped, so the backend/operator can see they didn't run.
        button_pub = self.create_publisher(String, "/button/event", 10)
        # SET_VISION_DEBUG is applied via VisionNode's standard ROS2
        # set_parameters service (see ros2/vision/node.py's
        # add_on_set_parameters_callback) -- these are separate OS processes
        # with no shared Python objects, so this is the only way in.
        vision_params_client: Client = self.create_client(SetParameters, "/vision_detector/set_parameters")

        self._command_channel = CommandChannel(
            backend_url=self._backend_url,
            command_channel_target=command_channel_target,
            button_pub=button_pub,
            vision_params_client=vision_params_client,
            logger=self.get_logger(),
        )
        self._command_channel.start()

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
        """Track a topic's freshness/rate and snapshot it for the backend POST.

        Exists solely to feed _build_topics_snapshot -> the backend's
        /telemetry/topics/update -- nothing else reads _topic_updates. With
        no backend to send it to, _msg_to_dict's recursive __slots__ walk
        (list-copying a LaserScan's 720+ ranges, at 10Hz, on every sensor
        callback) was pure wasted CPU sitting directly in this node's
        single-threaded executor's hot path -- exactly where a momentary
        backlog can make BEST_EFFORT/shallow-KEEP_LAST silently drop the
        next inbound /scan sample instead of the OLED ever seeing it.
        """
        if self._backend_down:
            return

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
        """Aggregate data and hand the backend POST off to a worker thread.

        Only the (cheap, in-memory) snapshot building happens here; the
        network call itself runs on _http_executor so a slow/hung backend
        can never stall this node's own callback processing.
        """
        # TEMP DIAGNOSTIC (2026-07-28): time each step of this callback's
        # synchronous portion -- this node runs on a plain rclpy.spin()
        # (single-threaded executor), so if ANY of these takes seconds, it
        # blocks _publish_ui_summary's timer right along with it (both share
        # the one thread/default callback group). Remove once root-caused.
        t0 = time.monotonic()

        now = time.monotonic()
        if self._backend_down and now < self._next_retry_time:
            return
        if self._telemetry_post_in_flight:
            # Previous POST hasn't finished (still inside its own timeout) --
            # skip this tick rather than queue up a second one behind it.
            return

        t1 = time.monotonic()
        snapshot = self._build_snapshot()
        t2 = time.monotonic()
        topics_snapshot = self._build_topics_snapshot()
        t3 = time.monotonic()

        self._telemetry_post_in_flight = True
        future = self._http_executor.submit(self._post_telemetry, snapshot, topics_snapshot)
        future.add_done_callback(self._on_telemetry_posted)
        t4 = time.monotonic()

        total = t4 - t0
        if total > 0.3:
            self.get_logger().warning(
                f"[DIAG] _publish_telemetry took {total:.2f}s "
                f"(pre-check={t1 - t0:.2f}s build_snapshot={t2 - t1:.2f}s "
                f"build_topics={t3 - t2:.2f}s submit={t4 - t3:.2f}s)",
            )

    def _post_telemetry(self, snapshot: RobotSnapshot, topics_snapshot: TopicsSnapshot) -> None:
        """Runs on the HTTP worker thread -- the two blocking POSTs live here."""
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

    def _on_telemetry_posted(self, future: Future[None]) -> None:
        """Done-callback for the background POST -- runs on the worker thread."""
        self._telemetry_post_in_flight = False
        now = time.monotonic()
        try:
            future.result()
        except requests.exceptions.RequestException as exc:
            if not self._backend_down:
                self.get_logger().warning(f"Backend unreachable: {exc}")
                self._publish_backend_status("disconnected")
            self._backend_down = True
            self._next_retry_time = now + self._backoff_delay
            self._backoff_delay = min(self._backoff_delay * 2, _BACKOFF_MAX)
        else:
            if self._backend_down:
                self.get_logger().info("Backend reconnected — telemetry resumed")
                self._publish_backend_status("connected")
            self._backend_down = False
            self._backoff_delay = _BACKOFF_INITIAL

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
        # TEMP DIAGNOSTIC (2026-07-28): pin down whether reported 1-2s OLED
        # freezes originate in this node's own publish cadence (this would
        # show it), in transit, or in the Pi Zero's render path (see
        # oled_display_node.py's matching instrumentation). Remove once
        # root-caused.
        now_monotonic = time.monotonic()
        if self._last_ui_summary_publish_time is not None:
            gap = now_monotonic - self._last_ui_summary_publish_time
            if gap > 0.5:
                self.get_logger().warning(f"[DIAG] ui_summary publish gap: {gap:.2f}s (expected ~0.1s)")
        self._last_ui_summary_publish_time = now_monotonic

        d0 = time.monotonic()
        front = left = right = 0.0
        if self._latest_scan is not None:
            c = _lidar_clearances(self._latest_scan.ranges)
            front = c.front_m * 100
            left = c.left_m * 100
            right = c.right_m * 100
        d1 = time.monotonic()

        yaw = 0.0
        if self._latest_imu is not None:
            q = self._latest_imu.orientation
            yaw = math.degrees(self._quaternion_to_yaw(q.x, q.y, q.z, q.w))
        d2 = time.monotonic()

        detection = _best_detection(self._latest_vision) if self._latest_vision is not None else None
        class_id, confidence = detection if detection is not None else (None, None)
        d3 = time.monotonic()

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
        d4 = time.monotonic()

        # TEMP DIAGNOSTIC (2026-07-28): times this function's OWN body, not
        # just the gap before it -- the gap-only measurement above can't
        # tell "the timer fired late" apart from "the previous invocation
        # itself took several seconds to return" (the next tick's measured
        # gap includes that time either way). Remove once root-caused.
        own_total = d4 - d0
        if own_total > 0.3:
            self.get_logger().warning(
                f"[DIAG] _publish_ui_summary body took {own_total:.2f}s "
                f"(lidar_clearances={d1 - d0:.2f}s imu_yaw={d2 - d1:.2f}s "
                f"best_detection={d3 - d2:.2f}s json+publish={d4 - d3:.2f}s)",
            )

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
            motor_snapshot = MotorStateSnapshot(
                steering_angle_deg=float(self._latest_joints.position[0]) if len(self._latest_joints.position) > 0 else 0.0,
                drive_speed=self._latest_cmd_vel.linear.x,
                encoder_position=int(self._latest_joints.position[1]) if len(self._latest_joints.position) > 1 else 0,
                timestamp=timestamp,
            )
            motor_state = _MotorStatePayload(
                steeringAngle=motor_snapshot.steering_angle_deg,
                driveSpeed=motor_snapshot.drive_speed,
                encoderPosition=motor_snapshot.encoder_position,
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

    @override
    def destroy_node(self) -> None:
        """Release the HTTP worker thread and command-channel thread before teardown."""
        self._command_channel.stop()
        self._http_executor.shutdown(wait=False)
        super().destroy_node()


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
