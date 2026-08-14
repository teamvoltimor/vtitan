"""ROS2 Telemetry Bridge — streams robot data to the backend over gRPC."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, override

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from shared.config.constants import RobotSpecs
from shared.config.coordinate_transform import quaternion_to_yaw
from shared.config.ros_topics import RosTopicConfig
from shared.domain.models import Detection, LidarClearances, MotorStateSnapshot
from std_msgs.msg import String

from src.config.tuning_helpers import get_tuning
from src.navigation.control.controllers.collision_avoidance_controller import CollisionAvoidanceController
from src.ros2.params import (
    declare_and_get_float_param,
    declare_and_get_int_param,
    declare_and_get_str_param,
    declare_param,
)
from src.ros2.qos import QOS_LATCHED_STATE, QOS_LIVE_READOUT, QOS_STREAM
from src.ros2.wire_models import TelemetrySummaryWire
from src.ros2.vision.detection_payload_keys import parse_detection
from vtitan_state_machine.command_channel import CommandChannel
from vtitan_state_machine.telemetry_ingest_channel import TelemetryIngestChannel

if TYPE_CHECKING:
    from rclpy.client import Client

_MIN_TIMESTAMPS_FOR_RATE = 2
"""Minimum tracked timestamps needed to compute a topic update rate."""

# QOS_LATCHED_STATE (src/ros2/qos.py) -- both this node and state_machine_node
# publish to /system_status, so a subscriber (the OLED) needs both
# durability-compatible to receive from either -- a VOLATILE publisher on this
# side previously forced the subscriber to also stay VOLATILE, which meant it
# could never get the latched current value from a fresh state_machine_node
# instance after a Pi 5 restart until the next periodic publish (if the
# restarted node's instance even re-matched the long-running subscriber at
# all). BEST_EFFORT (not RELIABLE): a RELIABLE publish() blocks on a
# slow/overloaded reader, which the Pi Zero's oled_display_node was measured
# doing for 30+ seconds at a time.

# QOS_LIVE_READOUT (src/ros2/qos.py) -- BEST_EFFORT so a slow/overloaded
# subscriber (the Pi Zero) can never make this publisher's own .publish() call
# block. RELIABLE's flow control will hold a writer's publish() until the
# reader acks or drops out -- confirmed on hardware: with the Pi Zero's
# oled_display_node occasionally taking 30+ seconds to keep up (I2C write
# stalls under CPU/memory contention on that board), this node's own
# publish() blocked for the same duration, stalling its entire
# single-threaded executor (every sensor callback and the backend-telemetry
# timer) right along with it. A dropped summary frame just means the OLED
# holds its last value one tick longer -- far better than dragging this
# node's whole pipeline down with it.

_MIN_POINTS_FOR_FORWARD_WINDOW = 20
"""Minimum LIDAR points needed to safely slice the +/-10-index forward window."""

_MIN_POINTS_FOR_SIDE_WINDOW = 4
"""Minimum LIDAR points needed to safely slice the quarter-arc left/right windows."""

_MIN_POINTS_FOR_BACK_WINDOW = 16
"""Minimum LIDAR points needed to safely slice the n/8 back window."""

_DIAG_TELEMETRY_SLOW_S = 0.3
"""Warn when the whole _publish_telemetry synchronous body exceeds this (s)."""

_DIAG_UI_SUMMARY_SLOW_S = 0.3
"""Warn when the _publish_ui_summary body exceeds this (s)."""

_DIAG_UI_SUMMARY_GAP_S = 0.5
"""Warn when consecutive _publish_ui_summary invocations are this far apart (s)."""


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
    """A single vision detection, as reported in a RobotSnapshot."""

    className: str
    confidence: float
    bbox: list[float]


@dataclass(frozen=True, slots=True)
class TelemetryMetrics:
    """Backend-facing telemetry metrics payload (streamed as part of RobotSnapshot)."""

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
    """Backend-facing telemetry snapshot streamed via TelemetryIngestService.StreamSnapshots."""

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
    """Backend-facing topic-health snapshot streamed via TelemetryIngestService.StreamTopics."""

    timestamp: float
    topics: list[_TopicUpdatePayload]




class RosMsgType:
    """ROS2 message type string constants."""

    LASER_SCAN = "sensor_msgs/LaserScan"
    ODOMETRY = "nav_msgs/Odometry"
    IMU = "sensor_msgs/Imu"
    STRING = "std_msgs/String"
    ACKERMANN_DRIVE_STAMPED = "ackermann_msgs/AckermannDriveStamped"
    JOINT_STATE = "sensor_msgs/JointState"


_LIDAR_YAW_OFFSET_RAD = RobotSpecs.lidar_yaw_offset_rad()
"""Rotates raw /scan bearings into the robot frame (0 rad = forward).

Previously recomputed `(180.0 if RobotSpecs.LIDAR_INVERTED else 0.0) +
RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG` locally, and that local copy
silently dropped the 180deg term (written 2026-07-28, before LIDAR_INVERTED
existed as a separate flag) while ros2_hardware_gateway.py's and
static_tfs.launch.py's copies got fixed 2026-08-02 -- confirmed live on
hardware 2026-08-09: the OLED's F/L/R readout was still un-inverted (front
read as back, left read as right). See RobotSpecs.lidar_yaw_offset_rad()'s
docstring for the full history.
"""


def _lidar_clearances(ranges: list[float], sector_half_fov_rad: float) -> LidarClearances:
    """Directional LIDAR clearances in meters for the OLED's RACING page.

    Reuses CollisionAvoidanceController's real angle-based sector logic
    (the same one detect_threat_direction/compute_forward_clearance use for
    actual collision avoidance) instead of the old min-of-raw-index-window
    approach -- that one had no self-detection filtering and took the single
    minimum reading in each window, so one stray noisy return (dust, an edge
    reflection) could dominate the whole sector and made the display jump to
    a nonsense 2-3cm reading. Mean-over-sector, like
    compute_forward_clearance, is far less sensitive to a single outlier.

    ``sector_half_fov_rad`` comes from ``NavigationTuning.lidar_sectors.
    FRONT_HALF_FOV_DEG`` -- that TOML's own header comment says it's "shared
    by collision avoidance and the OLED", but this function used to read a
    hardcoded ``radians(30)`` module constant instead, so a tuning change
    never actually reached the display it names.

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

    front = CollisionAvoidanceController.sector_ranges(ranges_t, angles, 0.0, sector_half_fov_rad)
    left = CollisionAvoidanceController.sector_ranges(
        ranges_t, angles, math.pi / 2, sector_half_fov_rad, filter_self_detection=True,
    )
    right = CollisionAvoidanceController.sector_ranges(
        ranges_t, angles, -math.pi / 2, sector_half_fov_rad, filter_self_detection=True,
    )

    return LidarClearances(
        front_m=float(np.mean(front)) if front.size else 0.0,
        left_m=float(np.mean(left)) if left.size else 0.0,
        right_m=float(np.mean(right)) if right.size else 0.0,
    )


def _parse_detections(raw: str) -> list[Detection]:
    """Parse vision_node's JSON detections payload on /vision/detections.

    vision_node (src/ros2/vision/node.py) only ever publishes a JSON-encoded
    std_msgs/String here -- it has never published vision_msgs/Detection2DArray
    on any topic. This node used to subscribe Detection2DArray on
    /hailo/detections, which nothing publishes, so visionDetections telemetry
    and the OLED's best-detection readout were both silently dead the entire
    time. Uses the same parse_detection ROS2HardwareGateway._vision_callback
    does (the real, working consumer of this same topic) -- this used to be
    an independent, hand-copied duplicate of that parsing.
    """
    try:
        raw_data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return []
    return [det for d in raw_data if (det := parse_detection(d)) is not None]


def _best_detection(detections: list[Detection]) -> tuple[str, float] | None:
    """Track the single most salient detection for the OLED's RACING page.

    Ranked by confidence x bbox area rather than confidence alone: a small,
    high-confidence false positive and a large, low-confidence smear are
    both less trustworthy than one detection that scores well on both axes.
    """
    best: tuple[str, float] | None = None
    best_score = -1.0
    for det in detections:
        score = det.confidence * det.area
        if score > best_score:
            best_score = score
            best = (det.class_name, det.confidence)
    return best


class TelemetryBridgeNode(Node):
    """Subscribes to robot topics and streams telemetry to the backend over gRPC."""

    def __init__(self) -> None:
        super().__init__("telemetry_bridge")

        # Configuration
        self._backend_url = declare_and_get_str_param(self, "backend_url", "http://localhost:8010")
        self._rate = declare_and_get_float_param(self, "publish_rate_hz", 10.0)
        self._max_history = declare_and_get_int_param(self, "max_path_history", 120)
        # Independent from publish_rate_hz above: that one drives the HTTP
        # POST to the backend over WiFi/LAN. This one drives a small JSON
        # blob to the Pi Zero over the USB-gadget link, kept decoupled so
        # tuning one doesn't silently affect the other. Matched to the OLED's
        # own 10Hz redraw rate -- the USB-gadget link and message size (a few
        # hundred bytes) have plenty of headroom at 10Hz; a lower rate here
        # was just making every other redraw show stale numbers.
        self._ui_summary_rate = declare_and_get_float_param(self, "ui_summary_rate_hz", 10.0)

        self._topics = RosTopicConfig.load_default()
        # Matches compute_forward_clearance's own forward cone width, for
        # consistency across the two sectors that both use a mean (this is a
        # display readout, not a threat gate like detect_threat_direction's
        # narrower, min-based +/-45 deg sectors).
        self._oled_sector_half_fov_rad = math.radians(get_tuning(None).lidar_sectors.FRONT_HALF_FOV_DEG)

        self._setup_subscriptions()

        # Latest data cache
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._latest_imu: Imu | None = None
        self._latest_state: str = "unknown"
        self._latest_ackermann_cmd: AckermannDriveStamped | None = None
        self._latest_joints: JointState | None = None
        self._latest_vision: list[Detection] | None = None

        self._topic_updates: dict[str, _TopicUpdatePayload] = {}
        self._topic_timestamps: dict[str, deque] = {}

        self._path_history: deque = deque(maxlen=self._max_history)
        self._logs: deque = deque(maxlen=10)

        # Backend-status publisher (reuses system_status topic). Driven by
        # the telemetry gRPC channel's connect/disconnect events -- see
        # _on_telemetry_channel_state_changed.
        self._backend_down = False
        self._system_status_pub = self.create_publisher(
            DiagnosticArray, self._topics.state_machine.system_status, QOS_LATCHED_STATE,
        )

        # Low-rate lidar/yaw/detection summary for the Pi Zero's OLED --
        # the only sensor telemetry it needs, so it doesn't have to
        # subscribe to /scan, /imu/data and /hailo/detections directly.
        self._ui_summary_pub = self.create_publisher(String, self._topics.ui.telemetry_summary, QOS_LIVE_READOUT)

        # TEMP DIAGNOSTIC (2026-07-28): see _publish_ui_summary.
        self._last_ui_summary_publish_time: float | None = None

        # command_channel_enabled/telemetry_channel_enabled are the LOCAL
        # recovery surface for both gRPC channels (e.g. `ros2 param set
        # /telemetry_bridge command_channel_enabled true` after a remote
        # DISABLE_COMMAND_CHANNEL command, which can only be undone this
        # way -- see command_channel.py's docstring). Both channels also
        # sync these params on their own state changes (_on_channel_state_
        # changed below), so `ros2 param get` always reflects reality even
        # after a remote-triggered stop/restart.
        default_channel_enabled = True
        declare_param(self, "command_channel_enabled", default_channel_enabled)
        declare_param(self, "telemetry_channel_enabled", default_channel_enabled)
        self._syncing_channel_param = False
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Backend<->robot gRPC channels. host:port, not a URL -- gRPC
        # channels don't take a scheme, unlike backend_url.
        command_channel_target = declare_and_get_str_param(self, "command_channel_target", "localhost:9010")
        # Same backend process, same grpcSrv (see cmd/server/main.go), so the
        # ingest service listens on the same port as the command channel --
        # reusing command_channel_target's default rather than inventing a
        # second port.
        telemetry_channel_target = declare_and_get_str_param(
            self, "telemetry_channel_target", command_channel_target
        )

        # START_RACE/STOP_RACE/EMERGENCY_STOP are applied by publishing the
        # same synthetic /button/event the physical button already produces
        # -- state_machine_node has no service/topic of its own for commands,
        # and reusing its one existing trigger path is simpler than adding a
        # second one. PAUSE/RESUME/RETURN_TO_START/REBOOT/SHUTDOWN have no
        # backing implementation anywhere in this codebase yet (no paused
        # state, no reboot/shutdown handler) -- those are acked FAILED, not
        # silently dropped, so the backend/operator can see they didn't run.
        button_pub = self.create_publisher(String, self._topics.button.event, QOS_STREAM)
        # SET_VISION_DEBUG is applied via VisionNode's standard ROS2
        # set_parameters service (see ros2/vision/node.py's
        # add_on_set_parameters_callback) -- these are separate OS processes
        # with no shared Python objects, so this is the only way in.
        #
        # The service path is built from a parameter rather than hardcoded:
        # VisionNode's own default name is "vision_detector", but the launch
        # file renames it (Node(name=...)), and the service lives under
        # whatever the deployed name is. Hardcoding the code-side default
        # meant this client pointed at a node that does not exist under the
        # real deployment, so every SET_VISION_DEBUG was acked FAILED with
        # "parameter service unavailable" -- the command looked delivered
        # (the backend returns 202 on stream delivery, not on execution) but
        # could never take effect. Launch files that rename the node must
        # pass the same name here; rpi5_nodes.launch.py derives both from
        # one constant.
        vision_node_name = declare_and_get_str_param(self, "vision_node_name", "vision")
        vision_params_client: Client = self.create_client(SetParameters, f"/{vision_node_name}/set_parameters")

        self._telemetry_channel = TelemetryIngestChannel(
            backend_target=telemetry_channel_target,
            logger=self.get_logger(),
            on_state_changed=self._on_telemetry_channel_state_changed,
        )
        self._command_channel = CommandChannel(
            backend_url=self._backend_url,
            command_channel_target=command_channel_target,
            button_pub=button_pub,
            vision_params_client=vision_params_client,
            logger=self.get_logger(),
            on_state_changed=self._on_command_channel_state_changed,
            telemetry_channel=self._telemetry_channel,
        )
        self._command_channel.start()
        self._telemetry_channel.start()

        # Timer for publishing
        self.create_timer(1.0 / self._rate, self._publish_telemetry)
        self.create_timer(1.0 / self._ui_summary_rate, self._publish_ui_summary)

        self.get_logger().info(f"Telemetry bridge started → {self._backend_url}")

    def _setup_subscriptions(self) -> None:
        """Subscribe to every sensor/state topic this bridge relays.

        Sensor topics use qos_profile_sensor_data to match the BEST_EFFORT
        QoS that hardware drivers publish with.
        """
        self.create_subscription(LaserScan, self._topics.sensors.scan, self._scan_callback, qos_profile_sensor_data)
        if self._topics.navigation.odometry:
            self.create_subscription(Odometry, self._topics.navigation.odometry, self._odom_callback, QOS_STREAM)
        self.create_subscription(Imu, self._topics.sensors.imu, self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(String, self._topics.state_machine.state, self._state_callback, QOS_STREAM)
        self.create_subscription(
            AckermannDriveStamped,
            self._topics.commands.ackermann_cmd,
            self._ackermann_cmd_callback,
            QOS_STREAM,
        )
        self.create_subscription(JointState, self._topics.actuators.joint_states, self._joint_callback, QOS_STREAM)
        # Default (reliable, depth 10) QoS to match vision_node's own
        # create_publisher(String, detections_topic, 10) -- not
        # qos_profile_sensor_data, which is BEST_EFFORT and would be
        # incompatible with that publisher's default RELIABLE reliability.
        self.create_subscription(
            String,
            self._topics.sensors.vision_detections,
            self._vision_callback,
            QOS_STREAM,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._update_raw_topic(self._topics.sensors.scan, RosMsgType.LASER_SCAN, msg)

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._update_raw_topic(self._topics.navigation.odometry or "/odom", RosMsgType.ODOMETRY, msg)
        pos = msg.pose.pose.position
        self._path_history.append([pos.x, pos.y, pos.z])

    def _imu_callback(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._update_raw_topic(self._topics.sensors.imu, RosMsgType.IMU, msg)

    def _state_callback(self, msg: String) -> None:
        self._latest_state = msg.data
        self._update_raw_topic(self._topics.state_machine.state, RosMsgType.STRING, msg)
        self._logs.append(f"State: {msg.data}")

    def _ackermann_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        self._latest_ackermann_cmd = msg
        self._update_raw_topic(self._topics.commands.ackermann_cmd, RosMsgType.ACKERMANN_DRIVE_STAMPED, msg)

    def _joint_callback(self, msg: JointState) -> None:
        self._latest_joints = msg
        self._update_raw_topic(self._topics.actuators.joint_states, RosMsgType.JOINT_STATE, msg)

    def _vision_callback(self, msg: String) -> None:
        self._latest_vision = _parse_detections(msg.data)
        self._update_raw_topic(self._topics.sensors.vision_detections, RosMsgType.STRING, msg)

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
                elif isinstance(value, list | tuple):
                    if len(value) > 0 and hasattr(value[0], "__slots__"):
                        result[field] = [self._msg_to_dict(item) for item in value]
                    else:
                        result[field] = list(value)
                else:
                    result[field] = value
        return result

    def _build_topics_snapshot(self) -> TopicsSnapshot:
        # update_rate_hz is genuinely 0.0 until a topic has accumulated
        # _MIN_TIMESTAMPS_FOR_RATE samples (see _update_raw_topic), but the
        # backend's TopicUpdate.update_rate_hz requires a strictly positive
        # value -- omit not-yet-measurable topics rather than sending 0 and
        # failing validation. They reappear a few ticks later once the rate
        # is real.
        return TopicsSnapshot(
            timestamp=time.time(),
            topics=[t for t in self._topic_updates.values() if t.updateRateHz > 0],
        )

    def _publish_telemetry(self) -> None:
        """Aggregate current sensor data and push it onto the telemetry channel.

        push_snapshot/push_topics are non-blocking by construction (keep-
        latest queues drained by TelemetryIngestChannel's own background
        threads), so there's no in-flight tracking needed here the way the
        old HTTP POST path required.
        """
        # TEMP DIAGNOSTIC (2026-07-28): time each step of this callback's
        # synchronous portion -- this node runs on a plain rclpy.spin()
        # (single-threaded executor), so if ANY of these takes seconds, it
        # blocks _publish_ui_summary's timer right along with it (both share
        # the one thread/default callback group). Remove once root-caused.
        t0 = time.monotonic()

        snapshot = self._build_snapshot()
        t1 = time.monotonic()
        topics_snapshot = self._build_topics_snapshot()
        t2 = time.monotonic()

        self._telemetry_channel.push_snapshot(snapshot)
        self._telemetry_channel.push_topics(topics_snapshot)
        t3 = time.monotonic()

        total = t3 - t0
        if total > _DIAG_TELEMETRY_SLOW_S:
            self.get_logger().warning(
                f"[DIAG] _publish_telemetry took {total:.2f}s "
                f"(build_snapshot={t1 - t0:.2f}s build_topics={t2 - t1:.2f}s push={t3 - t2:.2f}s)",
            )

    def _on_command_channel_state_changed(self, connected: bool) -> None:
        """Keep command_channel_enabled in sync after a remote-triggered stop."""
        self._sync_channel_param("command_channel_enabled", connected)

    def _on_telemetry_channel_state_changed(self, connected: bool) -> None:
        """Drive /system_status and telemetry_channel_enabled off gRPC stream health.

        Preserves the old HTTP-era /system_status connected/disconnected
        semantics -- only now driven by the ingest stream's own connect/
        disconnect events instead of POST response codes. Only acts (logs,
        publishes) on an actual transition: both the snapshot and topics
        stream threads call this independently, so it fires far more often
        than the connectivity state actually changes.
        """
        was_down = self._backend_down
        self._backend_down = not connected
        if connected and was_down:
            self.get_logger().info("Backend reconnected — telemetry resumed")
            self._publish_backend_status("connected")
        elif not connected and not was_down:
            self.get_logger().warning("Backend unreachable")
            self._publish_backend_status("disconnected")
        self._sync_channel_param("telemetry_channel_enabled", connected)

    def _sync_channel_param(self, name: str, value: bool) -> None:
        """Keep a ROS2 bool param in sync with actual channel state.

        Guarded by _syncing_channel_param so this internal set_parameters
        call doesn't re-enter _on_set_parameters and try to restart/stop the
        channel a second time (it would otherwise recurse indefinitely).
        """
        if self.get_parameter(name).value == value:
            return
        self._syncing_channel_param = True
        try:
            self.set_parameters([Parameter(name, Parameter.Type.BOOL, value)])
        finally:
            self._syncing_channel_param = False

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        """Local recovery surface for both gRPC channels.

        `ros2 param set /telemetry_bridge command_channel_enabled true` is
        the only way to re-enable the command channel after a remote
        DISABLE_COMMAND_CHANNEL command (see command_channel.py's docstring
        -- that command is deliberately one-way over gRPC).
        """
        if self._syncing_channel_param:
            return SetParametersResult(successful=True)
        for param in params:
            if param.name == "command_channel_enabled":
                old = self.get_parameter("command_channel_enabled").value
                if param.value and not old:
                    self._command_channel.restart()
                elif not param.value and old:
                    self._command_channel.stop()
            elif param.name == "telemetry_channel_enabled":
                old = self.get_parameter("telemetry_channel_enabled").value
                if param.value and not old:
                    self._telemetry_channel.restart()
                elif not param.value and old:
                    self._telemetry_channel.stop()
        return SetParametersResult(successful=True)

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
            if gap > _DIAG_UI_SUMMARY_GAP_S:
                self.get_logger().warning(f"[DIAG] ui_summary publish gap: {gap:.2f}s (expected ~0.1s)")
        self._last_ui_summary_publish_time = now_monotonic

        d0 = time.monotonic()
        front = left = right = 0.0
        if self._latest_scan is not None:
            c = _lidar_clearances(self._latest_scan.ranges, self._oled_sector_half_fov_rad)
            front = c.front_m * 100
            left = c.left_m * 100
            right = c.right_m * 100
        d1 = time.monotonic()

        yaw = 0.0
        if self._latest_imu is not None:
            q = self._latest_imu.orientation
            yaw = math.degrees(self._quaternion_to_yaw(q.x, q.y, q.z, q.w))
        d2 = time.monotonic()

        detection = _best_detection(self._latest_vision) if self._latest_vision else None
        class_id, confidence = detection if detection is not None else (None, None)
        d3 = time.monotonic()

        msg = String()
        msg.data = TelemetrySummaryWire(
            lidar_front_cm=front,
            lidar_left_cm=left,
            lidar_right_cm=right,
            gyro_yaw_deg=yaw,
            best_detection_class_id=class_id,
            best_detection_confidence=confidence,
        ).model_dump_json()
        self._ui_summary_pub.publish(msg)
        d4 = time.monotonic()

        # TEMP DIAGNOSTIC (2026-07-28): times this function's OWN body, not
        # just the gap before it -- the gap-only measurement above can't
        # tell "the timer fired late" apart from "the previous invocation
        # itself took several seconds to return" (the next tick's measured
        # gap includes that time either way). Remove once root-caused.
        own_total = d4 - d0
        if own_total > _DIAG_UI_SUMMARY_SLOW_S:
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
        if self._latest_joints and self._latest_ackermann_cmd:
            motor_snapshot = MotorStateSnapshot(
                steering_angle_deg=float(self._latest_joints.position[0]) if len(self._latest_joints.position) > 0 else 0.0,
                drive_speed=self._latest_ackermann_cmd.drive.speed,
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
            vision_detections = [
                _VisionDetectionPayload(
                    className=det.class_name,
                    confidence=det.confidence,
                    bbox=list(det.bbox),
                )
                for det in self._latest_vision
            ]

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
        if self._latest_ackermann_cmd:
            speed = self._latest_ackermann_cmd.drive.speed

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
        """Stop both gRPC channel threads before teardown."""
        self._command_channel.stop()
        self._telemetry_channel.stop()
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
