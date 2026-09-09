"""Shared bag-replay mechanics for the diag_bag_*.py diagnostic scripts.

Every diag_bag_*.py script replays a recorded ROS2 mcap bag; this module holds
the parts that were previously copy-pasted into each one -- opening the
reader, decoding /scan and /nav_debug messages -- so each script only has to
write its own analysis.

Decoded values reuse the same types the real navigation code already uses,
rather than inventing anonymous tuples/dicts:

* /scan decodes to `LidarScan` (`src.navigation.ports`) -- its own docstring
  says it exists specifically to replace "the previous anonymous
  ``tuple[list[float], list[float]]`` scan".
* /nav_debug decodes to `NavigatorDebugSnapshot` (`shared.domain.models`) --
  the exact pydantic model `TrackNavigatorNode` publishes
  (`self._debug_pub.publish(String(data=self._latest_debug.model_dump_json()))`),
  so field names/optionality/types are read from one source of truth instead
  of re-typed as bare dict-key strings per script.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import rosbag2_py
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan
from shared.config.constants import RobotSpecs
from shared.domain.models import Detection, NavigatorDebugSnapshot, SignColor
from std_msgs.msg import Float32, String

from src.navigation.ports import LidarScan

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.enums import Direction


class Topics:
    """Topic names shared by the diag_bag_*.py scripts."""

    NAV_DEBUG = "/nav_debug"
    SCAN = "/scan"
    VISION_DETECTIONS = "/vision/detections"
    ACKERMANN_CMD = "/ackermann_cmd"
    ROBOT_STATE = "/robot_state"
    IMU_DATA = "/imu/data"
    MOTOR_DRIVE_SPEED = "/motor/drive_speed"
    MOTOR_STEERING_POSITION = "/motor/steering_position"
    MOTOR_PREFIX = "/motor/"
    """Prefix match for any /motor/* feedback topic, not just the two named above."""


def open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    """Open a recorded mcap bag for sequential replay."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def elapsed_seconds(t: int, t0: int) -> float:
    """Nanosecond bag timestamp `t` relative to the bag's first timestamp `t0`."""
    return (t - t0) / 1e9


def decode_scan(
    msg: LaserScan,
    yaw_offset_rad: float,
    max_range_m: float = RobotSpecs.LIDAR_MAX_RANGE,
) -> LidarScan:
    """Decode a LaserScan into a `LidarScan` (robot-frame ranges/angles).

    Dropouts (non-finite ranges) are filled with `max_range_m` -- the same
    substitution `ROS2HardwareGateway._lidar_callback` makes on the real
    node. `yaw_offset_rad` is the same mount-rotation correction the gateway
    applies; pass `src.ros2.navigation.ros2_hardware_gateway._LIDAR_YAW_OFFSET_RAD`.
    """
    ranges = [v if math.isfinite(v) else max_range_m for v in msg.ranges]
    n = len(ranges)
    angles = [
        msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1) + yaw_offset_rad
        for i in range(n)
    ]
    return LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(angles))


def decode_nav_debug(data: bytes) -> NavigatorDebugSnapshot:
    """Decode a /nav_debug String message into the snapshot it was published from."""
    return NavigatorDebugSnapshot.model_validate_json(deserialize_message(data, String).data)


def decode_detections(payload: list[dict]) -> list[Detection]:
    """Rebuild typed :class:`Detection` models from the /vision/detections payload.

    The topic publishes a JSON list of detection dicts (the vision node's own
    wire shape, so the schema lives on the vision side, not here). Records
    that fail to rebuild are SKIPPED, not raised: a detector frame is advisory
    evidence and one malformed dict is routine (a class name of UNKNOWN, a
    bbox dropped by the publisher), while failing the whole replay would hide
    99 good detections behind 1 bad one.

    The class of a record is read from ``class_name``, falling back to ``class``
    (two generations of publisher both produced this topic); a record with
    neither is not a detection.
    """
    out: list[Detection] = []
    for d in payload:
        try:
            colour = SignColor(d["class_name"]) if "class_name" in d else SignColor(d["class"])
        except (KeyError, ValueError):
            continue
        try:
            x_min, y_min, x_max, y_max = (float(v) for v in d.get("bbox", ()))
        except (TypeError, ValueError):
            continue
        out.append(
            Detection(
                class_name=colour,
                confidence=float(d.get("confidence", 0.0)),
                bbox=(x_min, y_min, x_max, y_max),
                x=float(d.get("x", 0.0)),
                y=float(d.get("y", 0.0)),
                width=float(d.get("width", 0.0)),
                height=float(d.get("height", 0.0)),
                area=float(d.get("area", 0.0)),
            )
        )
    return out


def read_vision_rows(
    bag_dir: Path,
) -> tuple[list[tuple[float, NavigatorDebugSnapshot]], list[tuple[float, list[dict]]]]:
    """Replay one bag once, collecting /nav_debug rows AND /vision/detections frames.

    Returns `(rows, frames)`, each `(elapsed_s, payload)`: rows decode to the
    navigator snapshot, frames to the raw detection dicts a bag stores (pass
    them to :func:`decode_detections`, which response-frames lazily and skips
    malformed records). Scripts replaying the router over detections plus pose
    need both in one pass; reading the bag twice costs a minute+ on the
    hardware bags.
    """
    reader = open_reader(bag_dir)
    t0 = None
    rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    frames: list[tuple[float, list[dict]]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
        elif topic == Topics.VISION_DETECTIONS:
            frames.append((rel, json.loads(deserialize_message(data, String).data) or []))
    return rows, frames


def read_nav_debug_rows(
    reader: rosbag2_py.SequentialReader,
) -> tuple[list[tuple[float, NavigatorDebugSnapshot]], Counter[str]]:
    """Replay a bag, collecting every /nav_debug snapshot and a topic histogram.

    Each returned row pairs the decoded snapshot with elapsed seconds since
    the bag's first message. The topic Counter covers every message in the
    bag, not just /nav_debug -- callers that don't need it can discard it.
    Consumes the reader.
    """
    t0 = None
    rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    topics: Counter[str] = Counter()
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        topics[topic] += 1
        if topic != Topics.NAV_DEBUG:
            continue
        rows.append((elapsed_seconds(t, t0), decode_nav_debug(data)))
    return rows, topics


def load_nav_debug_rows(
    bag_dir: Path,
) -> tuple[list[tuple[float, NavigatorDebugSnapshot]], Counter[str]]:
    """Open a bag and load all /nav_debug rows in one call.

    Convenience wrapper collapsing the `open_reader()` + `read_nav_debug_rows()`
    pair used in 9+ scripts. Returns the same tuple: `(rows, topic_counts)`.
    """
    return read_nav_debug_rows(open_reader(bag_dir))


def posed_rows(
    rows: Sequence[tuple[float, NavigatorDebugSnapshot]],
) -> list[tuple[float, NavigatorDebugSnapshot]]:
    """The rows carrying a usable pose.

    Every phase before the navigator has a fix publishes a snapshot with
    ``pose_x``/``pose_y`` still None, and any analysis that indexes position
    has to drop those first. Four lap diagnostics each wrote the same
    isinstance pair inline.
    """
    return [(t, s) for t, s in rows if isinstance(s.pose_x, (int, float)) and isinstance(s.pose_y, (int, float))]


def settled_direction(rows: Sequence[tuple[float, NavigatorDebugSnapshot]]) -> Direction | None:
    """The direction the run committed to, or None if it never settled.

    A bag records the direction only once inference has settled, so the first
    non-null value is the committed one. None is a finding rather than an
    error: it means the round never left blind creep.
    """
    return next((s.direction for _, s in rows if s.direction), None)


def measured_start(rows: Sequence[tuple[float, NavigatorDebugSnapshot]]) -> tuple[float, float] | None:
    """Where ``measure_start_pose`` put the start, or None if it refused.

    None is normal, not a gap in the bag: the measurement declines when a ray
    is blocked (an operator still standing over the robot is the usual case)
    and the node falls back to the assumed start.
    """
    return next(
        ((s.start_measured_x, s.start_measured_y) for _, s in rows if isinstance(s.start_measured_x, (int, float))),
        None,
    )


def read_bag(
    reader: rosbag2_py.SequentialReader,
    yaw_offset_rad: float,
    max_range_m: float = RobotSpecs.LIDAR_MAX_RANGE,
) -> tuple[list[tuple[float, LidarScan]], list[tuple[float, NavigatorDebugSnapshot]]]:
    """Replay a bag once, decoding both /scan and /nav_debug.

    Returns `(scans, nav_debug_rows)`, each a list of `(elapsed_seconds, ...)`
    pairs. Consumes the reader.
    """
    t0 = None
    scans: list[tuple[float, LidarScan]] = []
    rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.SCAN:
            msg = deserialize_message(data, LaserScan)
            scans.append((rel, decode_scan(msg, yaw_offset_rad, max_range_m)))
        elif topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
    return scans, rows


def quaternion_yaw(q) -> float:  # noqa: ANN001
    """Yaw from an IMU orientation quaternion.

    The gyro cannot be used: the robot runs ``bno08x_uart_rvc_node`` and BNO08x
    UART-RVC mode provides no angular velocity at all, so ``/imu/data``
    publishes ``angular_velocity`` as zeros with covariance -1 (the ROS
    "unavailable" convention). Differentiating this quaternion at ~166 Hz is the
    supported way to get a yaw rate, not a workaround.

    ``pose_yaw`` is NOT a substitute -- it is localizer-fused and damped, and
    differentiating it understates the achieved yaw rate by roughly half.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass(slots=True)
class MotionStreams:
    """The four ``(elapsed_s, value)`` series that describe how the chassis moved.

    Everything needed to compare a recorded run against
    :class:`~src.simulation.kinematics.AckermannKinematics`: what was commanded,
    what the actuators reported back, and what the IMU says actually happened.
    Collected in one replay pass because a bag is large and reading it four
    times to answer four questions is the slowest thing these scripts do.
    """

    imu_yaw_rad: list[tuple[float, float]] = field(default_factory=list)
    """Chassis yaw, differentiate for the ACHIEVED yaw rate."""

    cmd_speed_mps: list[tuple[float, float]] = field(default_factory=list)
    cmd_steer_rad: list[tuple[float, float]] = field(default_factory=list)
    """Commanded WHEEL angle, the frame ``/ackermann_cmd`` publishes in."""

    steer_pos_deg: list[tuple[float, float]] = field(default_factory=list)
    """Steering feedback in WHEEL degrees. NOT an independent measurement on a
    servo chassis -- ``ServoDriver.get_steering_position`` returns the last
    commanded angle, so this is the command echoed back through the linkage
    ratio and the servo trim, and it cannot show servo tracking lag."""

    drive_speed_dps: list[tuple[float, float]] = field(default_factory=list)
    """Encoder-derived WHEEL speed in deg/s (not motor-shaft, not rpm)."""

    def drive_speed_mps(self) -> list[tuple[float, float]]:
        """``drive_speed_dps`` converted to m/s at the wheel rim."""
        scale = math.radians(1.0) * RobotSpecs.WHEEL_RADIUS
        return [(t, dps * scale) for t, dps in self.drive_speed_dps]


def read_motion_streams(bag_dir: Path) -> MotionStreams:
    """Replay a bag once, collecting every command/actuator/IMU series."""
    reader = open_reader(bag_dir)
    streams = MotionStreams()
    t0 = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.IMU_DATA:
            streams.imu_yaw_rad.append((rel, quaternion_yaw(deserialize_message(data, Imu).orientation)))
        elif topic == Topics.ACKERMANN_CMD:
            drive = deserialize_message(data, AckermannDriveStamped).drive
            streams.cmd_speed_mps.append((rel, drive.speed))
            streams.cmd_steer_rad.append((rel, drive.steering_angle))
        elif topic == Topics.MOTOR_STEERING_POSITION:
            streams.steer_pos_deg.append((rel, deserialize_message(data, Float32).data))
        elif topic == Topics.MOTOR_DRIVE_SPEED:
            streams.drive_speed_dps.append((rel, deserialize_message(data, Float32).data))
    return streams


def create_bag_parser(description: str = "") -> argparse.ArgumentParser:
    """Standard argument parser for diagnostic scripts that analyze a single bag.

    Adds a positional `bag_dir` argument (Path). Scripts can extend with
    optional arguments via `.add_argument()`.

    Example:
        parser = create_bag_parser("Analyze corner overshoots")
        parser.add_argument("--max-corners", type=int, default=8)
        args = parser.parse_args()
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("bag_dir", type=Path, help="Path to rosbag directory")
    return parser


def create_bags_parser(
    description: str = "", formatter_class: type[argparse.HelpFormatter] = argparse.HelpFormatter
) -> argparse.ArgumentParser:
    """Standard argument parser for diagnostic scripts that analyze multiple bags.

    Adds a positional `bag_dirs` argument (list of Paths). Scripts can extend
    with optional arguments via `.add_argument()`. Pass
    `formatter_class=argparse.RawDescriptionHelpFormatter` for a script whose
    docstring is pre-formatted (multi-paragraph, code examples) rather than
    prose argparse should rewrap.

    Example:
        parser = create_bags_parser("Compare two runs")
        args = parser.parse_args()
        for bag_dir in args.bag_dirs:
            ...
    """
    parser = argparse.ArgumentParser(description=description, formatter_class=formatter_class)
    parser.add_argument("bag_dirs", type=Path, nargs="+", help="Paths to rosbag directories")
    return parser


