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
import math
from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs
from shared.domain.models import NavigatorDebugSnapshot
from std_msgs.msg import String

from src.navigation.ports import LidarScan

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.enums import Direction


class Topics:
    """Topic names shared by the diag_bag_*.py scripts."""

    NAV_DEBUG = "/nav_debug"
    SCAN = "/scan"
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


