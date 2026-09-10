"""Raw-topic trace for bags lacking /nav_debug and /scan.

Every other diag_bag_*.py script leans on /nav_debug and/or /scan. Some
bags predate one or both -- the one this was written for (2026-08-08's
145948 run) has neither, but does have /imu/data, /ackermann_cmd,
/robot_state and /motor/*. This answers the fallback question those bags
still need answered: did the robot actually turn, and what did it command
vs. feed back while doing it. Consolidates a one-off tmp_imu.py.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_imu_trace.py RUN_DIR
    pixi run -e dev python scripts/bag/diag_bag_imu_trace.py RUN_DIR --limit 50
"""

from __future__ import annotations

import logging
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from shared.config.coordinate_transform import quaternion_to_yaw
from std_msgs.msg import Float32, String

from scripts.common.bag_io import Topics, create_bag_parser, elapsed_seconds, open_reader
from scripts.common.tables import print_table

_logger = logging.getLogger(__name__)

_DEFAULT_MOTOR_ROW_LIMIT = 24
"""How many /motor/* feedback rows to print by default -- these bags run at a high
enough rate that printing every row floods the terminal; 24 was enough to see the
motion pattern in the run this was written for. Override with --limit."""


def main() -> None:
    """Replay a bag's raw IMU/motor/state/cmd topics and print each as a table."""
    parser = create_bag_parser("Raw-topic (/imu/data, /motor/*, /robot_state, /ackermann_cmd) trace for bags lacking /nav_debug or /scan.")
    parser.add_argument("--limit", type=int, default=_DEFAULT_MOTOR_ROW_LIMIT, help="max /motor/* feedback rows to print (0 = all)")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    t0: int | None = None
    imu_rows: list[tuple[float, float, float]] = []
    cmd_rows: list[tuple[float, float, float]] = []
    motor_rows: list[tuple[float, str, float]] = []
    state_rows: list[tuple[float, str]] = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.IMU_DATA:
            msg = deserialize_message(data, Imu)
            q = msg.orientation
            imu_rows.append((rel, math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w)), msg.angular_velocity.z))
        elif topic == Topics.ACKERMANN_CMD:
            msg = deserialize_message(data, AckermannDriveStamped)
            cmd_rows.append((rel, msg.drive.speed, msg.drive.steering_angle))
        elif topic == Topics.ROBOT_STATE:
            state_rows.append((rel, deserialize_message(data, String).data))
        elif topic.startswith(Topics.MOTOR_PREFIX):
            try:
                value = deserialize_message(data, Float32).data
            except Exception:  # noqa: BLE001 - deserialization failure modes are unknown-in-advance, but must be logged, not swallowed
                _logger.warning("failed to decode %s as Float32 at t=%.2fs", topic, rel, exc_info=True)
                continue
            motor_rows.append((rel, topic, value))

    print(f"robot_state: {state_rows}")

    print(f"\n/ackermann_cmd ({len(cmd_rows)}):")
    if cmd_rows:
        print_table([(f"{t:.2f}", f"{spd:+.3f}", f"{steer:+.3f}") for t, spd, steer in cmd_rows], ["t", "speed", "steer"])

    limit = len(motor_rows) if args.limit <= 0 else args.limit
    print(f"\n/motor feedback ({len(motor_rows)}, showing {min(limit, len(motor_rows))}):")
    if motor_rows:
        print_table([(f"{t:.2f}", topic, f"{value:+.4f}") for t, topic, value in motor_rows[:limit]], ["t", "topic", "value"])

    print(f"\nIMU heading ({len(imu_rows)} samples):")
    if imu_rows:
        print_table([(f"{t:.2f}", f"{yaw:.2f}", f"{gz:.4f}") for t, yaw, gz in imu_rows], ["t", "yaw_deg", "gyro_z"])


if __name__ == "__main__":
    main()
