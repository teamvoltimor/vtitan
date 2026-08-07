"""Print a full-rate /nav_debug trace with the fields that drive escape decisions.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_trace.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --from 0 --until 12
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, fmt_optional, open_reader, print_table


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--cmd", action="store_true", help="also print /ackermann_cmd rows")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)

    t_start = None
    nav_rows = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = elapsed_seconds(t, t_start)
        if not (args.start <= ts <= args.until):
            continue
        if topic == Topics.ACKERMANN_CMD and args.cmd:
            msg = deserialize_message(data, AckermannDriveStamped)
            print(f"{ts:7.2f}s CMD  speed={msg.drive.speed:6.3f} steer_rad={msg.drive.steering_angle:6.3f}")
            continue
        if topic != Topics.NAV_DEBUG:
            continue
        snap = decode_nav_debug(data)
        nav_rows.append((
            ts,
            snap.phase,
            snap.pose_x,
            snap.pose_y,
            snap.pose_yaw,
            snap.forward_clearance_m,
            snap.risk,
            snap.escape_risk,
            snap.current_corridor,
            snap.commanded_speed_mps,
            snap.commanded_steering_norm,
            snap.escape_count,
            snap.stuck_count
        ))
    if nav_rows:
        print_table(nav_rows, ["t", "phase", "x", "y", "yaw", "fwd", "risk", "erisk", "corr", "cmd_v", "cmd_s", "esc", "stuck"])


if __name__ == "__main__":
    main()
