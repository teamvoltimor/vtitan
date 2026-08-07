"""Print a full-rate /nav_debug trace with the fields that drive escape decisions.

Usage:
    pixi run -e dev python scripts/diag_bag_trace.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --from 0 --until 12
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message


def _f(value: object, spec: str = "6.3f") -> str:
    return format(value, spec) if isinstance(value, (int, float)) else "  None"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--cmd", action="store_true", help="also print /ackermann_cmd rows")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)

    t_start = None
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
        print(
            f"{ts:7.2f}s {snap.phase!s:22} pose=({_f(snap.pose_x)},{_f(snap.pose_y)},{_f(snap.pose_yaw)}) "
            f"fwd={_f(snap.forward_clearance_m)} risk={snap.risk!s:9} erisk={snap.escape_risk!s:9} "
            f"corr={snap.current_corridor!s:6} cmd_v={_f(snap.commanded_speed_mps)} "
            f"cmd_s={_f(snap.commanded_steering_norm)} esc={snap.escape_count!s:4} stuck={snap.stuck_count!s:4}",
        )


if __name__ == "__main__":
    main()
