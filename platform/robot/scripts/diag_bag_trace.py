"""Print a full-rate /nav_debug trace with the fields that drive escape decisions.

Usage:
    pixi run -e dev python scripts/diag_bag_trace.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --from 0 --until 12
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def _f(value: object, spec: str = "6.3f") -> str:
    return format(value, spec) if isinstance(value, (int, float)) else "  None"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--cmd", action="store_true", help="also print /ackermann_cmd rows")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9
        if not (args.start <= ts <= args.until):
            continue
        if topic == "/ackermann_cmd" and args.cmd:
            msg = deserialize_message(data, AckermannDriveStamped)
            print(f"{ts:7.2f}s CMD  speed={msg.drive.speed:6.3f} steer_rad={msg.drive.steering_angle:6.3f}")
            continue
        if topic != "/nav_debug":
            continue
        p = json.loads(deserialize_message(data, String).data)
        print(
            f"{ts:7.2f}s {p.get('phase')!s:22} pose=({_f(p.get('pose_x'))},{_f(p.get('pose_y'))},{_f(p.get('pose_yaw'))}) "
            f"fwd={_f(p.get('forward_clearance_m'))} risk={p.get('risk')!s:9} erisk={p.get('escape_risk')!s:9} "
            f"corr={p.get('current_corridor')!s:6} cmd_v={_f(p.get('commanded_speed_mps'))} "
            f"cmd_s={_f(p.get('commanded_steering_norm'))} esc={p.get('escape_count')!s:4} stuck={p.get('stuck_count')!s:4}",
        )


if __name__ == "__main__":
    main()
