"""Print the pure-pursuit inputs behind each steering command in a race bag.

Answers "why did it not turn": the waypoint it was chasing, the target point,
the heading error that steering is proportional to, and the corridor-width
belief the plan was built from.

Usage:
    pixi run -e dev python scripts/diag_bag_steer.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --until 7
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def _f(value: object, spec: str = "6.3f") -> str:
    return format(value, spec) if isinstance(value, (int, float)) else "  None"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--every", type=float, default=0.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    next_print = args.start
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9
        if topic != "/nav_debug" or not (args.start <= ts <= args.until) or ts < next_print:
            continue
        next_print += args.every
        p = json.loads(deserialize_message(data, String).data)
        print(
            f"{ts:6.2f}s wp={p.get('waypoint_index')!s:4} corr={p.get('current_corridor')!s:6} "
            f"pose=({_f(p.get('pose_x'))},{_f(p.get('pose_y'))},{_f(p.get('pose_yaw'))}) "
            f"tgt=({_f(p.get('steer_target_x'))},{_f(p.get('steer_target_y'))}) "
            f"aerr={_f(p.get('angle_error_rad'))} xtrack={_f(p.get('crosstrack_error_m'))} "
            f"look={_f(p.get('lookahead_distance_m'))} steer={_f(p.get('commanded_steering_norm'))} "
            f"belief N/S/E/W={_f(p.get('belief_north_m'), '5.2f')}/{_f(p.get('belief_south_m'), '5.2f')}/"
            f"{_f(p.get('belief_east_m'), '5.2f')}/{_f(p.get('belief_west_m'), '5.2f')}",
        )


if __name__ == "__main__":
    main()
