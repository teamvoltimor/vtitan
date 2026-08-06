"""Print a sampled /nav_debug timeline from a recorded race bag.

Usage:
    pixi run -e dev python scripts/diag_bag_timeline.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--every 1.0]
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--every", type=float, default=1.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    next_print = 0.0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9
        if topic != "/nav_debug":
            continue
        if ts < next_print:
            continue
        next_print += args.every
        payload = json.loads(deserialize_message(data, String).data)
        print(
            f"{ts:7.2f}s phase={payload.get('phase')!s:14} corridor={payload.get('current_corridor')!s:6} "
            f"wp={payload.get('waypoint_index')!s:4} laps={payload.get('laps_completed')} "
            f"pose=({payload.get('pose_x'):.3f},{payload.get('pose_y'):.3f},{payload.get('pose_yaw'):.2f}) "
            f"fwd={payload.get('forward_clearance_m')!s:6} min_r={payload.get('min_lidar_range_m')!s:6} "
            f"risk={payload.get('risk')!s:6} stuck={payload.get('is_stuck')} stuck_cnt={payload.get('stuck_count')}",
        )


if __name__ == "__main__":
    main()
