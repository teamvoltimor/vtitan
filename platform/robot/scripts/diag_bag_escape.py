"""Print every escape/maneuver tick from a recorded race bag.

Shows the K-turn steering sign, threat direction, escape escalation counter and
the pose the stuck detector is being fed, so a wedged run can be read as a
sequence of maneuver decisions rather than a sampled timeline.

Usage:
    pixi run -e dev python scripts/diag_bag_escape.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
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
    parser.add_argument("--until", type=float, default=1e9)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    prev_key = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9
        if topic != "/nav_debug" or ts > args.until:
            continue
        p = json.loads(deserialize_message(data, String).data)
        key = (
            p.get("phase"),
            p.get("active_maneuver_type"),
            p.get("maneuver_steering"),
            p.get("maneuver_speed_mps"),
            p.get("escape_count"),
            p.get("direction"),
        )
        if key == prev_key:
            continue
        prev_key = key
        print(
            f"{ts:7.2f}s phase={p.get('phase')!s:22} man={p.get('active_maneuver_type')!s:16} "
            f"steer={p.get('maneuver_steering')!s:7} spd={p.get('maneuver_speed_mps')!s:7} "
            f"frames={p.get('maneuver_frames_left')!s:4} esc={p.get('escape_count')!s:3} "
            f"dir={p.get('direction')!s:18} pose=({p.get('pose_x'):.3f},{p.get('pose_y'):.3f},{p.get('pose_yaw'):.2f}) "
            f"min_r={p.get('min_lidar_range_m')!s:8} stuck={p.get('stuck_count')}",
        )


if __name__ == "__main__":
    main()
