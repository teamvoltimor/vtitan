"""Print every escape/maneuver tick from a recorded race bag.

Shows the K-turn steering sign, threat direction, escape escalation counter and
the pose the stuck detector is being fed, so a wedged run can be read as a
sequence of maneuver decisions rather than a sampled timeline.

Usage:
    pixi run -e dev python scripts/diag_bag_escape.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import open_reader, read_nav_debug_rows

_DEFAULT_UNTIL_S = 1e9


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--until", type=float, default=_DEFAULT_UNTIL_S)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, _topics = read_nav_debug_rows(reader)

    prev_key = None
    for ts, snap in rows:
        if ts > args.until:
            continue
        key = (
            snap.phase,
            snap.active_maneuver_type,
            snap.maneuver_steering,
            snap.maneuver_speed_mps,
            snap.escape_count,
            snap.direction,
        )
        if key == prev_key:
            continue
        prev_key = key
        print(
            f"{ts:7.2f}s phase={snap.phase!s:22} man={snap.active_maneuver_type!s:16} "
            f"steer={snap.maneuver_steering!s:7} spd={snap.maneuver_speed_mps!s:7} "
            f"frames={snap.maneuver_frames_left!s:4} esc={snap.escape_count!s:3} "
            f"dir={snap.direction!s:18} pose=({snap.pose_x:.3f},{snap.pose_y:.3f},{snap.pose_yaw:.2f}) "
            f"min_r={snap.min_lidar_range_m!s:8} stuck={snap.stuck_count}",
        )


if __name__ == "__main__":
    main()
