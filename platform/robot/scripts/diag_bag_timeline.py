"""Print a sampled /nav_debug timeline from a recorded race bag.

Usage:
    pixi run -e dev python scripts/diag_bag_timeline.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--every 1.0]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import open_reader, read_nav_debug_rows

_DEFAULT_SAMPLE_INTERVAL_S = 1.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--every", type=float, default=_DEFAULT_SAMPLE_INTERVAL_S)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, _topics = read_nav_debug_rows(reader)

    next_print = 0.0
    for ts, snap in rows:
        if ts < next_print:
            continue
        next_print += args.every
        print(
            f"{ts:7.2f}s phase={snap.phase!s:14} corridor={snap.current_corridor!s:6} "
            f"wp={snap.waypoint_index!s:4} laps={snap.laps_completed} "
            f"pose=({snap.pose_x:.3f},{snap.pose_y:.3f},{snap.pose_yaw:.2f}) "
            f"fwd={snap.forward_clearance_m!s:6} min_r={snap.min_lidar_range_m!s:6} "
            f"risk={snap.risk!s:6} stuck={snap.is_stuck} stuck_cnt={snap.stuck_count}",
        )


if __name__ == "__main__":
    main()
