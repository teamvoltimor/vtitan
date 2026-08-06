"""Print the pure-pursuit inputs behind each steering command in a race bag.

Answers "why did it not turn": the waypoint it was chasing, the target point,
the heading error that steering is proportional to, and the corridor-width
belief the plan was built from.

Usage:
    pixi run -e dev python scripts/diag_bag_steer.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --until 7
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import open_reader, read_nav_debug_rows


def _f(value: object, spec: str = "6.3f") -> str:
    return format(value, spec) if isinstance(value, (int, float)) else "  None"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--every", type=float, default=0.0)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, _topics = read_nav_debug_rows(reader)

    next_print = args.start
    for ts, snap in rows:
        if not (args.start <= ts <= args.until) or ts < next_print:
            continue
        next_print += args.every
        print(
            f"{ts:6.2f}s wp={snap.waypoint_index!s:4} corr={snap.current_corridor!s:6} "
            f"pose=({_f(snap.pose_x)},{_f(snap.pose_y)},{_f(snap.pose_yaw)}) "
            f"tgt=({_f(snap.steer_target_x)},{_f(snap.steer_target_y)}) "
            f"aerr={_f(snap.angle_error_rad)} xtrack={_f(snap.crosstrack_error_m)} "
            f"look={_f(snap.lookahead_distance_m)} steer={_f(snap.commanded_steering_norm)} "
            f"belief N/S/E/W={_f(snap.belief_north_m, '5.2f')}/{_f(snap.belief_south_m, '5.2f')}/"
            f"{_f(snap.belief_east_m, '5.2f')}/{_f(snap.belief_west_m, '5.2f')}",
        )


if __name__ == "__main__":
    main()
