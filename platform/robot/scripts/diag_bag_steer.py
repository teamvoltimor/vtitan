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

from _bag_io import fmt_optional, open_reader, read_nav_debug_rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--every", type=float, default=0.0)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, _topics = read_nav_debug_rows(reader)

    def f6(v: float | None) -> str:
        return fmt_optional(v, "6.3f")

    def f5(v: float | None) -> str:
        return fmt_optional(v, "5.2f")

    next_print = args.start
    for ts, snap in rows:
        if not (args.start <= ts <= args.until) or ts < next_print:
            continue
        next_print += args.every
        print(
            f"{ts:6.2f}s wp={snap.waypoint_index!s:4} corr={snap.current_corridor!s:6} "
            f"pose=({f6(snap.pose_x)},{f6(snap.pose_y)},{f6(snap.pose_yaw)}) "
            f"tgt=({f6(snap.steer_target_x)},{f6(snap.steer_target_y)}) "
            f"aerr={f6(snap.angle_error_rad)} xtrack={f6(snap.crosstrack_error_m)} "
            f"look={f6(snap.lookahead_distance_m)} steer={f6(snap.commanded_steering_norm)} "
            f"belief N/S/E/W={f5(snap.belief_north_m)}/{f5(snap.belief_south_m)}/"
            f"{f5(snap.belief_east_m)}/{f5(snap.belief_west_m)}",
        )


if __name__ == "__main__":
    main()
