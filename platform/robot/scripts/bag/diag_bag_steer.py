"""Print the pure-pursuit inputs behind each steering command in a race bag.

Answers "why did it not turn": the waypoint it was chasing, the target point,
the heading error that steering is proportional to, and the corridor-width
belief the plan was built from.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_steer.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX --until 7
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import fmt_optional, print_table


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--until", type=float, default=1e9)
    parser.add_argument("--every", type=float, default=0.0)
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    def f6(v: float | None) -> str:
        return fmt_optional(v, "6.3f")

    def f5(v: float | None) -> str:
        return fmt_optional(v, "5.2f")

    table_rows = []
    next_print = args.start
    for ts, snap in rows:
        if not (args.start <= ts <= args.until) or ts < next_print:
            continue
        next_print += args.every
        table_rows.append((
            ts,
            snap.waypoint_index,
            snap.current_corridor,
            f"{snap.pose_x or 0:.3f}",
            f"{snap.pose_y or 0:.3f}",
            f"{snap.pose_yaw or 0:.3f}",
            f"{snap.steer_target_x or 0:.3f}",
            f"{snap.steer_target_y or 0:.3f}",
            snap.angle_error_rad,
            snap.crosstrack_error_m,
            snap.lookahead_distance_m,
            snap.commanded_steering_norm,
            snap.belief_north_m,
            snap.belief_south_m,
            snap.belief_east_m,
            snap.belief_west_m,
        ))
    print_table(
        table_rows,
        ["t", "wp", "corr", "x", "y", "yaw", "tgt_x", "tgt_y", "aerr", "xtrack", "look", "steer", "N", "S", "E", "W"]
    )


if __name__ == "__main__":
    main()
