"""Show why a blind run never committed to a travel direction.

The navigator only plans waypoints once it has inferred which way round the
track it is going. When a bag stays in blind_creep with direction=null for the
whole race, the vote accumulator or its gate is at fault. This prints the vote
counts over time, the per-tick verdict, and the left/right ranges the verdict is
derived from, so it is clear whether the votes never accumulated or accumulated
and were never accepted.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_direction_gate.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import print_table


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    parser.add_argument("--every", type=float, default=20.0)
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    print(f"== {args.bag_dir.name}  samples={len(rows)}")

    verdicts = Counter(str(snap.direction_gate_verdict) for _, snap in rows)
    print("\ndirection_gate_verdict:")
    for val, count in verdicts.most_common():
        print(f"  {count:6d}  {val}")

    numeric_cw = [snap.direction_votes_clockwise for _, snap in rows if snap.direction_votes_clockwise is not None]
    numeric_ccw = [
        snap.direction_votes_counterclockwise for _, snap in rows if snap.direction_votes_counterclockwise is not None
    ]
    if numeric_cw:
        print(f"\nvotes clockwise:        min={min(numeric_cw)} max={max(numeric_cw)} last={numeric_cw[-1]}")
    if numeric_ccw:
        print(f"votes counterclockwise: min={min(numeric_ccw)} max={max(numeric_ccw)} last={numeric_ccw[-1]}")

    def stats(key: str) -> str:
        vals = [getattr(snap, key) for _, snap in rows]
        seen = [v for v in vals if isinstance(v, (int, float))]
        nulls = sum(1 for v in vals if v is None)
        if not seen:
            return f"all null ({nulls})"
        seen_sorted = sorted(seen)
        median = seen_sorted[len(seen_sorted) // 2]
        return (
            f"n={len(seen)} nulls={nulls} min={min(seen):.2f} "
            f"median={median:.2f} max={max(seen):.2f}"
        )

    print("\nranges the verdict reads:")
    for key in ("direction_left_range_m", "direction_right_range_m", "min_lidar_range_m"):
        print(f"  {key:26s} {stats(key)}")

    print(f"\ntimeline (every {args.every:.0f}s):")
    cols = (
        "direction_votes_clockwise",
        "direction_votes_counterclockwise",
        "direction_gate_verdict",
        "direction_left_range_m",
        "direction_right_range_m",
        "current_corridor",
    )
    col_labels = tuple(c.replace("direction_", "")[:12] for c in cols)
    table_rows = []
    next_t = 0.0
    for t, snap in rows:
        if t < next_t:
            continue
        next_t = t + args.every
        row = [t] + [getattr(snap, key) for key in cols]
        table_rows.append(row)
    if table_rows:
        print_table(table_rows, ["t"] + list(col_labels))


if __name__ == "__main__":
    main()
