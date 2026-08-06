"""Show why a blind run never committed to a travel direction.

The navigator only plans waypoints once it has inferred which way round the
track it is going. When a bag stays in blind_creep with direction=null for the
whole race, the vote accumulator or its gate is at fault. This prints the vote
counts over time, the per-tick verdict, and the left/right ranges the verdict is
derived from, so it is clear whether the votes never accumulated or accumulated
and were never accepted.

Usage:
    pixi run -e dev python scripts/diag_bag_direction_gate.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--every", type=float, default=20.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t0 = None
    rows: list[dict] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != "/nav_debug":
            continue
        p = json.loads(deserialize_message(data, String).data)
        p["_t"] = (t - t0) / 1e9
        rows.append(p)

    print(f"== {args.bag_dir.name}  samples={len(rows)}")

    verdicts = Counter(str(r.get("direction_gate_verdict")) for r in rows)
    print("\ndirection_gate_verdict:")
    for val, count in verdicts.most_common():
        print(f"  {count:6d}  {val}")

    cw = [r.get("direction_votes_clockwise") for r in rows]
    ccw = [r.get("direction_votes_counterclockwise") for r in rows]
    numeric_cw = [v for v in cw if isinstance(v, (int, float))]
    numeric_ccw = [v for v in ccw if isinstance(v, (int, float))]
    if numeric_cw:
        print(f"\nvotes clockwise:        min={min(numeric_cw)} max={max(numeric_cw)} last={numeric_cw[-1]}")
    if numeric_ccw:
        print(f"votes counterclockwise: min={min(numeric_ccw)} max={max(numeric_ccw)} last={numeric_ccw[-1]}")

    def stats(key: str) -> str:
        vals = [r.get(key) for r in rows]
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
    print("     t  " + "  ".join(f"{c.replace('direction_', '')[:12]:>12s}" for c in cols))
    next_t = 0.0
    for row in rows:
        if row["_t"] < next_t:
            continue
        next_t = row["_t"] + args.every
        cells = []
        for key in cols:
            val = row.get(key)
            cells.append(f"{val:12.2f}" if isinstance(val, float) else f"{str(val)[:12]:>12s}")
        print(f"{row['_t']:6.1f}  " + "  ".join(cells))


if __name__ == "__main__":
    main()
