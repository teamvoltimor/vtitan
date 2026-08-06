"""Print how the navigator's high-level state evolves across a race bag.

When a bag has poses but no direction and no waypoint index, the failure is
upstream of the lap gate: the navigator never committed to a travel direction,
so it never planned a path. This dumps the fields that decide that commitment,
sampled over time, plus the set of keys the bag actually carries.

Usage:
    pixi run -e dev python scripts/diag_bag_state_timeline.py \
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

WATCH = (
    "state",
    "phase",
    "mode",
    "challenge",
    "direction",
    "direction_votes",
    "corridor",
    "section",
    "waypoint_index",
    "laps_completed",
    "commanded_speed_mps",
    "escape_state",
    "corridor_width_belief",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--every", type=float, default=20.0, help="seconds between printed samples")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t0 = None
    rows: list[dict] = []
    topics: Counter[str] = Counter()
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        topics[topic] += 1
        if topic != "/nav_debug":
            continue
        p = json.loads(deserialize_message(data, String).data)
        p["_t"] = (t - t0) / 1e9
        rows.append(p)

    print(f"== {args.bag_dir.name}")
    print("topics:")
    for name, count in topics.most_common():
        print(f"  {count:7d}  {name}")

    if not rows:
        print("no /nav_debug samples")
        return

    print(f"\nnav_debug samples: {len(rows)}  span: {rows[0]['_t']:.1f}..{rows[-1]['_t']:.1f}s")

    keys = sorted({k for r in rows for k in r})
    print(f"\nkeys present: {', '.join(keys)}")

    present = [k for k in WATCH if k in keys]
    print("\ndistinct values per watched key:")
    for key in present:
        vals = Counter(json.dumps(r.get(key)) for r in rows)
        shown = ", ".join(f"{v}x{c}" for v, c in vals.most_common(6))
        print(f"  {key:24s} {shown}")

    print(f"\ntimeline (every {args.every:.0f}s):")
    header = "    t  " + "  ".join(f"{k[:14]:>14s}" for k in present)
    print(header)
    next_t = 0.0
    for row in rows:
        if row["_t"] < next_t:
            continue
        next_t = row["_t"] + args.every
        cells = []
        for key in present:
            val = row.get(key)
            if isinstance(val, float):
                cells.append(f"{val:14.3f}")
            else:
                cells.append(f"{str(val)[:14]:>14s}")
        print(f"{row['_t']:6.1f}  " + "  ".join(cells))


if __name__ == "__main__":
    main()
