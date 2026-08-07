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

from _bag_io import open_reader, print_table, read_nav_debug_rows
from shared.domain.models import NavigatorDebugSnapshot

# Real NavigatorDebugSnapshot field names (shared.domain.models). An earlier
# version of this tuple used names that didn't match the schema at all
# ("state", "mode", "challenge", "section", "escape_state",
# "corridor_width_belief") -- with the bag /nav_debug payload read as an
# untyped dict, a typo like that just silently vanished from `present` below
# instead of failing loudly.
WATCH = (
    "phase",
    "direction",
    "current_corridor",
    "waypoint_index",
    "laps_completed",
    "commanded_speed_mps",
    "direction_votes_clockwise",
    "direction_votes_counterclockwise",
    "active_maneuver_type",
    "corridor_width_belief_m",
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--every", type=float, default=20.0, help="seconds between printed samples")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, topics = read_nav_debug_rows(reader)

    print(f"== {args.bag_dir.name}")
    print("topics:")
    for name, count in topics.most_common():
        print(f"  {count:7d}  {name}")

    if not rows:
        print("no /nav_debug samples")
        return

    print(f"\nnav_debug samples: {len(rows)}  span: {rows[0][0]:.1f}..{rows[-1][0]:.1f}s")

    populated = sorted(
        name
        for name in NavigatorDebugSnapshot.model_fields
        if any(getattr(snap, name) is not None for _, snap in rows)
    )
    print(f"\nfields with at least one non-null value: {', '.join(populated)}")

    present = [k for k in WATCH if k in populated]
    print("\ndistinct values per watched key:")
    for key in present:
        vals = Counter(json.dumps(getattr(snap, key)) for _, snap in rows)
        shown = ", ".join(f"{v}x{c}" for v, c in vals.most_common(6))
        print(f"  {key:24s} {shown}")

    print(f"\ntimeline (every {args.every:.0f}s):")
    table_rows = []
    next_t = 0.0
    for t, snap in rows:
        if t < next_t:
            continue
        next_t = t + args.every
        row = [t] + [getattr(snap, key) for key in present]
        table_rows.append(row)
    if table_rows:
        print_table(table_rows, ["t"] + present)


if __name__ == "__main__":
    main()
