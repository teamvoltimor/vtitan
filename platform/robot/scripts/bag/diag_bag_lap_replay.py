"""Replay the real LapDetector over a bag, for every candidate start section.

``diag_bag_lap_gate.py`` counts raw start-line crossings, which is enough when
the waypoint ring is the thing that froze. When the ring wraps correctly and
laps still do not count, the question is narrower: given the ACTUAL waypoint
wraps the bag recorded, which start section would have let LapDetector fire, and
which gate refused the one that was configured.

This drives the real class -- normal from TRAVEL_DIRS, the negative-to-
non-negative dot test, the current-section equality test and the
waypoint-pending latch -- rather than reimplementing them.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_lap_replay.py \
        data/live/runs/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction, Section
from shared.domain.models import Waypoint

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows, measured_start, posed_rows, settled_direction
from scripts.common.tables import print_table
from src.navigation.race_tracker import TRAVEL_DIRS, LapDetector


def main() -> None:
    parser = create_bag_parser("TODO: add description")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    posed = posed_rows(rows)
    if not posed:
        print("no posed samples")
        return

    direction = settled_direction(rows)
    if direction is None:
        print("direction never settled -- lap counting was never reachable")
        return
    origin = (posed[0][1].pose_x, posed[0][1].pose_y)

    print(f"== {args.bag_dir.name}  direction={direction.value}  posed samples={len(posed)}")
    print(f"first posed sample (stands in for the start origin): ({origin[0]:.2f}, {origin[1]:.2f})")

    # The bag records the index, not the wrap event; recover the wraps the same
    # way the node's own bookkeeping does -- a large backwards step.
    wraps = []
    prev_idx = None
    for t, snap in posed:
        idx = snap.waypoint_index
        if not isinstance(idx, int):
            continue
        if prev_idx is not None and idx < prev_idx - 5:
            wraps.append(t)
        prev_idx = idx
    print(f"waypoint wraps at: {', '.join(f'{w:.0f}s' for w in wraps) or 'none'}")

    corridors = {snap.current_corridor for _, snap in posed}
    print(f"current_corridor values seen: {sorted(c for c in corridors if c)}")

    print("\nreplaying the real LapDetector per candidate start section:")
    rows = []
    for section in Section:
        detector = LapDetector(start_pos=Waypoint(*origin), start_section=section, direction=direction)
        normal = TRAVEL_DIRS[(section, direction)]
        laps: list[float] = []
        geo_only = 0
        prev_idx = None
        for t, snap in posed:
            idx = snap.waypoint_index
            if isinstance(idx, int):
                if prev_idx is not None and idx < prev_idx - 5:
                    detector.notify_waypoint_wrapped()
                prev_idx = idx
            corridor = snap.current_corridor
            if corridor is None:
                continue
            before = detector._prev_dot  # noqa: SLF001 - diagnosing the gate, not using it
            if detector.update(Waypoint(snap.pose_x, snap.pose_y), corridor):
                laps.append(t)
            elif before is not None and before < 0.0:
                ox, oy = origin
                nx, ny = normal
                dot = (snap.pose_x - ox) * nx + (snap.pose_y - oy) * ny
                if dot >= 0.0 and corridor is section:
                    geo_only += 1
        times = ", ".join(f"{t:.0f}s" for t in laps[:6])
        if laps:
            blocked = ""
        elif geo_only:
            blocked = f"crossed {geo_only}x in-section but waypoint latch was down"
        else:
            blocked = "never crossed the line while inside this section"
        rows.append((section.value, str(normal), len(laps), times, blocked))
    print_table(rows, ["start_section", "normal", "laps", "lap_times", "blocked_by"])


if __name__ == "__main__":
    main()
