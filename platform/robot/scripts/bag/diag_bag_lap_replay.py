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
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Direction, Section

from scripts.common.bag_io import load_nav_debug_rows, print_table
from src.navigation.race_tracker import TRAVEL_DIRS, LapDetector

_SECTIONS = {s.value: s for s in Section}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

    posed = [
        (t, snap) for t, snap in rows if isinstance(snap.pose_x, (int, float)) and isinstance(snap.pose_y, (int, float))
    ]
    if not posed:
        print("no posed samples")
        return

    direction_name = next((snap.direction for _, snap in rows if snap.direction), None)
    if direction_name is None:
        print("direction never settled -- lap counting was never reachable")
        return
    direction = Direction(direction_name)
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
    for name, section in _SECTIONS.items():
        detector = LapDetector(start_pos=origin, start_section=section, direction=direction)
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
            if corridor not in _SECTIONS:
                continue
            before = detector._prev_dot  # noqa: SLF001 - diagnosing the gate, not using it
            if detector.update((snap.pose_x, snap.pose_y), _SECTIONS[corridor]):
                laps.append(t)
            elif before is not None and before < 0.0:
                ox, oy = origin
                nx, ny = normal
                dot = (snap.pose_x - ox) * nx + (snap.pose_y - oy) * ny
                if dot >= 0.0 and _SECTIONS[corridor] is section:
                    geo_only += 1
        times = ", ".join(f"{t:.0f}s" for t in laps[:6])
        if laps:
            blocked = ""
        elif geo_only:
            blocked = f"crossed {geo_only}x in-section but waypoint latch was down"
        else:
            blocked = "never crossed the line while inside this section"
        rows.append((name, str(normal), len(laps), times, blocked))
    print_table(rows, ["start_section", "normal", "laps", "lap_times", "blocked_by"])


if __name__ == "__main__":
    main()
