"""Replay the real LapDetector against both candidate origins, side by side.

``diag_bag_lap_replay.py`` anchors the detector at the first posed bag sample,
which stands in for the MEASURED start. That is not what the node does. When
LIDAR inference AGREES with the launch direction, _commit_direction takes its
``elif measured is not None`` branch, which re-seeds the pose frame to the
measured start but leaves the LapDetector built by _reset_for_new_race anchored
at the ASSUMED start. When inference OVERTURNS the direction, the ``if changed``
branch rebuilds the detector at the measured start.

CCW is the assumed default, so a CCW round takes the elif branch and a CW round
takes the if branch. This script asks the only question that separates those two
worlds: replayed over the same bag, does the assumed origin count laps that the
measured origin counts?

Usage:
    pixi run -e dev python scripts/bag/diag_bag_lap_origin.py \
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
from src.navigation.start_conditions import CANONICAL_SECTION, assumed_start_conditions


def _replay(posed, origin, section, direction):
    """Drive the real LapDetector over the bag, returning the lap times."""
    detector = LapDetector(start_pos=Waypoint(*origin), start_section=section, direction=direction)
    laps: list[float] = []
    geo_only = 0
    prev_idx = None
    normal = TRAVEL_DIRS[(section, direction)]
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
            dot = (snap.pose_x - origin[0]) * normal[0] + (snap.pose_y - origin[1]) * normal[1]
            if dot >= 0.0 and corridor is section:
                geo_only += 1
    return laps, geo_only


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

    # What the node believed before it measured anything: the launch default.
    # Both directions are shown because the bag records only the SETTLED
    # direction, and which branch _commit_direction took depends on whether that
    # matched the launch parameter.
    section = CANONICAL_SECTION
    assumed = {}
    for candidate in Direction:
        cond = assumed_start_conditions(candidate)
        assumed[candidate] = (cond["position"]["x"], cond["position"]["y"])

    measured = measured_start(rows)
    first_posed = (posed[0][1].pose_x, posed[0][1].pose_y)

    print(f"== {args.bag_dir.name}  settled_direction={direction.value}  posed={len(posed)}")
    print(f"start_section (assumed, canonical) = {section.value}")
    for candidate, xy in assumed.items():
        print(f"assumed start if launched {candidate.value:>16}: ({xy[0]:.4f}, {xy[1]:.4f})")
    print(f"measured start (from bag)          : {measured}")
    print(f"first posed sample                 : ({first_posed[0]:.4f}, {first_posed[1]:.4f})")
    print(f"normal for ({section.value}, {direction.value}) = {TRAVEL_DIRS[(section, direction)]}")

    candidates = [(f"ASSUMED[{d.value}]", xy) for d, xy in assumed.items()]
    if measured is not None:
        candidates.append(("MEASURED (bag)", measured))
    candidates.append(("first posed sample", first_posed))

    out = []
    for label, origin in candidates:
        laps, geo_only = _replay(posed, origin, section, direction)
        if laps:
            blocked = ""
        elif geo_only:
            blocked = f"crossed {geo_only}x in-section, waypoint latch down"
        else:
            blocked = "never crossed the line while inside the start section"
        out.append(
            (
                label,
                f"({origin[0]:.3f}, {origin[1]:.3f})",
                len(laps),
                ", ".join(f"{t:.0f}s" for t in laps[:6]),
                blocked,
            ),
        )
    print()
    print_table(out, ["origin", "position", "laps", "lap_times", "blocked_by"])


if __name__ == "__main__":
    main()
