r"""Would the bay-exit re-anchor have been refused, and did refusing it matter?

The hand-over out of the pocket re-anchors the plan to the pose of that tick
(``adr:0060-bay-exit-clearance-guard``). That fixed a real failure -- the two
rounds whose steer target sat 121-165 degrees BEHIND the chassis both turned
around and died inside 30 s -- but it trusts the pose at the one moment in a
round when trust is least deserved, because the pocket is where the localizer is
worst.

``pose_is_worth_anchoring`` refuses the re-anchor when the fit cost at hand-over
is above ``relocalize_cost_threshold``. This scores that gate on the bags, and it
is the only ruler available: the corpus scenarios START OUTSIDE the bay and their
own path already resyncs, so the simulator cannot fire this branch at all.

WHAT THIS CANNOT DO, stated because the difference decides how much the output is
worth. It is an OBSERVATIONAL counterfactual, not a replay: it reads the fit cost
and the waypoint index that production actually published, and reports whether
the gate would have refused. It cannot say what the round would then have DONE,
because the alternative plan was never driven. So it prices the TRIGGER
(does the gate separate the rounds that went wrong from the ones that did not)
and never the OUTCOME.

Three columns carry the argument:

* ``fit@hand`` -- ``localizer_fit_cost`` on the first tick after the phase leaves
  ``bay_exit``. This is the number the gate reads.
* ``wp@hand`` -- the waypoint index the re-anchored plan picked. A pocket exit
  should land in the low single digits; a much larger index means the plan was
  laid out around a pose somewhere else entirely, which is the damage.
* ``travelled`` -- cumulative angular progress about the mat centre, the measure
  a corner cannot change the sign of (see ``diag_bag_bay_exit_heading.py``).
  Negative against the believed direction means the chassis drove the wrong way
  round.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_reanchor_gate.py \
        data/live/runs --prefix run_
    pixi run -e dev python scripts/bag/diag_bag_reanchor_gate.py \
        data/live/runs/run_XXXXXXXX_XXXXXX [more bags]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.navigation_tuning import LocalizationParams, shipped_group
from shared.domain.enums import Direction

from scripts.common.bag_io import load_nav_debug_rows
from scripts.common.tables import print_table
from src.ros2.navigation.track_navigator_node import pose_is_worth_anchoring

_MAT_CENTRE = (1.5, 1.5)


def _phase(row: object) -> str | None:
    phase = getattr(row, "phase", None)
    return getattr(phase, "value", phase)


def _handover_index(rows: list[tuple[float, object]]) -> int | None:
    """First row after the phase leaves ``bay_exit``, or None if it never does.

    A round that never enters ``bay_exit`` did not start in the pocket and has
    no hand-over to judge; it is skipped rather than counted as a pass, since
    including it would dilute the only population this gate can act on.
    """
    previous = None
    for index, (_, row) in enumerate(rows):
        current = _phase(row)
        if previous == "bay_exit" and current != "bay_exit":
            return index
        previous = current
    return None


def _angular_progress(rows: list[tuple[float, object]]) -> float:
    """Cumulative signed turn about the mat centre, in degrees."""
    total = 0.0
    previous = None
    for _, row in rows:
        if row.pose_x is None or row.pose_y is None:
            continue
        angle = math.atan2(row.pose_y - _MAT_CENTRE[1], row.pose_x - _MAT_CENTRE[0])
        if previous is not None:
            delta = angle - previous
            total += (delta + math.pi) % (2 * math.pi) - math.pi
        previous = angle
    return math.degrees(total)


def _first_fit_cost(rows: list[tuple[float, object]], start: int) -> float | None:
    for _, row in rows[start:]:
        if row.localizer_fit_cost is not None:
            return float(row.localizer_fit_cost)
    return None


def _believed_direction(rows: list[tuple[float, object]]) -> Direction | None:
    for _, row in rows:
        if row.direction is not None:
            return row.direction
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+", type=Path, help="Bag directories, or a parent to scan with --prefix.")
    parser.add_argument("--prefix", default=None, help="Scan each path for child directories with this prefix.")
    args = parser.parse_args()

    bags: list[Path] = []
    for path in args.paths:
        if args.prefix is not None and path.is_dir():
            bags.extend(sorted(p for p in path.iterdir() if p.is_dir() and p.name.startswith(args.prefix)))
        else:
            bags.append(path)

    threshold = shipped_group(LocalizationParams).relocalize_cost_threshold
    _HEADERS = ("run", "dir", "laps", "wp@hand", "fit@hand", "re-anchor", "travelled_deg", "believed_sense")
    rows_out: list[tuple[object, ...]] = []
    refused = 0
    skipped = 0

    for bag in bags:
        try:
            rows, _ = load_nav_debug_rows(bag)
        except Exception as exc:  # a truncated or storage-less bag is data, not a crash
            rows_out.append((bag.name, "-", "-", "-", f"unreadable: {type(exc).__name__}", "-", "-", "-"))
            continue
        index = _handover_index(rows)
        if index is None:
            skipped += 1
            continue
        _, handover = rows[index]
        fit = _first_fit_cost(rows, index)
        worth = pose_is_worth_anchoring(fit, threshold)
        refused += int(not worth)
        direction = _believed_direction(rows)
        progress = _angular_progress(rows)
        # Signed so that POSITIVE always means "the way it believed it was
        # going", whichever way that was. Comparing raw signs across directions
        # is how the earlier reading of these rounds went wrong.
        if direction is Direction.CLOCKWISE:
            progress = -progress
        laps = max((row.laps_completed or 0) for _, row in rows)
        rows_out.append(
            (
                bag.name,
                direction.value if direction is not None else "-",
                laps,
                handover.waypoint_index,
                f"{fit:.4f}" if fit is not None else "None",
                "KEPT" if worth else "REFUSED",
                f"{progress:+.0f}",
                "no" if progress < 0 else "yes",
            )
        )

    print(f"== re-anchor gate at relocalize_cost_threshold = {threshold}")
    print(f"   in-bay rounds judged: {len(rows_out)}   refused: {refused}   no bay_exit hand-over (skipped): {skipped}")
    print()
    print_table(rows_out, _HEADERS)
    print()
    print("   'believed_sense'=no means the chassis made negative angular progress against the")
    print("   direction it believed, i.e. it drove the wrong way round. That is the damage this")
    print("   gate exists to prevent, and the gate is only useful if REFUSED lines up with it.")
    print("   This is a TRIGGER measurement: the refused rounds' alternative plan was never")
    print("   driven, so nothing here says what they would have done instead.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
