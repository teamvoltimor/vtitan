"""Can the corridor width be recovered from LIDAR alone, with no map?

Feasibility probe for blind navigation. The WRO rules fix each corridor to one
of two widths (0.6 m or 1.0 m), so the robot does not need to *measure* the
width so much as *classify* it — and the two classes are 0.4 m apart against a
0.03 m LIDAR sigma.

The measurement needs no map and no position estimate: the LIDAR sits at the
chassis centre, so the range directly left plus the range directly right spans
wall-to-wall through the robot. That is the corridor width, wherever in the
corridor the robot happens to be.

It is only valid while the robot is *beside* the inner block and roughly
aligned with the corridor. At a corner the inward ray misses the block and runs
off down the next corridor, so this reports how often the reading is usable as
well as how accurate it is when it is.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_width_probe.py
"""

from __future__ import annotations

import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.constants import CorridorDimensions, TrackDimensions
from shared.config.enums import Section

from src.navigation.planning.sign_router import corridor_for_position
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.scenario_catalog import all_test_scenarios

_NARROW = CorridorDimensions.NARROW
_WIDE = CorridorDimensions.WIDE
_DECISION_BOUNDARY = (_NARROW + _WIDE) / 2.0
"""0.8 m — halfway between the only two legal widths."""

_CORNER_MISS_MARGIN_M = 0.25
_MAX_PLAUSIBLE_WIDTH = _WIDE + _CORNER_MISS_MARGIN_M
"""Beyond this the inward ray has missed the inner block (robot is at a corner)."""

_ALIGNMENT_TOLERANCE_RAD = math.radians(25.0)
"""How far off the corridor axis the chassis may be for the side rays to still
span the corridor rather than a diagonal."""

_WIDTH_VALIDITY_MARGIN_M = 0.25
_MAX_STEPS = 4000


def _sample(scan_ranges: list[float], scan_angles: list[float], target_rad: float) -> float:
    """Range at the bearing nearest ``target_rad`` in the robot frame."""
    best_i = min(range(len(scan_angles)), key=lambda i: abs(_wrap(scan_angles[i] - target_rad)))
    return scan_ranges[best_i]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def measure_width(scan_ranges: list[float], scan_angles: list[float], yaw: float) -> float | None:
    """Corridor width from the two perpendicular rays, or ``None`` if unusable.

    Requires the chassis to be roughly parallel to a track axis (otherwise the
    side rays cut a diagonal, which overestimates) and the total to be
    physically plausible (otherwise the inward ray has escaped past the inner
    block at a corner).
    """
    # Nearest track axis; the corridor always runs along one of them.
    axis_error = _wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2))
    if abs(axis_error) > _ALIGNMENT_TOLERANCE_RAD:
        return None

    left = _sample(scan_ranges, scan_angles, math.pi / 2)
    right = _sample(scan_ranges, scan_angles, -math.pi / 2)
    # Project back onto the corridor normal: a small heading error stretches
    # both rays by 1/cos(error).
    width = (left + right) * math.cos(axis_error)
    if not (_NARROW - _WIDTH_VALIDITY_MARGIN_M < width < _MAX_PLAUSIBLE_WIDTH):
        return None
    return width


def main() -> None:
    """Report classification accuracy over every Open Challenge fixture."""
    correct = 0
    wrong = 0
    usable_ticks = 0
    total_ticks = 0
    errors: list[float] = []
    per_section_wrong: dict[str, int] = defaultdict(int)

    for scenario in all_test_scenarios():
        widths = {
            Section.from_string(s): scenario.metadata["corridor_widths"][s]["width_mm"] / 1000.0
            for s in ("north", "south", "east", "west")
        }
        sim = ScenarioSimulator(scenario.metadata, num_laps=1, seed=scenario.seed)

        def on_step(state, scan, widths=widths) -> None:  # noqa: ANN001
            nonlocal correct, wrong, usable_ticks, total_ticks
            total_ticks += 1
            if scan is None:
                return
            measured = measure_width(list(scan.ranges_m), list(scan.angles_rad), state.yaw)
            if measured is None:
                return
            # Skip corners: only score where the robot is squarely alongside
            # one corridor, which is where a real estimator would trust it.
            corridor = corridor_for_position(state.x, state.y)
            depth = state.x if corridor in (Section.SOUTH, Section.NORTH) else state.y
            if not (TrackDimensions.CORNER_MIN < depth < TrackDimensions.CORNER_MAX):
                return
            usable_ticks += 1
            truth = widths[corridor]
            errors.append(measured - truth)
            guess = _NARROW if measured < _DECISION_BOUNDARY else _WIDE
            if math.isclose(guess, truth):
                correct += 1
            else:
                wrong += 1
                per_section_wrong[corridor.value] += 1

        sim.run(max_steps=_MAX_STEPS, on_step=on_step)

    total = correct + wrong
    mean_err = sum(errors) / len(errors) if errors else float("nan")
    worst = max((abs(e) for e in errors), default=float("nan"))
    print(f"usable ticks           : {usable_ticks}/{total_ticks} ({usable_ticks / total_ticks:.1%})")
    print(f"width classification   : {correct}/{total} correct ({correct / total:.3%})")
    print(f"raw measurement error  : mean {mean_err * 100:+.2f}cm  worst {worst * 100:.2f}cm")
    print(f"decision margin needed : {(_WIDE - _NARROW) / 2 * 100:.0f}cm from the {_DECISION_BOUNDARY:.2f}m boundary")
    if per_section_wrong:
        print(f"misclassified by section: {dict(per_section_wrong)}")


if __name__ == "__main__":
    main()
