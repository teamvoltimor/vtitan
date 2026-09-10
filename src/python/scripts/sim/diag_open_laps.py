"""Audit blind-mode lap credits against the distance and the angle actually driven.

``diag_open_fixtures.py`` reports ``laps=3/3`` without showing *when* each lap
was credited. A lap credited a few steps after the previous one is not a lap
the robot drove -- it is a bookkeeping artifact. This prints the step index and
the path distance between consecutive lap credits, so a spurious credit shows
up as a near-zero inter-lap distance.

It also prints ``geom``: laps counted purely from the pose track, as the total
angle swept about the mat centre divided by a full turn. That number is what
the robot *drove*, computed without consulting the waypoint lap gate at all, so
the pair (credited, geom) separates the two ways an Open case can read as
``incomplete``. ``geom~=3, credited<3`` is a gate that failed to fire on laps
that happened; ``geom<3`` is a robot that genuinely never got round.

Usage (from ``src``, with PYTHONPATH=".;shared/src")::

    python scripts/sim/diag_open_laps.py [--blind] [--only go_open_0013]
    python scripts/sim/diag_open_laps.py --open-case 47 --sample 128 --seed 0 [--yaw-gain-compensation 0.55]

``--open-case`` runs a case out of the 640-case Open population instead of a
fixture. The number is the one a sweep printed, which indexes that sweep's
DRAW rather than the population, so the same ``--sample``/``--seed``/``--all``
must be given for it to resolve to the same scenario.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, TrackDimensions

from scripts.common.diag_base import add_sweep_args, add_yaw_gain_compensation_arg, load_tuning, select_cases
from scripts.common.formats import DISTANCE_FORMAT, TIME_FORMAT
from scripts.common.open_cases import SIDES, case_space
from src.navigation.race_tracker import LapDetector
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Iterator

    from shared.config.navigation_tuning import NavigationTuning

_SWEEP_SAMPLE_SIZE = 128
"""Draw size the Open A/B screens on, so a case number quoted from it resolves by default."""

_SWEEP_SEED = 0

_TRACK_CENTER = TrackDimensions.MAX_COORD / 2


def _geometric_laps(track: list[tuple[float, float]]) -> float:
    """Laps the pose track actually drove, as swept angle about the mat centre.

    Independent of the waypoint lap gate by construction: it reads only where
    the robot was, never what the navigator believed about its progress. The
    angle is unwrapped per step, so a robot that reverses gives that back
    rather than accumulating it, and a robot that oscillates in place nets out
    near zero instead of counting laps it did not drive.
    """
    swept = 0.0
    previous: float | None = None
    for x, y in track:
        angle = math.atan2(y - _TRACK_CENTER, x - _TRACK_CENTER)
        if previous is not None:
            step = angle - previous
            # Unwrap: a real step is far smaller than half a turn, so anything
            # larger is the atan2 branch cut, not motion.
            step -= math.tau * round(step / math.tau)
            swept += step
        previous = angle
    return abs(swept) / math.tau



@contextlib.contextmanager
def _counted_lap_gate() -> Iterator[Counter[str]]:
    """Count each half of the lap gate separately for the duration of a run.

    A lap is credited only when the geometric crossing and the waypoint wrap
    BOTH hold (see ``LapDetector``), and the two fail for different reasons:
    a missing crossing means the robot never drove past the finish line, a
    crossing that arrives with no wrap pending means it did drive past and the
    waypoint half withheld the credit. The verdict ``incomplete`` hides which
    one happened, so count them.

    Patched here rather than instrumented in ``LapDetector`` itself: this is a
    diagnostic question, and the shipped detector should not carry counters
    only a diag script reads.
    """
    counts: Counter[str] = Counter()
    original_update = LapDetector.update
    original_notify = LapDetector.notify_waypoint_wrapped

    def update(self: LapDetector, robot_pos: object, current_section: object) -> bool:
        previous_dot = self._prev_dot
        confirmed = original_update(self, robot_pos, current_section)
        # update() stores this tick's dot in _prev_dot on both paths, so the
        # crossing test can be reproduced exactly rather than recomputed from
        # the origin and normal, which would be a second copy of the geometry.
        dot = self._prev_dot
        crossed = (
            previous_dot is not None
            and previous_dot < 0.0
            and dot is not None
            and dot >= 0.0
            and current_section is self._start_section
        )
        if crossed:
            counts["crossing"] += 1
            if not confirmed:
                counts["crossing_unconfirmed"] += 1
        if confirmed:
            counts["confirmed"] += 1
        return confirmed

    def notify(self: LapDetector) -> None:
        counts["wrap"] += 1
        original_notify(self)

    LapDetector.update = update  # type: ignore[method-assign]
    LapDetector.notify_waypoint_wrapped = notify  # type: ignore[method-assign]
    try:
        yield counts
    finally:
        LapDetector.update = original_update  # type: ignore[method-assign]
        LapDetector.notify_waypoint_wrapped = original_notify  # type: ignore[method-assign]


def _report(
    metadata: Any, *, laps: int, seed: int, label: str, blind: bool, tuning: NavigationTuning | None
) -> None:
    """Run one scenario and print its lap credits against distance and geometry."""
    # Sample the path at every tick so inter-lap distance can be integrated
    # between the recorded lap step indices, and the swept angle with it.
    track: list[tuple[float, float]] = []
    sim = ScenarioSimulator(metadata, num_laps=laps, seed=seed, blind=blind, tuning=tuning)
    with _counted_lap_gate() as gate:
        result = sim.run(on_step=lambda state, _scan: track.append((state.x, state.y)))

    cumulative = [0.0]
    for i in range(1, len(track)):
        cumulative.append(
            cumulative[-1] + math.hypot(track[i][0] - track[i - 1][0], track[i][1] - track[i - 1][1])
        )

    splits = []
    prev_step = 0
    for lap_step in result.lap_step_indices:
        # on_step samples lag the loop's step counter by one on creep ticks.
        a = min(prev_step, len(cumulative) - 1)
        b = min(lap_step, len(cumulative) - 1)
        splits.append(f"lap@step{lap_step}(+{cumulative[b] - cumulative[a]:{DISTANCE_FORMAT}}m)")
        prev_step = lap_step

    print(
        f"{'OK  ' if result.success else 'FAIL'} {label} "
        f"laps={result.laps_completed}/{result.target_laps} "
        f"geom={_geometric_laps(track):.2f} "
        f"cross={gate['crossing']} wrap={gate['wrap']} "
        f"cross_no_wrap={gate['crossing_unconfirmed']} "
        f"collided={result.collided} stuck={result.stuck} "
        f"rev_run={result.reverse_run_violation} dist={result.distance_m:{DISTANCE_FORMAT}}m "
        f"t={result.sim_time_s:{TIME_FORMAT}}s{' OVER-TIME' if result.over_time else ''} | "
        + " ".join(splits),
        flush=True,
    )


def main() -> None:
    """Print per-lap step, distance and swept-angle splits for Open scenarios."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blind", action="store_true")
    parser.add_argument("--only", default=None, help="Run a single fixture label.")
    parser.add_argument(
        "--open-case",
        type=int,
        action="append",
        help=(
            "Case number as a sweep printed it, repeatable. That number indexes the DRAW, "
            "not the population, so pass the same --sample/--seed/--all the sweep ran under."
        ),
    )
    add_sweep_args(
        parser,
        default_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        default_sample=_SWEEP_SAMPLE_SIZE,
        default_seed=_SWEEP_SEED,
    )
    add_yaw_gain_compensation_arg(parser)
    args = parser.parse_args()

    tuning = load_tuning(None, args.yaw_gain_compensation)

    if args.open_case:
        # Reproduce the sweep's draw, then keep the requested numbers out of it.
        # A sampled sweep labels each case by its position in the draw and seeds
        # the simulator by that position, so indexing the population directly
        # would run a different scenario than the one reported under that number.
        wanted = set(args.open_case)
        cases = select_cases(case_space(), sample=args.sample, seed=args.seed, all_=args.all)
        for index, (widths, section, direction, cell) in cases:
            if index not in wanted:
                continue
            widths_mm = dict(zip(SIDES, widths, strict=True))
            metadata = build_open_metadata(
                widths_mm, section, direction, scenario_id=index, start_cell=cell
            )
            label = f"[{index:>3}] {'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}"
            _report(metadata, laps=args.laps, seed=index, label=label, blind=True, tuning=tuning)
        return

    for scenario in all_test_scenarios():
        if args.only and args.only not in scenario.label:
            continue
        _report(
            scenario.metadata,
            laps=scenario.laps,
            seed=scenario.seed,
            label=scenario.label,
            blind=args.blind,
            tuning=tuning,
        )


if __name__ == "__main__":
    main()
