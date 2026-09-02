"""Is the FIRST turn harder than the ones after it, on the blind Open Challenge?

Every other ``diag_open_*`` sweep scores a run as a whole -- verdict, laps, sim
time -- so a run that fought its way through one corner and cruised the other
eleven is indistinguishable from one that was uniformly mediocre. This localises
the difficulty to a turn *ordinal*, which is the only way to ask whether the
opening corner is special or whether it just happens to be the first one you
watch.

Segmentation is by yaw, not by position, so it needs no track geometry and works
identically for either direction. Unwrapped yaw is turned into signed progress
``s`` (positive in the run's own travel direction) and then made monotone with a
running max, so an escape oscillation cannot un-complete a turn it already made.
``leg k`` is every tick between completing turn ``k-1`` and completing turn
``k``: the straight approach plus the corner arc. Legs tile the run with no gaps,
so a struggle cannot fall between two windows and go unattributed.

Note what a leg is NOT: it is not the corner arc alone. A long leg 1 could be a
long straight -- the start cell sits somewhere along its side, so leg 1's
straight is a random fraction of a full side while later legs are always a whole
one. That makes raw leg *duration* the wrong headline; read the reverse and
stall tick counts, which no amount of straight-line driving inflates.

Usage (from ``platform/robot``, with PYTHONPATH=".")::

    python scripts/sim/diag_open_first_turn.py [--sample 64] [--seed 0] [--all]

Leg 1 is the only leg that bundles two independent handicaps: the blind
direction-inference creep, and an exit corridor the robot has never driven.
``--told-direction`` removes the first and leaves the second, which is the one
run that tells the two apart. It is a CONTROL, not a competition condition --
the round's direction is drawn on the day, so a told run measures a robot with
information no robot has.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

if TYPE_CHECKING:
    from src.simulation.kinematics import AckermannState

from shared.config.constants import CompetitionSpecs, CorridorDimensions, RobotSpecs
from shared.config.hardware_profile import active_profiles

from scripts.common.diag_base import (
    add_sweep_args,
    add_tuning_arg,
    draw_sample,
    load_tuning,
    print_pool_progress,
    resolve_jobs,
    run_pool,
)
from scripts.common.open_cases import SIDES, case_space
from scripts.common.tables import print_table
from scripts.sim.diag_open_exhaustive import _verdict
from src.simulation.scenario_builder import build_open_metadata
from src.simulation.scenario_simulator import CONTROL_DT, ScenarioSimulator

_DEFAULT_SAMPLE_SIZE = 64
_DEFAULT_SEED = 0

_STALL_SPEED_MPS = 0.01
"""Below this the chassis is not making headway, whichever way it points."""

_CRAWL_FRACTION = 0.5
"""Forward speed below this share of the ceiling counts as hesitating, not driving.

A fraction of the ACTIVE profile's ceiling rather than an absolute m/s, so the
same threshold means the same thing under ``fastonly`` as under base -- an
absolute one would silently reclassify every corner as a crawl the moment the
profile raised the top speed."""

_STEER_DEADBAND_RAD = 0.02
"""Steering below this is noise; a sign change across it is not a real flip."""

_QUARTER_TURN_DEG = 90.0

_TURN_START_DEG = 5.0
"""Heading change from the straight that counts as the turn having begun.

Small enough to catch the entry speed before the corner has scrubbed any off,
large enough not to trigger on the centring wander of a straight -- measured
p90 steering flips on a straight leave heading well inside this."""


class _YawTracer:
    """Per-tick pose recorder. Keeps only what leg segmentation needs.

    The creep flag matters because leg 1 alone contains the blind
    direction-inference creep, which is slow *by design*. Counting its ticks
    against leg 1's hesitation would manufacture exactly the asymmetry this
    script exists to test for.
    """

    def __init__(self, simulator: ScenarioSimulator) -> None:
        self._simulator = simulator
        self.yaw_deg: list[float] = []
        self.speed: list[float] = []
        self.steer: list[float] = []
        self.creeping: list[bool] = []

    def on_step(self, state: AckermannState, _scan: object) -> None:
        """Record one tick."""
        estimator = self._simulator.direction_estimator
        self.yaw_deg.append(math.degrees(state.yaw))
        self.speed.append(state.v)
        self.steer.append(state.steer)
        self.creeping.append(estimator is not None and not estimator.is_settled)


def _monotone_progress(yaw_deg: list[float]) -> list[float]:
    """Unwrapped yaw as a monotone, non-negative turn count in degrees.

    Signed by the run's own net rotation so clockwise and counter-clockwise
    scenarios share one scale, then clamped to its running max: the question is
    which turn the robot is *on*, and a corner it fought its way around and
    partly backed out of is still that corner.
    """
    if not yaw_deg:
        return []
    unwrapped = [yaw_deg[0]]
    for previous, current in zip(yaw_deg, yaw_deg[1:], strict=False):
        step = (current - previous + 180.0) % 360.0 - 180.0
        unwrapped.append(unwrapped[-1] + step)
    sign = 1.0 if unwrapped[-1] >= unwrapped[0] else -1.0
    signed = [(value - unwrapped[0]) * sign for value in unwrapped]
    running = []
    peak = 0.0
    for value in signed:
        peak = max(peak, value)
        running.append(peak)
    return running


@dataclass(frozen=True, slots=True)
class LegProfile:
    """One leg's summary: the straight approach plus the corner arc it ends on."""

    leg: int
    corner: str
    ticks: int
    seconds: float
    creep: float
    reverse: int
    stalled: int
    crawling: int
    steer_flips: int
    entry_speed: float | None


def _leg_profile(index: int, ticks: list[tuple[float, float, float, bool]], corner: str) -> LegProfile:
    """Summarise one leg's ticks of ``(progress_deg, speed, steer, creeping)``."""
    driving = [(v, steer) for _, v, steer, creeping in ticks if not creeping]
    flips = sum(
        1
        for (_, before), (_, after) in zip(driving, driving[1:], strict=False)
        if before * after < 0 and min(abs(before), abs(after)) > _STEER_DEADBAND_RAD
    )

    # Speed at the instant the turn begins, i.e. the first tick whose heading
    # has left the straight by TURN_START_DEG. This is the quantity that
    # separates W->N from N->W: the two are the SAME physical corner with the
    # SAME planned arc (verified by diag_open_corner_geometry.py -- clearance
    # identical to three decimals), so nothing about the path explains why one
    # fails 8/36 and the other 0/24. What differs is the state the chassis
    # arrives in, and a wide corridor feeds the speed ladder more forward
    # clearance than a narrow one.
    base = index * _QUARTER_TURN_DEG
    entering = [v for progress, v, _, creeping in ticks if not creeping and progress - base >= _TURN_START_DEG]
    return LegProfile(
        leg=index + 1,
        corner=corner,
        ticks=len(ticks),
        seconds=len(ticks) * CONTROL_DT,
        creep=sum(1 for _, _, _, creeping in ticks if creeping) * CONTROL_DT,
        reverse=sum(1 for v, _ in driving if v < -_STALL_SPEED_MPS),
        stalled=sum(1 for v, _ in driving if abs(v) <= _STALL_SPEED_MPS),
        crawling=sum(1 for v, _ in driving if _STALL_SPEED_MPS < v < _CRAWL_FRACTION * RobotSpecs.MAX_SPEED_MPS),
        steer_flips=flips,
        # None when the run died before this turn ever started -- which is
        # itself the failure being investigated, so it must not read as 0.0.
        entry_speed=entering[0] if entering else None,
    )


def _legs(tracer: _YawTracer, corners: list[str]) -> list[LegProfile]:
    """Split a run into legs, one per turn reached, each ending as that turn completes.

    ``corners`` labels the corner each leg ENDS on, cycling with the lap, so a
    leg's difficulty can be attributed to the width transition it is driving
    into rather than only to its ordinal.
    """
    progress = _monotone_progress(tracer.yaw_deg)
    buckets: defaultdict[int, list[tuple[float, float, float, bool]]] = defaultdict(list)
    for value, tick in zip(progress, zip(tracer.speed, tracer.steer, tracer.creeping, strict=True), strict=True):
        buckets[int(value // _QUARTER_TURN_DEG)].append((value, *tick))
    return [
        _leg_profile(index, ticks, corners[index % len(corners)] if corners else "?")
        for index, ticks in sorted(buckets.items())
    ]


def _corner_sequence(widths_mm: dict[str, int], section: Any, direction: Any) -> list[str]:
    """``W->N`` style label for each corner, in the order this run meets them.

    Index k is the corner that ENDS leg k+1, so it lines up with the turn
    ordinals the rest of this script reports.
    """
    from src.navigation.planning.waypoints import _build_corridor_order, _rotate_to_start

    wide = (CorridorDimensions.NARROW + CorridorDimensions.WIDE) / 2
    order = _rotate_to_start(_build_corridor_order(direction), section)
    labels = []
    for i, entry in enumerate(order):
        exit_ = order[(i + 1) % len(order)]
        pair = (widths_mm[entry.value.lower()] / 1000.0, widths_mm[exit_.value.lower()] / 1000.0)
        labels.append("->".join("W" if w > wide else "N" for w in pair))
    return labels


@dataclass(frozen=True, slots=True)
class _CaseResult:
    """One scenario's outcome, with its full per-leg breakdown."""

    index: int
    verdict: str
    stuck: bool
    sim_time_s: float
    legs: list[LegProfile]
    label: str


def _run_case(payload: tuple[int, tuple[int, ...], str, str, int, int, str | None, bool]) -> _CaseResult:
    """Run one scenario and return its per-leg profile. Primitive-valued so it pickles."""
    from shared.domain.enums import Direction, Section

    index, widths, section_value, direction_value, cell, laps, tuning_path, told = payload
    section = Section(section_value)
    direction = Direction(direction_value)
    widths_mm = dict(zip(SIDES, widths, strict=True))
    meta = build_open_metadata(widths_mm, section, direction, scenario_id=index, start_cell=cell)

    sim = ScenarioSimulator(
        meta,
        num_laps=laps,
        tuning=load_tuning(tuning_path),
        seed=index,
        blind=True,
        infer_direction=not told,
    )
    tracer = _YawTracer(sim)
    result = sim.run(on_step=tracer.on_step)

    return _CaseResult(
        index=index,
        verdict=_verdict(result),
        stuck=result.stuck,
        sim_time_s=result.sim_time_s,
        legs=_legs(tracer, _corner_sequence(widths_mm, section, direction)),
        label=f"{'-'.join(str(w) for w in widths)} {section.value}/{direction.value} c{cell}",
    )


def _leg_table(rows: list[_CaseResult]) -> None:
    """Aggregate every run's legs by ordinal and print the per-ordinal profile."""
    by_leg: defaultdict[int, list[LegProfile]] = defaultdict(list)
    for row in rows:
        for leg in row.legs:
            by_leg[leg.leg].append(leg)

    table = []
    for leg_index in sorted(by_leg):
        legs = by_leg[leg_index]
        reversing = [leg for leg in legs if leg.reverse > 0]
        table.append(
            [
                leg_index,
                len(legs),
                f"{sum(leg.seconds for leg in legs) / len(legs):.1f}s",
                f"{sum(leg.creep for leg in legs) / len(legs):.1f}s",
                f"{len(reversing) / len(legs):.1%}",
                f"{sum(leg.reverse for leg in legs) / len(legs):.1f}",
                f"{max((leg.reverse for leg in legs), default=0):.0f}",
                f"{sum(leg.crawling for leg in legs) / len(legs):.1f}",
                f"{sum(leg.steer_flips for leg in legs) / len(legs):.1f}",
            ]
        )
    print_table(
        table,
        ["turn", "runs", "mean time", "of it creep", "any reverse", "mean rev", "max rev", "mean crawl", "mean flips"],
    )


def _corner_table(rows: list[_CaseResult]) -> None:
    """Entry speed and reversing per width transition, split by first corner or later.

    The first corner is separated because it is the only one the robot meets
    without having driven the exit corridor before, and because every failure
    in the corpus lands there -- pooling it with laps 2 and 3 would dilute the
    contrast by roughly twelve to one.
    """
    by_corner: defaultdict[tuple[str, str], list[LegProfile]] = defaultdict(list)
    for row in rows:
        for leg in row.legs:
            when = "first" if leg.leg == 1 else "later"
            by_corner[(leg.corner, when)].append(leg)

    table = []
    for corner in ("W->W", "W->N", "N->W", "N->N"):
        for when in ("first", "later"):
            legs = by_corner.get((corner, when), [])
            if not legs:
                continue
            speeds = [leg.entry_speed for leg in legs if leg.entry_speed is not None]
            never = sum(1 for leg in legs if leg.entry_speed is None)
            reversing = [leg for leg in legs if leg.reverse > 0]
            table.append(
                [
                    corner,
                    when,
                    len(legs),
                    f"{sum(speeds) / len(speeds):.4f}" if speeds else "-",
                    f"{max(speeds):.4f}" if speeds else "-",
                    never,
                    f"{len(reversing) / len(legs):.1%}",
                ]
            )
    print_table(
        table,
        ["corner", "turn", "count", "mean entry m/s", "max entry", "never turned", "any reverse"],
    )


def _worst_leg_table(rows: list[_CaseResult]) -> None:
    """Which turn ordinal owns each run's worst patch of reversing."""
    worst: Counter[int | str] = Counter()
    for row in rows:
        legs = [leg for leg in row.legs if leg.reverse > 0]
        worst[max(legs, key=lambda leg: leg.reverse).leg if legs else "none"] += 1
    total = sum(worst.values())
    print_table(
        [[key, count, f"{count / total:.1%}"] for key, count in sorted(worst.items(), key=lambda kv: str(kv[0]))],
        ["worst turn", "runs", "share"],
    )


def main() -> None:
    """Sweep the scenario space and report where in the lap the reversing happens."""
    parser = argparse.ArgumentParser(description=__doc__)
    add_sweep_args(
        parser,
        default_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        default_sample=_DEFAULT_SAMPLE_SIZE,
        default_seed=_DEFAULT_SEED,
        jobs=True,
    )
    add_tuning_arg(parser)
    parser.add_argument(
        "--told-direction",
        action="store_true",
        help="hand the robot its travel direction instead of inferring it (NOT a competition condition)",
    )
    args = parser.parse_args()

    population = case_space()
    cases = draw_sample(population, sample=args.sample, seed=args.seed, all_=args.all)
    jobs = resolve_jobs(args.jobs)

    profiles = active_profiles()
    print(
        f"profile {','.join(profiles) if profiles else '<base>'}: "
        f"max_speed {RobotSpecs.MAX_SPEED_MPS:.3f} m/s, "
        f"max wheel angle {math.degrees(RobotSpecs.MAX_STEERING_ANGLE):.1f} deg",
        flush=True,
    )
    print(
        f"{len(cases)} of {len(population)} scenarios, seed={args.seed}, {jobs} workers, "
        f"direction {'TOLD (control)' if args.told_direction else 'inferred'}\n",
        flush=True,
    )

    payloads = [
        (i, widths, section.value, direction.value, cell, args.laps, args.tuning, args.told_direction)
        for i, (widths, section, direction, cell) in enumerate(cases)
    ]
    rows = run_pool(_run_case, payloads, jobs, on_result=print_pool_progress("legs"))
    rows.sort(key=lambda row: row.index)

    verdicts = Counter(row.verdict for row in rows)
    print(f"\nverdicts: {dict(verdicts)}, stuck={sum(1 for row in rows if row.stuck)}\n", flush=True)

    print("per turn ordinal, over every run that reached it:", flush=True)
    _leg_table(rows)

    print("\nwhich turn owns each run's worst reversing:", flush=True)
    _worst_leg_table(rows)

    print("\nby corner width transition -- the planned arc is IDENTICAL for W->N and", flush=True)
    print("N->W (same corner, driven both ways), so any gap here is arrival state:", flush=True)
    _corner_table(rows)

    ended_early = [row for row in rows if row.verdict != "ok"]
    if ended_early:
        print(f"\n{len(ended_early)} runs that did not finish, and the turn they died on:", flush=True)
        print_table(
            [
                [
                    row.index,
                    row.label,
                    row.verdict,
                    len(row.legs),
                    f"{row.legs[-1].reverse:.0f}" if row.legs else "-",
                    f"{row.sim_time_s:.1f}s",
                ]
                for row in ended_early
            ],
            ["#", "scenario", "verdict", "turns reached", "rev on last", "time"],
        )


if __name__ == "__main__":
    main()
