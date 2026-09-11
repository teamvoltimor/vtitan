"""Probe: what actually stops the run once mapped signs are masked from escape.

The mapped/unmapped split withholds a routed sign's LIDAR returns from the
CRITICAL escape trigger, which was measured to be the gate. If the offset knob
is still inert afterwards, something else is holding the run — this reports the
candidates per tick so the answer is measured rather than guessed:

* ``raw`` / ``masked``: risk from the unmasked and masked scans. Ticks where
  they differ are the ones the split actually changed.
* ``k_turn``: CRITICAL escapes begun (the path the split suppresses).
* ``stuck``: escapes begun by ``StuckDetector`` instead — a completely separate
  trigger the split does not touch, and the obvious suspect if a masked robot
  merely creeps into the sign rather than reversing off it.
* ``creep`` / ``slow``: ticks where raw forward clearance or raw risk capped the
  speed. Under ``lidar_blind`` a sign affects neither; under the split it still
  affects both, which is the remaining behavioural difference between them.

``--census`` answers a different question on the same hooks: not "what does the
mask change" but "how often does the escape gate fire at all, and what does it
cost the clock". Added 2026-08-22, when the LIDAR mount fix (``6c727c87``) took
blind corpus collisions 195 -> 57 while leaving in-time flat at 27 -> 26 and
timeouts 19 -> 132 -- the failure moved into escape thrash, and the gate's
threshold is stated in a frame the fix changed under it.

Usage (from ``src``, PYTHONPATH=.)::

    python scripts/sim/diag_escape_mask.py            # all 16 fixtures, summary
    python scripts/sim/diag_escape_mask.py 0 --ticks  # per-tick detail for one
    python scripts/sim/diag_escape_mask.py --census --corpus --workers 8
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from shared.config.constants import CompetitionSpecs, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import RiskLevel

from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from scripts.common.stats import percentile
from scripts.common.tables import print_table
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_EXPECTED_ASSESS_CALLS = 2
"""``CoreNavigator.step`` assesses risk twice per tick with a sign router
present: once on the raw scan (speed) and once on the masked one (escape)."""


def probe(index: int, show_ticks: bool = False) -> Counter:
    """Run one fixture with the navigator instrumented; return event counts."""
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
    nav = sim._navigator  # noqa: SLF001 - a probe, by design
    counts: Counter = Counter()

    original_assess = nav._collision_controller.assess_risk  # noqa: SLF001 - a probe, by design
    original_begin = nav._begin_maneuver  # noqa: SLF001
    pending: list[RiskLevel] = []

    def assess(ranges: Any, angles: Any = None) -> RiskLevel:
        risk = original_assess(ranges, angles)
        pending.append(risk)
        return risk

    def begin(maneuver: Any) -> None:
        counts[f"begin_{maneuver.maneuver_type}"] += 1
        return original_begin(maneuver)

    nav._collision_controller.assess_risk = assess  # type: ignore[method-assign]  # noqa: SLF001
    nav._begin_maneuver = begin  # type: ignore[method-assign]  # noqa: SLF001

    def on_step(state: Any, _scan: Any) -> None:
        # step() calls assess_risk once on the raw scan and, when a sign router
        # is present, once more on the masked scan. Anything else means the
        # call sites changed and this probe is reading the wrong thing.
        if len(pending) == _EXPECTED_ASSESS_CALLS:
            raw, masked = pending
            counts["ticks"] += 1
            if raw != masked:
                counts[f"split_{raw}_to_{masked}"] += 1
            if raw == RiskLevel.CRITICAL:
                counts["raw_critical"] += 1
            if masked == RiskLevel.CRITICAL:
                counts["masked_critical"] += 1
            if show_ticks and raw != masked:
                print(f"  t={counts['ticks']:>4} ({state.x:.2f},{state.y:.2f}) raw={raw} masked={masked}")
        elif pending:
            counts[f"unexpected_assess_calls_{len(pending)}"] += 1
        pending.clear()

    result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=on_step)
    counts["laps"] = result.laps_completed
    counts["collided"] = int(result.collided)
    return counts


@dataclass(frozen=True)
class _Census:
    """One scenario's escape-gate record, reduced to picklable scalars.

    Per-tick arrays stay in the worker: 256 scenarios x up to
    ``OBSTACLES_MAX_STEPS`` ticks is millions of floats, and every question
    below is answerable from counts and a handful of percentiles.
    """

    index: int
    ticks: int
    laps: int
    sim_time_s: float
    timed_out: bool
    collided: bool

    escape_starts: int
    escape_episodes: int
    """Runs of escape starts separated by more than ``_EPISODE_GAP_TICKS``.

    ``escape_starts`` alone cannot tell one long recovery from many separate
    ones, and thrash is precisely the case where the two diverge.
    """

    crit_ticks: int
    """Masked forward min-range below ``contact_dist`` -- the escape gate."""

    obstacle_ticks: int
    """Between ``contact_dist`` and ``slow_dist``: no escape, but the speed
    ladder caps the tick to slow. This is the clock cost of the same shift."""

    would_slow_body_frame: int
    """OBSTACLE ticks that would still be speed-capped in the body-centred frame.

    The counterfactual that matters once ``crit_ticks`` turns out to be zero.
    The escape gate never fires, but the SLOW band does -- 27% of ticks in a
    winning run and 50% in a failing one -- and it is read in the same shifted
    frame: a 0.25 m threshold on readings that start ~12.2 cm further forward
    caps the speed at what was ~0.37 m body-centred. If this is much smaller
    than ``obstacle_ticks``, most of the clock cost is frame, not geometry.
    """

    would_fire_body_frame: int
    """Ticks that would still be CRITICAL if the gate were read in the OLD
    body-centred frame, i.e. ``min_range - LIDAR_MOUNT_X_OFFSET < contact``.

    The fix moved forward readings ~12.2 cm closer without moving the
    threshold. If this is ~0 while ``crit_ticks`` is large, then essentially
    every front escape now firing is one the pre-fix stack could not fire, and
    the ladder is being read in a frame it was never tuned for.
    """

    min_range_p10: float
    min_range_p50: float

    early_starts: int
    early_episodes: int
    early_crit_ticks: int
    early_obstacle_ticks: int
    """The same four counts restricted to the first ``_EARLY_WINDOW_TICKS``.

    The control that decides whether the escape rate CAUSES the timeout or
    merely records it. A run that is stuck escapes repeatedly by definition, so
    a whole-run count cannot separate the two -- every failing run scores high
    whichever is true. Measured over an opening window that every run reaches
    while still driving normally, the counts precede the outcome, so a gap here
    is predictive rather than descriptive. Same circularity, and the same fix,
    as the passed-sign control in the lane investigation.
    """


_EARLY_WINDOW_TICKS = 400
"""Opening window for the causality control (~20 s at 20 Hz).

Long enough to contain the first corner and the first sign encounters, short
enough that even the runs which later stall are still driving normally through
it -- the in-time median run is ~3300 ticks, so this is its opening ~12%.
"""

_EPISODE_GAP_TICKS = 20
"""Ticks of quiet before a further escape start counts as a NEW episode.

Above the 12-frame K-turn cap and the 8+10-frame slalom, so a single maneuver
re-triggering as it completes stays one episode rather than inflating the count.
"""


def census(args: tuple[int, str | None]) -> _Census:
    """Run one scenario and record what the escape gate did, tick by tick."""
    index, scenarios_dir = args
    directory = Path(scenarios_dir) if scenarios_dir else None
    scenario = all_obstacles_demo_scenarios(directory)[index]
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
    nav = sim._navigator  # noqa: SLF001 - a probe, by design
    tuning = NavigationTuning.load_default()
    # RESOLVED for Obstacles, not the base field. OBSTACLES_CONTACT_DIST ships
    # 0.05 over a base of 0.10, and CoreNavigator resolves it, so reading the
    # base recomputes every gate column at TWICE the threshold these runs
    # actually used -- the shadowing failure OBSTACLES_CONTACT_DIST has caused
    # twice before, and it reads as a real result rather than as an error.
    clearance = tuning.clearance.for_obstacles_challenge()
    contact, slow = clearance.CONTACT_DIST, clearance.SLOW_DIST
    offset = RobotSpecs.LIDAR_MOUNT_X_OFFSET

    controller = nav._collision_controller  # noqa: SLF001
    original_assess = controller.assess_risk
    original_begin = nav._begin_maneuver  # noqa: SLF001
    pending: list[float] = []
    starts: list[int] = []
    ticks = 0
    crit = obstacle = body_frame = slow_body_frame = 0
    early_crit = early_obstacle = 0
    mins: list[float] = []

    def assess(ranges: Any, angles: Any = None) -> RiskLevel:
        path = controller._forward_path_ranges(ranges, angles)  # noqa: SLF001
        pending.append(float(np.min(path)) if path.size else float("inf"))
        return original_assess(ranges, angles)

    def begin(maneuver: Any) -> None:
        starts.append(ticks)
        return original_begin(maneuver)

    controller.assess_risk = assess  # type: ignore[method-assign]
    nav._begin_maneuver = begin  # type: ignore[method-assign]  # noqa: SLF001

    def on_step(_state: Any, _scan: Any) -> None:
        nonlocal ticks, crit, obstacle, body_frame, slow_body_frame, early_crit, early_obstacle
        ticks += 1
        # The MASKED scan is the escape gate (`step` assesses raw first, then
        # masked). Reading the raw one here would count threats the navigator
        # has already decided the router owns.
        if len(pending) == _EXPECTED_ASSESS_CALLS:
            masked = pending[-1]
            mins.append(masked)
            if masked < contact:
                crit += 1
            elif masked < slow:
                obstacle += 1
            # The same tick judged in the frame the threshold was tuned in. A
            # forward ray now starts ~12.2 cm further along, so the pre-fix
            # reading of this obstacle was `masked + offset`; the gate fired
            # only if THAT was inside contact range.
            if masked + offset < contact:
                body_frame += 1
            elif masked + offset < slow:
                slow_body_frame += 1
            if ticks <= _EARLY_WINDOW_TICKS:
                if masked < contact:
                    early_crit += 1
                elif masked < slow:
                    early_obstacle += 1
        pending.clear()

    result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=on_step)

    episodes = early_episodes = 0
    previous: int | None = None
    for tick in starts:
        if previous is None or tick - previous > _EPISODE_GAP_TICKS:
            episodes += 1
            if tick <= _EARLY_WINDOW_TICKS:
                early_episodes += 1
        previous = tick

    return _Census(
        index=index,
        ticks=ticks,
        laps=result.laps_completed,
        sim_time_s=result.sim_time_s,
        timed_out=result.timed_out,
        collided=result.collided,
        escape_starts=len(starts),
        escape_episodes=episodes,
        crit_ticks=crit,
        obstacle_ticks=obstacle,
        would_slow_body_frame=slow_body_frame,
        would_fire_body_frame=body_frame,
        min_range_p10=percentile(mins, 0.1) if mins else 0.0,
        min_range_p50=percentile(mins, 0.5) if mins else 0.0,
        early_starts=sum(1 for t in starts if t <= _EARLY_WINDOW_TICKS),
        early_episodes=early_episodes,
        early_crit_ticks=early_crit,
        early_obstacle_ticks=early_obstacle,
    )


def report_census(workers: int, scenarios_dir: str | None) -> None:
    """Census the escape gate across a corpus, split by how the run ended.

    Split by OUTCOME rather than reported as one aggregate, because the
    question is not "how often does the gate fire" but "does firing more
    separate the runs that finish from the runs that time out". A level that is
    equally high in both columns is what the stack always does; only a gap
    between them is a cause. This is the same pass->collision-gap discipline
    that corrected the clamped-shift reading, applied to the clock.
    """
    directory = Path(scenarios_dir) if scenarios_dir else None
    count = len(all_obstacles_demo_scenarios(directory))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(census, [(i, scenarios_dir) for i in range(count)]))

    tuning = NavigationTuning.load_default()
    clearance = tuning.clearance.for_obstacles_challenge()
    print(
        f"GATE contact_dist={clearance.CONTACT_DIST:.2f}m  "
        f"slow_dist={clearance.SLOW_DIST:.2f}m  "
        f"lidar mount offset={RobotSpecs.LIDAR_MOUNT_X_OFFSET:.4f}m  n={len(rows)}",
        flush=True,
    )

    # The SAME definitions `diag_sign_sweep` reports, so the columns here can be
    # laid against that RESULT line. In particular in-time is the competition
    # test (3 laps inside the round limit), NOT merely "did not exhaust the step
    # budget" -- a run can finish its laps and still be far over the clock.
    laps_target, limit = CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS, CompetitionSpecs.ROUND_TIME_LIMIT_S
    finished = [r for r in rows if r.laps >= laps_target]
    groups: tuple[tuple[str, list[_Census]], ...] = (
        ("in-time", [r for r in finished if r.sim_time_s <= limit]),
        ("slow 3-lap", [r for r in finished if r.sim_time_s > limit]),
        ("under 3 laps", [r for r in rows if r.laps < laps_target and not r.collided]),
        # Overlaps the rows above by construction -- a collision is a reason a
        # run ended, not a fourth bucket of runs. Kept as a lens, not a partition.
        ("collided", [r for r in rows if r.collided]),
    )
    for label, group in groups:
        if not group:
            continue
        ticks = sum(r.ticks for r in group) or 1
        print(
            f"CENSUS {label:<11} n={len(group):>4}  "
            f"escape episodes/run {sum(r.escape_episodes for r in group) / len(group):7.1f}  "
            f"starts/run {sum(r.escape_starts for r in group) / len(group):8.1f}  "
            f"CRITICAL ticks {sum(r.crit_ticks for r in group) / ticks * 100:5.1f}%  "
            f"OBSTACLE ticks {sum(r.obstacle_ticks for r in group) / ticks * 100:5.1f}%  "
            f"fwd min p10 {percentile([r.min_range_p10 for r in group], 0.5):5.3f}m  "
            f"p50 {percentile([r.min_range_p50 for r in group], 0.5):5.3f}m  "
            f"sim_time {percentile([r.sim_time_s for r in group], 0.5):6.1f}s",
            flush=True,
        )

    # The causality control, printed as its own block so it is read against the
    # whole-run figures above rather than instead of them. If these columns are
    # flat while the ones above fan out 12x, escape activity is a symptom of a
    # run already going wrong and the trigger rate is the wrong lever.
    print(f"EARLY  first {_EARLY_WINDOW_TICKS} ticks only (~{_EARLY_WINDOW_TICKS / 20:.0f}s)", flush=True)
    for label, group in groups:
        if not group:
            continue
        early_ticks = sum(min(r.ticks, _EARLY_WINDOW_TICKS) for r in group) or 1
        print(
            f"EARLY  {label:<11} n={len(group):>4}  "
            f"episodes/run {sum(r.early_episodes for r in group) / len(group):6.2f}  "
            f"starts/run {sum(r.early_starts for r in group) / len(group):7.2f}  "
            f"CRITICAL ticks {sum(r.early_crit_ticks for r in group) / early_ticks * 100:5.1f}%  "
            f"OBSTACLE ticks {sum(r.early_obstacle_ticks for r in group) / early_ticks * 100:5.1f}%",
            flush=True,
        )

    fired = sum(r.crit_ticks for r in rows)
    body = sum(r.would_fire_body_frame for r in rows)
    slowed = sum(r.obstacle_ticks for r in rows)
    slow_body = sum(r.would_slow_body_frame for r in rows)
    print(
        f"FRAME  SLOW ticks now {slowed}  "
        f"of which would ALSO cap body-centred {slow_body} "
        f"({slow_body / slowed * 100 if slowed else 0.0:.1f}%)  "
        f"-> {slowed - slow_body} are capped only because forward readings moved "
        f"{RobotSpecs.LIDAR_MOUNT_X_OFFSET * 100:.1f}cm closer",
        flush=True,
    )
    print(
        f"FRAME  CRITICAL ticks now {fired}  "
        f"of which would ALSO fire body-centred {body} ({body / fired * 100 if fired else 0.0:.1f}%)  "
        f"-> {fired - body} are reachable only because forward readings moved "
        f"{RobotSpecs.LIDAR_MOUNT_X_OFFSET * 100:.1f}cm closer",
        flush=True,
    )


def main() -> None:
    """Probe one fixture or all of them."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("index", nargs="?", type=int)
    parser.add_argument("--ticks", action="store_true", help="print each tick the split changed")
    parser.add_argument("--census", action="store_true", help="corpus escape-gate census, split by outcome")
    parser.add_argument("--scenarios-dir")
    parser.add_argument("--corpus", action="store_true", help=f"shorthand for --scenarios-dir {CORPUS_DIR}")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    scenarios_dir = args.scenarios_dir or (str(CORPUS_DIR) if args.corpus else None)
    if args.census:
        report_census(args.workers, scenarios_dir)
        return

    indices = [args.index] if args.index is not None else range(len(all_obstacles_demo_scenarios()))
    total: Counter = Counter()
    rows = []
    for i in indices:
        counts = probe(i, show_ticks=args.ticks)
        row = [f"FIXTURE {i:>2}"] + [counts.get(k, 0) for k in sorted(counts.keys())]
        rows.append(row)
        total.update(counts)
        print(f"FIXTURE {i:>2} " + "  ".join(f"{k}={v}" for k, v in sorted(counts.items())), flush=True)
    if len(list(indices)) > 1:
        print("TOTAL " + "  ".join(f"{k}={v}" for k, v in sorted(total.items())))


if __name__ == "__main__":
    main()
