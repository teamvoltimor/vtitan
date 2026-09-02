"""Probe: does the router mishandle TWO signs bracketing a gap?

Observed on the mat: with the robot on the inner side of a corridor and two
obstacles ahead with a gap between them, it steers to centre itself in the gap
rather than continuing straight through a line that already clears both.

``SignRouter.deform_waypoint`` picks exactly ONE sign -- the nearest applicable
active candidate -- and overrides the lateral axis with a value derived from
that sign alone. It has no notion of two signs bounding a gap, and no test of
the form "is the line I am already on clear of every sign?". So with two signs
in play the commanded line is whatever the current nearest-wins winner implies,
and if the winner changes mid-approach the commanded lateral target jumps
between two different values while the chassis is committed.

This reports, for each fixture's fatal approach:

* how many signs were within ``activation_dist`` per tick (the "two signs in
  play" condition),
* which sign index won the nearest-wins race, and how often that winner
  CHANGED,
* the commanded lateral target each tick, so a jump between two values is
  visible directly,
* whether the robot's own lateral position at the start of the approach was
  already clear of every sign in play -- i.e. whether "just go straight" was
  actually available.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/sim/diag_sign_pairs.py            # all 16, summary
    python scripts/sim/diag_sign_pairs.py 4 --ticks  # per-tick detail for one
    python scripts/sim/diag_sign_pairs.py --scenarios-dir <corpus> --quiet

The committed 16 fixtures contain almost none of this configuration, so the
last form -- against a larger generated corpus -- is the one that can actually
say how often it occurs. See ``diag_sign_sweep.SweepConfig.scenarios_dir`` for
how to generate one.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.domain.enums import Section

from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from scripts.sim.diag_sign_sweep import SweepConfig
from src.navigation.planning.sign_router import corridor_for_position, signs_from_metadata
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_APPROACH_TICKS = 60
"""Ticks before the collision to treat as "the fatal approach" (~3 s at 20 Hz)."""

CHASSIS_HALF_DIAGONAL = math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)
_PASS_CLEARANCE = CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre lateral separation a mid-turn pass needs (0.205 m)."""

_PAIR = 2
"""Signs in play that constitutes the "two bracketing a gap" case."""


def _lateral_of(x: float, y: float, corridor: Section) -> float:
    """The coordinate that is 'across' this corridor."""
    return y if corridor in (Section.SOUTH, Section.NORTH) else x


@dataclass(frozen=True, slots=True)
class SignApproachTick:
    """One tick's router state, captured for the fatal-approach window."""

    pos: Any
    """``robot_pos`` as passed to ``_active_sign_candidates`` (pre-``xy``)."""

    corridor: Section
    in_play: list[tuple[int, float]]
    """``(sign index, distance)`` pairs within ``activation_dist`` this tick."""

    winner: int | None
    """The sign index ``deform_waypoint`` actually committed to this tick."""

    xy: tuple[float, float]


@dataclass(frozen=True, slots=True)
class SignPairResult:
    """One fixture's fatal-approach summary."""

    label: str
    collided: bool
    laps: int
    max_in_play: int
    ticks_with_2plus: int
    approach_ticks: int
    winner_switches: int
    straight_was_clear: bool | None


def probe(
    index: int,
    show_ticks: bool = False,
    fixtures_dir: Path | None = None,
    commit_hysteresis: bool | None = None,
) -> SignPairResult | None:
    """Run one fixture, recording the router's choice through the fatal approach.

    Returns ``None`` if the scenario has no sign router to trace (nothing to
    probe).
    """
    scenario = all_obstacles_demo_scenarios(fixtures_dir)[index]
    signs = signs_from_metadata(
        scenario.metadata if isinstance(scenario.metadata, dict) else scenario.metadata.model_dump(),
    )
    # Reuse the sweep's tuning override so both harnesses build the arms the
    # same way -- a second hand-rolled copy is how the last two measurements
    # ended up reporting a config that was never actually applied.
    tuning = SweepConfig("probe", commit_hysteresis=commit_hysteresis).tuning()
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, tuning=tuning)
    router = sim._navigator.sign_router  # noqa: SLF001 - a probe, by design
    if router is None:
        return None

    original_candidates = router._active_sign_candidates  # noqa: SLF001
    ticks: list[SignApproachTick] = []
    pending: dict[str, Any] = {}

    def traced(robot_pos: Any, robot_yaw: Any, corridor: Any) -> Any:
        result = original_candidates(robot_pos, robot_yaw, corridor)
        in_play = [(i, d) for i, d in result if d <= router._config.activation_dist]  # noqa: SLF001
        pending.clear()
        pending.update(
            {
                "pos": robot_pos,
                "corridor": corridor,
                "in_play": in_play,
                "winner": in_play[0][0] if in_play else None,
            },
        )
        return result

    router._active_sign_candidates = traced  # type: ignore[method-assign]  # noqa: SLF001

    def on_step(state: Any, _scan: Any) -> None:
        if pending:
            # ``_committed`` is the sign deform_waypoint ACTUALLY selected this
            # tick, after _prefer_committed has reordered the race and the
            # applicability checks have run. The raw first-place finisher of
            # _active_sign_candidates is not the same thing once hysteresis is
            # in play -- reading it made the hysteresis invisible and reported
            # an unchanged switch count for a fix that was working.
            chosen = router._committed  # noqa: SLF001
            ticks.append(
                SignApproachTick(
                    pos=pending["pos"],
                    corridor=pending["corridor"],
                    in_play=pending["in_play"],
                    winner=chosen,
                    xy=(state.x, state.y),
                ),
            )
            pending.clear()

    result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=on_step)

    approach = [t for t in ticks if t.in_play][-_APPROACH_TICKS:]
    winners = [t.winner for t in approach]
    switches = sum(1 for a, b in itertools.pairwise(winners) if a != b)
    max_in_play = max((len(t.in_play) for t in approach), default=0)
    two_plus = sum(1 for t in approach if len(t.in_play) >= _PAIR)

    # Was a straight line through already clear? Take the robot's lateral
    # position at the START of the approach and ask whether holding it would
    # have cleared every sign that came into play.
    straight_was_clear = None
    if approach:
        first = approach[0]
        corridor = first.corridor
        held = _lateral_of(first.xy[0], first.xy[1], corridor)
        involved = {i for t in approach for i, _ in t.in_play}
        clearances = [
            abs(held - _lateral_of(signs[i].x, signs[i].y, corridor))
            for i in involved
            if corridor_for_position(signs[i].x, signs[i].y) == corridor
        ]
        if clearances:
            straight_was_clear = min(clearances) >= _PASS_CLEARANCE

    if show_ticks:
        for t in approach:
            lat = _lateral_of(t.xy[0], t.xy[1], t.corridor)
            print(
                f"  ({t.xy[0]:.2f},{t.xy[1]:.2f}) lat={lat:.3f} {t.corridor} "
                f"in_play={[(i, round(d, 2)) for i, d in t.in_play]} winner={t.winner}",
            )

    return SignPairResult(
        label=scenario.label,
        collided=result.collided,
        laps=result.laps_completed,
        max_in_play=max_in_play,
        ticks_with_2plus=two_plus,
        approach_ticks=len(approach),
        winner_switches=switches,
        straight_was_clear=straight_was_clear,
    )


def _report_summary(
    verdicts: Counter, total_switches: int, clear_when_collided: list[int], args: argparse.Namespace
) -> None:
    print("\nSUMMARY " + "  ".join(f"{k}={v}" for k, v in verdicts.most_common()))
    # The number the hysteresis A/B turns on: how often the SELECTED sign
    # changed mid-approach across the whole corpus, not just how many runs saw
    # at least one change.
    print(f"TOTAL WINNER SWITCHES (hysteresis={args.hysteresis or 'default'}): {total_switches}")
    not_clear, clear = clear_when_collided
    total = clear + not_clear
    if total:
        # Of the runs that ended in a collision, how many were on a line that
        # already cleared every sign in play? A high number means the router is
        # putting an already-safe robot back into contention rather than
        # rescuing an unsafe one.
        print(f"STRAIGHT-WAS-CLEAR at collision: {clear}/{total} ({100 * clear / total:.0f}%)")


def main() -> None:
    """Probe one fixture or all of them."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("index", nargs="?", type=int)
    parser.add_argument("--ticks", action="store_true", help="print each approach tick")
    parser.add_argument("--scenarios-dir", default=None, help="generated corpus to run instead of the committed 16")
    parser.add_argument("--corpus", action="store_true", help="shorthand for the pinned-seed corpus directory")
    parser.add_argument("--quiet", action="store_true", help="summary only, no per-scenario rows")
    parser.add_argument(
        "--hysteresis",
        choices=("on", "off"),
        default=None,
        help="override COMMIT_HYSTERESIS; omit to use the configured default",
    )
    args = parser.parse_args()

    hysteresis = None if args.hysteresis is None else args.hysteresis == "on"
    fixtures = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else None)
    indices = [args.index] if args.index is not None else range(len(all_obstacles_demo_scenarios(fixtures)))
    verdicts: Counter = Counter()
    clear_when_collided = [0, 0]
    total_switches = 0
    for i in indices:
        r = probe(i, show_ticks=args.ticks, fixtures_dir=fixtures, commit_hysteresis=hysteresis)
        if r is None:
            continue
        if not r.collided:
            verdict = "no collision"
        elif r.winner_switches > 0:
            verdict = "WINNER SWITCHED during approach"
        elif r.max_in_play >= _PAIR:
            verdict = "two in play, one winner throughout"
        else:
            verdict = "single sign in play"
        verdicts[verdict] += 1
        total_switches += r.winner_switches
        if r.collided and r.straight_was_clear is not None:
            clear_when_collided[1 if r.straight_was_clear else 0] += 1
        if args.quiet:
            continue
        print(
            f"{r.label:<34} collided={r.collided} laps={r.laps} "
            f"max_in_play={r.max_in_play} ticks>=2={r.ticks_with_2plus}/{r.approach_ticks} "
            f"switches={r.winner_switches} straight_clear={r.straight_was_clear} -> {verdict}",
            flush=True,
        )
    _report_summary(verdicts, total_switches, clear_when_collided, args)


if __name__ == "__main__":
    main()
