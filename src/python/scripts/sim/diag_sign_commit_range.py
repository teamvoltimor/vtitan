r"""At what RANGE and SPEED does the sim's router commit to a pillar, and does the arc fit?

The hardware bags say the commit step is where a crossing pass is decided:
over the 2026-09-14/15 rounds the router commits at p50 0.537 m (p10 0.328),
and within the passes that must CROSS to the legal side the execution-failure
rate is 61% under 0.40 m, 38% at 0.40-0.55 and 4% past 0.55
(``diag_bag_pass_side_speed.py``). Before the simulator is used to screen any
fix aimed at that step, this asks whether it puts the router in the same
regime at all: a sim that commits a metre out cannot score a lever that only
matters under half a metre.

Per commit (first tick the router names a NEW sign) this records, on the SAME
ruler as the bag instrument:

* ``range``   -- robot centre to the committed sign, true frame;
* ``lateral`` -- how far the robot already sits on the LEGAL side of the sign
  (``committed_pass_side_offset``), negative when it must cross;
* ``speed``   -- the chassis speed that tick (``AckermannState.v``);
* ``margin``  -- ``range^2 / (2 R(v)) - max(0, -lateral)``, with
  ``R = intercept + slope * v`` from the robot profile: the lateral an arc of
  the chassis's own radius can buy over the range left, minus what the cross
  needs. Negative means the pass is geometrically short at commit.

Usage (from ``src/python``, with ``PYTHONPATH=.``)::

    pixi run -e dev python scripts/sim/diag_sign_commit_range.py [--corpus] [--workers 8]
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs

from scripts.common.sign_router_capture import patched_deform_waypoint
from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from scripts.common.stats import percentile
from scripts.common.tables import print_table
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.navigation.planning.sign_router import SignRouter
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

RANGE_BANDS = ((0.0, 0.40), (0.40, 0.55), (0.55, 99.0))
"""The bag instrument's bands, so the two tables read side by side."""


def _turn_radius(v: float) -> float:
    return min(
        RobotSpecs.MIN_TURN_RADIUS_CAP_M,
        RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M + RobotSpecs.MIN_TURN_RADIUS_SLOPE_S * abs(v),
    )


@dataclass(slots=True)
class Commit:
    range_m: float
    lateral_m: float
    speed_mps: float = math.nan

    @property
    def crossing(self) -> bool:
        return self.lateral_m < 0.0

    @property
    def needed_m(self) -> float:
        return max(0.0, -self.lateral_m)

    def margin_at(self, v: float) -> float:
        return self.range_m**2 / (2.0 * _turn_radius(v)) - self.needed_m

    @property
    def margin_m(self) -> float:
        return self.margin_at(self.speed_mps)


@dataclass(slots=True)
class ScenarioCommits:
    label: str
    collided: bool
    laps: int
    commits: list[Commit] = field(default_factory=list)


def _scenarios(corpus: bool):  # noqa: ANN202
    return all_obstacles_demo_scenarios(CORPUS_DIR if corpus else None)


def _analyse(job: tuple[int, bool]) -> ScenarioCommits:
    index, corpus = job
    scenario = _scenarios(corpus)[index]
    out = ScenarioCommits(label=scenario.label, collided=False, laps=0)
    last_key: list[tuple[float, float] | None] = [None]
    pending: list[Commit] = []

    def make_wrapper(original: Callable) -> Callable:
        def wrapper(router: SignRouter, waypoint, robot_pos, robot_yaw, corridor, *a, **kw):  # noqa: ANN001,ANN202
            result = original(router, waypoint, robot_pos, robot_yaw, corridor, *a, **kw)
            anchor = router.committed_sign_position
            key = None if anchor is None else (round(anchor.x, 2), round(anchor.y, 2))
            if key is not None and key != last_key[0]:
                lateral = router.committed_pass_side_offset((robot_pos[0], robot_pos[1]))
                if lateral is not None:
                    c = Commit(
                        range_m=math.hypot(anchor.x - robot_pos[0], anchor.y - robot_pos[1]),
                        lateral_m=lateral,
                    )
                    out.commits.append(c)
                    pending.append(c)
            last_key[0] = key
            return result

        return wrapper

    def on_step(state: AckermannState, _scan: LidarScan) -> None:
        # deform_waypoint ran inside this tick; the state that produced it is
        # the one before the tick advanced, but the speed carried across a
        # single 50 ms tick is the same number for this purpose.
        while pending:
            pending.pop().speed_mps = state.v

    with patched_deform_waypoint(make_wrapper):
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
        result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=on_step)
    out.collided = result.collided
    out.laps = result.laps_completed
    return out


def _band(x: float) -> int:
    for i, (lo, hi) in enumerate(RANGE_BANDS):
        if lo <= x < hi:
            return i
    return len(RANGE_BANDS) - 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--corpus", action="store_true", help=f"run the generated corpus under {CORPUS_DIR}")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--only", type=str, default=None, help="substring filter on scenario label")
    args = parser.parse_args()

    scenarios = _scenarios(args.corpus)
    jobs = [(i, args.corpus) for i, s in enumerate(scenarios) if args.only is None or args.only in s.label]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_analyse, jobs))

    commits = [c for r in results for c in r.commits if not math.isnan(c.speed_mps)]
    print(
        f"== {len(results)} scenarios, {len(commits)} commits"
        f" ({sum(1 for r in results if r.collided)} collided,"
        f" {sum(1 for r in results if r.laps < 3)} short of 3 laps)"
    )
    if not commits:
        return 0

    rng = sorted(c.range_m for c in commits)
    spd = sorted(c.speed_mps for c in commits)
    print(
        f"   commit range  p10/p50/p90 = {percentile(rng, 0.1):.3f} / {percentile(rng, 0.5):.3f} / {percentile(rng, 0.9):.3f} m"
        f"   (bags 09-14/15: 0.328 / 0.537 / 0.870)"
    )
    print(
        f"   commit speed  p10/p50/p90 = {percentile(spd, 0.1):.3f} / {percentile(spd, 0.5):.3f} / {percentile(spd, 0.9):.3f} m/s"
        f"   (bags: 81% at 0.220)"
    )
    cross = [c for c in commits if c.crossing]
    print(
        f"   must CROSS at commit: {len(cross)}/{len(commits)} ({100 * len(cross) / len(commits):.1f}%)"
        f"   (bags: 78/163 = 47.9%)"
    )
    print()

    print("== crossing commits by range band (bags: exec-fail 61% / 38% / 4%)")
    rows = []
    for i, (lo, hi) in enumerate(RANGE_BANDS):
        band = [c for c in cross if _band(c.range_m) == i]
        neg = sum(1 for c in band if c.margin_m < 0)
        rows.append(
            [
                f"{lo:.2f}-{min(hi, 9.0):.2f} m",
                str(len(band)),
                str(neg),
                f"{100 * neg / len(band):.0f}%" if band else "--",
            ]
        )
    print_table(rows, ["range", "n", "margin<0", "share"])
    print()

    print("== margin s^2/(2R(v)) - cross needed, crossing commits, at the commit speed and at floors")
    rows = []
    for v in (None, 0.22, 0.152, 0.12, 0.10, 0.08):
        ms = sorted(c.margin_m if v is None else c.margin_at(v) for c in cross)
        if not ms:
            break
        neg = sum(1 for m in ms if m < 0)
        rows.append(
            [
                "as driven" if v is None else f"{v:.3f}",
                "--" if v is None else f"{_turn_radius(v):.3f}",
                f"{percentile(ms, 0.1):+.3f}",
                f"{percentile(ms, 0.5):+.3f}",
                str(neg),
                f"{100 * neg / len(ms):.0f}%",
            ]
        )
    print_table(rows, ["speed", "R m", "margin p10", "margin p50", "neg n", "neg share"])
    print()

    print("== per scenario")
    rows = []
    for r in sorted(results, key=lambda r: r.label):
        cs = [c for c in r.commits if not math.isnan(c.speed_mps)]
        xs = [c for c in cs if c.crossing]
        rr = sorted(c.range_m for c in cs)
        rows.append(
            [
                r.label,
                "HIT" if r.collided else "",
                str(r.laps),
                str(len(cs)),
                f"{percentile(rr, 0.5):.3f}" if rr else "--",
                str(len(xs)),
                str(sum(1 for c in xs if c.margin_m < 0)),
            ]
        )
    print_table(rows, ["scenario", "", "laps", "commits", "range p50", "cross", "cross margin<0"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
