"""Split the surviving sign collisions between the two known failure modes.

Both modes are diagnosed (see docs/sign-avoidance-investigation.md) but their
SHARE is not, and that is what decides where the next fix goes. Fixing the
geometry (Mode A) is worthless if most runs die with no deformation at all.

At the tick the run ends, ask what the router was doing:

* **Mode B -- avoidance was OFF.** ``deform_waypoint`` returned the waypoint
  untouched: either no sign was committed, or the corner guard rejected every
  candidate. Nothing the deformation geometry does can help here.
* **Mode A -- avoidance was ON but insufficient.** A sign was committed and the
  waypoint was deformed, yet the chassis hit anyway. Sub-split by whether the
  commanded lateral line could ever have cleared the sign:
  - ``A-clamped``: the deformed target was itself closer to the sign than a
    yawed chassis needs, so following it perfectly still collides. This is the
    wall-clamp shortfall the yaw-aware clamp would fix.
  - ``A-lag``: the target was far enough out, but the robot had not converged
    onto it yet -- pure pursuit closes cross-track error over distance and the
    chassis drew abreast of the sign first.

``--yaw`` additionally splits the ``A-clamped`` group by the chassis heading at
the fatal tick, which is what decides whether "arrive square" is a real lever
(investigation doc, Next item 2b). The clearance a pass needs scales with the
angle ``th`` off the corridor axis as ``(L/2)|sin th| + (W/2)|cos th| +
sign_half``, so the same commanded line that fails while turning can be ample
square. Each ``A-clamped`` collision is re-asked as: given the clearance the
line actually offered, was it short only because the chassis was yawed
(``recoverable`` -- arriving square fixes it) or short even square
(``hard`` -- no heading fix reaches it)?

Runs in the competition configuration (blind) by default, since that is the one
that counts.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/sim/diag_failure_split.py --corpus
    python scripts/sim/diag_failure_split.py --corpus --sighted
    python scripts/sim/diag_failure_split.py --corpus --yaw
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs, TrafficSignSpecs

import src.navigation.planning.sign_router as sign_router_module
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

MAX_STEPS = 6000

CORPUS_DIR = Path(__file__).resolve().parents[2] / ".corpus" / "obstacles" / "scenarios"

_CHASSIS_HALF_DIAGONAL = math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)
_PASS_CLEARANCE = _CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre separation a mid-turn pass needs.

The yawed figure deliberately, not the aligned one: a target closer than this
cannot be followed safely at an arbitrary heading, which is the condition
``A-clamped`` is testing for.
"""

_SQUARE_CLEARANCE = RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre separation a pass needs with the chassis ON the corridor axis.

The floor of the yaw-dependent requirement: at ``th = 0`` the length term drops
out entirely and only the half-WIDTH presents. Any commanded line at or above
this could have been followed cleanly by a square chassis, so a collision on
such a line is a heading failure, not a clamp failure.
"""

_DEFAULT_WORKERS = 8

_YAW_BUCKETS_DEG = (10.0, 20.0, 28.0, 40.0)
"""Histogram edges. 28 deg is the measured budget at the clamp's 0.181 m."""


@dataclass(frozen=True, slots=True)
class Verdict:
    """One scenario's outcome, plus the geometry behind an ``A-clamped`` label.

    Carries the yaw terms rather than a pre-computed boolean so the "arrive
    square" question can be re-asked at a different threshold without re-running
    256 scenarios -- the same separation of run from verdict ``_label`` exists
    for, after a wrong verdict cost a full sweep once already.
    """

    kind: str
    label: str
    yaw_off_axis_deg: float | None = None
    target_offset_m: float | None = None
    robot_offset_m: float | None = None
    needed_at_yaw_m: float | None = None

    @property
    def clearance_m(self) -> float | None:
        """How far the commanded line sat from the sign, unsigned."""
        return None if self.target_offset_m is None else abs(self.target_offset_m)

    @property
    def line_was_adequate(self) -> bool | None:
        """Whether the commanded line cleared the sign at the yaw actually held.

        ``A-clamped`` is judged against the half-DIAGONAL, i.e. the worst yaw
        the chassis could possibly present. A line short of that can still be
        ample at the heading the robot was really on, and when it is, the
        commanded line did not cause the collision -- the chassis not being on
        it did.
        """
        if self.clearance_m is None or self.needed_at_yaw_m is None:
            return None
        return self.clearance_m >= self.needed_at_yaw_m

    @property
    def tracking_error_m(self) -> float | None:
        """How far the chassis sat off the line it was commanded to hold.

        Differenced SIGNED, both measured from the sign along the same lateral
        axis. Unsigned it would collapse a chassis sitting on the wrong side of
        the sign entirely onto one that merely undershot, and the wrong-side
        case is the larger error of the two.
        """
        if self.target_offset_m is None or self.robot_offset_m is None:
            return None
        return abs(self.target_offset_m - self.robot_offset_m)

    @property
    def recoverable_by_squaring(self) -> bool | None:
        """Whether arriving on the corridor axis would have cleared this sign.

        Only meaningful for a genuinely short line: if the line was already
        adequate at the held yaw, squaring the chassis was never the missing
        ingredient.
        """
        if self.clearance_m is None or self.line_was_adequate:
            return None
        return self.clearance_m >= _SQUARE_CLEARANCE


def _yaw_off_axis(robot_yaw: float, lateral_axis: str) -> float:
    """Angle between the chassis and the corridor axis, folded into [0, 90] deg.

    The lateral axis is the one the deformation moves along, so the corridor
    runs along the OTHER one: a y-lateral corridor (south/north) is travelled
    along x, heading 0. Folded because the clearance requirement is symmetric --
    a chassis 30 deg off the axis presents the same footprint whichever way it
    leans, and whether it is nose-first or reversed does not matter either.
    """
    axis_heading = 0.0 if lateral_axis == "y" else math.pi / 2
    off = abs(math.atan2(math.sin(robot_yaw - axis_heading), math.cos(robot_yaw - axis_heading)))
    return math.degrees(min(off, math.pi - off))


def _needed_clearance(yaw_off_axis_deg: float) -> float:
    """Centre-to-centre separation a chassis at this heading needs to clear a sign."""
    th = math.radians(yaw_off_axis_deg)
    half_footprint = (RobotSpecs.LENGTH / 2) * abs(math.sin(th)) + (RobotSpecs.WIDTH / 2) * abs(math.cos(th))
    return half_footprint + TrafficSignSpecs.WIDTH / 2


def _classify(index: int, fixtures: Path | None, blind: bool) -> Verdict:
    """Run one scenario and name the failure mode at the tick it ended."""
    scenario = all_obstacles_demo_scenarios(fixtures)[index]

    original_deform = sign_router_module.SignRouter.deform_waypoint
    last: dict[str, Any] = {}

    def capturing(router: Any, waypoint: Any, robot_pos: Any, robot_yaw: float, *args: Any, **kwargs: Any) -> Any:
        result = original_deform(router, waypoint, robot_pos, robot_yaw, *args, **kwargs)
        last["raw"] = waypoint
        last["deformed"] = result
        last["committed"] = router._committed  # noqa: SLF001 - a probe, by design
        last["signs"] = router._signs  # noqa: SLF001
        last["corridors"] = router._sign_corridors  # noqa: SLF001
        last["direction"] = router._direction  # noqa: SLF001
        last["pos"] = robot_pos
        last["yaw"] = robot_yaw
        return result

    sign_router_module.SignRouter.deform_waypoint = capturing  # type: ignore[method-assign]
    try:
        sim = ScenarioSimulator(
            scenario.metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            blind=blind,
            park=False,
        )
        result = sim.run(max_steps=MAX_STEPS)
    finally:
        sign_router_module.SignRouter.deform_waypoint = original_deform  # type: ignore[method-assign]

    if not result.collided:
        return Verdict("no collision", scenario.label)
    return _verdict(last, scenario.label)


def _verdict(last: dict[str, Any], label: str) -> Verdict:
    """Name the failure mode, and for Mode A attach the yaw geometry.

    Geometry is attached to the whole of Mode A, not just ``A-clamped``: the two
    sub-labels are decided against the worst-case yaw, so which side of that
    line a run falls on says little about what actually went wrong at the
    heading it really held.
    """
    kind = _label(last)
    if not kind.startswith("A-"):
        return Verdict(kind, label)

    routing = sign_router_module._ROUTING_TABLE.get((last["corridors"][last["committed"]], last["direction"]))  # noqa: SLF001
    if routing is None:
        return Verdict(kind, label)
    axis = 1 if routing[0] == "y" else 0
    sign_lat = (last["signs"][last["committed"]].x, last["signs"][last["committed"]].y)[axis]
    yaw_deg = _yaw_off_axis(last["yaw"], routing[0])
    return Verdict(
        kind,
        label,
        yaw_off_axis_deg=yaw_deg,
        target_offset_m=last["deformed"][axis] - sign_lat,
        robot_offset_m=last["pos"][axis] - sign_lat,
        needed_at_yaw_m=_needed_clearance(yaw_deg),
    )


def _label(last: dict[str, Any]) -> str:
    """Name the failure mode from the router state captured at the fatal tick.

    Split out from ``_classify`` so the run and the verdict stay separable: the
    verdict has been wrong once already (it compared Euclidean gaps, see below)
    and re-deriving it should not mean re-running 256 scenarios.
    """
    if not last:
        return "B-never-ran"

    committed = last.get("committed")
    raw, deformed = last.get("raw"), last.get("deformed")
    if committed is None or raw == deformed:
        # No sign selected, or the corner guard rejected every candidate and the
        # waypoint came back untouched. Avoidance was not running.
        return "B-no-deform"

    # Avoidance WAS running. Could the line it commanded ever have cleared?
    signs = last.get("signs") or []
    if committed >= len(signs):
        return "A-lag"
    sign = signs[committed]

    # Compare LATERAL separations, not Euclidean distances. The deformation only
    # moves the waypoint on the corridor's lateral axis, and the target leads the
    # robot by a lookahead along the DEPTH axis -- so a Euclidean gap is inflated
    # by an along-track term that has nothing to do with clearing the sign. Using
    # it made ``A-clamped`` almost unreachable: a line clamped to 0.181 m of real
    # clearance still measured >0.205 m once the lookahead was folded in, so
    # every run classified as ``A-lag`` and the clamp looked exonerated when it
    # is in fact saturated at half of all legal sign/colour combinations.
    routing = sign_router_module._ROUTING_TABLE.get((last["corridors"][committed], last["direction"]))  # noqa: SLF001
    if routing is None:
        return "A-other"
    axis = 1 if routing[0] == "y" else 0
    sign_lat = (sign.x, sign.y)[axis]
    target_lat = abs(deformed[axis] - sign_lat)
    robot_lat = abs(last["pos"][axis] - sign_lat)
    if target_lat < _PASS_CLEARANCE:
        return "A-clamped"
    return "A-lag" if robot_lat < target_lat else "A-other"


def _job(args: tuple[int, str | None, bool]) -> Verdict:
    index, fixtures, blind = args
    return _classify(index, Path(fixtures) if fixtures else None, blind)


def main() -> None:
    """Classify every failing scenario and report the split."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", action="store_true", help="use the pinned-seed 256 corpus")
    parser.add_argument("--sighted", action="store_true", help="run sighted instead of the blind competition config")
    parser.add_argument("--yaw", action="store_true", help="also break A-clamped down by chassis yaw at the fatal tick")
    parser.add_argument("--workers", type=int, default=_DEFAULT_WORKERS)
    args = parser.parse_args()

    fixtures = CORPUS_DIR if args.corpus else None
    blind = not args.sighted
    count = len(all_obstacles_demo_scenarios(fixtures))
    jobs = [(i, str(fixtures) if fixtures else None, blind) for i in range(count)]

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        verdicts = list(pool.map(_job, jobs))
    tally = Counter(v.kind for v in verdicts)

    mode = "BLIND (competition)" if blind else "sighted"
    print(f"\nFAILURE SPLIT over {count} scenarios -- {mode}")
    collisions = sum(v for k, v in tally.items() if k != "no collision")
    for kind, n in tally.most_common():
        share = "" if kind == "no collision" else f"  ({100 * n / collisions:.0f}% of collisions)"
        print(f"  {kind:<14} {n:>4}{share}")
    a = sum(v for k, v in tally.items() if k.startswith("A-"))
    b = sum(v for k, v in tally.items() if k.startswith("B-"))
    if collisions:
        print(f"\n  Mode A (deformation ran, insufficient): {a}/{collisions} ({100 * a / collisions:.0f}%)")
        print(f"  Mode B (deformation absent):            {b}/{collisions} ({100 * b / collisions:.0f}%)")

    if args.yaw:
        _report_yaw([v for v in verdicts if v.yaw_off_axis_deg is not None])


def _report_yaw(mode_a: list[Verdict]) -> None:
    """Report what the chassis was doing at the fatal tick, and what it implies."""
    print(f"\nYAW AT THE FATAL TICK -- {len(mode_a)} Mode A collisions")
    if not mode_a:
        return

    print(f"  a square pass needs {_SQUARE_CLEARANCE:.3f} m centre-to-centre")
    yaws = sorted(v.yaw_off_axis_deg or 0.0 for v in mode_a)
    print(f"  yaw off corridor axis: median {yaws[len(yaws) // 2]:.1f} deg, max {yaws[-1]:.1f} deg")
    lo = 0.0
    for edge in (*_YAW_BUCKETS_DEG, 90.0):
        n = sum(1 for y in yaws if lo <= y < edge)
        print(f"    {lo:>4.0f}-{edge:<4.0f} deg  {n:>4}  ({100 * n / len(yaws):.0f}%)")
        lo = edge

    # The question item 2b actually turns on: was the commanded line the problem?
    adequate = [v for v in mode_a if v.line_was_adequate]
    short = [v for v in mode_a if not v.line_was_adequate]
    print(f"\n  line ADEQUATE at the held yaw: {len(adequate)}/{len(mode_a)} ({100 * len(adequate) / len(mode_a):.0f}%)")
    print("    the geometry was there and the chassis was not on it -- a tracking failure")
    if adequate:
        errs = sorted(v.tracking_error_m or 0.0 for v in adequate)
        print(f"    median offset from the commanded line: {1000 * errs[len(errs) // 2]:.0f} mm")
        wrong_side = sum(1 for v in adequate if (v.target_offset_m or 0.0) * (v.robot_offset_m or 0.0) < 0)
        print(f"    of which on the WRONG SIDE of the sign entirely: {wrong_side}/{len(adequate)}")
    print(f"  line SHORT at the held yaw:    {len(short)}/{len(mode_a)} ({100 * len(short) / len(mode_a):.0f}%)")
    if short:
        recoverable = [v for v in short if v.recoverable_by_squaring]
        print(f"    of which squaring would fix: {len(recoverable)}/{len(short)}")
        deficits = sorted((v.needed_at_yaw_m or 0.0) - (v.clearance_m or 0.0) for v in short)
        print(f"    median shortfall: {1000 * deficits[len(deficits) // 2]:.0f} mm")


if __name__ == "__main__":
    main()
