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

Runs in the competition configuration (blind) by default, since that is the one
that counts.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/diag_failure_split.py --corpus
    python scripts/diag_failure_split.py --corpus --sighted
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.constants import RobotSpecs, TrafficSignSpecs

import src.navigation.planning.sign_router as sign_router_module
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

MAX_STEPS = 6000

CORPUS_DIR = Path(__file__).resolve().parents[1] / ".corpus" / "obstacles" / "scenarios"

_CHASSIS_HALF_DIAGONAL = math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)
_PASS_CLEARANCE = _CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2
"""Centre-to-centre separation a mid-turn pass needs.

The yawed figure deliberately, not the aligned one: a target closer than this
cannot be followed safely at an arbitrary heading, which is the condition
``A-clamped`` is testing for.
"""


def _classify(index: int, fixtures: Path | None, blind: bool) -> tuple[str, str]:
    """Run one scenario and name the failure mode at the tick it ended."""
    scenario = all_obstacles_demo_scenarios(fixtures)[index]

    original_deform = sign_router_module.SignRouter.deform_waypoint
    last: dict[str, Any] = {}

    def capturing(router: Any, waypoint: Any, robot_pos: Any, *args: Any, **kwargs: Any) -> Any:
        result = original_deform(router, waypoint, robot_pos, *args, **kwargs)
        last["raw"] = waypoint
        last["deformed"] = result
        last["committed"] = router._committed  # noqa: SLF001 - a probe, by design
        last["signs"] = router._signs  # noqa: SLF001
        last["corridors"] = router._sign_corridors  # noqa: SLF001
        last["direction"] = router._direction  # noqa: SLF001
        last["pos"] = robot_pos
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
        return ("no collision", scenario.label)
    return (_label(last), scenario.label)


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


def _job(args: tuple[int, str | None, bool]) -> tuple[str, str]:
    index, fixtures, blind = args
    return _classify(index, Path(fixtures) if fixtures else None, blind)


def main() -> None:
    """Classify every failing scenario and report the split."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", action="store_true", help="use the pinned-seed 256 corpus")
    parser.add_argument("--sighted", action="store_true", help="run sighted instead of the blind competition config")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    fixtures = CORPUS_DIR if args.corpus else None
    blind = not args.sighted
    count = len(all_obstacles_demo_scenarios(fixtures))
    jobs = [(i, str(fixtures) if fixtures else None, blind) for i in range(count)]

    tally: Counter = Counter()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for kind, _label in pool.map(_job, jobs):
            tally[kind] += 1

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


if __name__ == "__main__":
    main()
