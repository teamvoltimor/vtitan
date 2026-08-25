"""Ground-truth audit of the Obstacles pass-side rule.

Scores every sign from the TRUE layout against the TRUE trajectory, sharing
nothing with what the robot believes, and reports it beside the router's own
believed-frame record. The gap between the two is discovery error; the truth
column is what a judge would call.

Also carries the checks that qualify that number:

- **Retreat vs pass.** ``SignRouter`` retires a sign on distance ALONE
  (``d > passed_dist``), unlike the candidate filter directly below it, which
  also requires the sign not to be behind the chassis -- so a robot that backs
  away from a sign should cross the same threshold as one that drove past.
  REFUTED at 0 retreats in 181 retirements (4 in 1275 untruncated); every
  retirement has the sign genuinely behind at median -1.2 m. Kept because it
  is cheap and would catch the mechanism if escape behaviour ever changed.
- **Verdict stability.** Whether a violation survives nudging the sign's
  corridor label -- see ``_verdict_is_ambiguous``.
- **Escape attribution.** Whether the chassis was mid-escape at the closest
  approach, which is the first bucket of the wrong-side investigation.

``--no-terminate`` lets runs continue past a violation, so the rate is measured
over the full three laps rather than on a trajectory the verdict truncated.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_pass_side_retire.py --scenarios-dir <dir> [--limit 64] [--no-terminate]
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import statistics
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from shared.domain.enums import Axis
from src.navigation.planning import sign_router as sign_router_module
from src.navigation.planning.waypoints import corridor_for_position
from src.simulation.scenario_simulator import ScenarioSimulator

# After the simulator: importing shared.domain.models first hits the
# models<->enums cycle that enums.py resolves by deferring its re-export.
from shared.domain.models import ScenarioMetadata  # noqa: E402

_BEHIND_EPS_M = 0.0
"""Along-track sign that separates a pass from a retreat; 0 is the geometric split."""

_PASSED_NEAR_M = 0.60
"""Closest approach within which the run is treated as having passed a sign.

Wide enough to include a deliberately wide berth (the lane's own spec is
+27.86 cm on the free side) and short enough to exclude a sign in the opposite
corridor, which is metres away."""


_VIOLATIONS: list[tuple[str, str, float, bool, str]] = []
"""Per-violation detail from ``_truth_violations``: (colour, corridor, margin
onto the forbidden side in m, verdict flips under a corridor nudge, navigator
phase at closest approach)."""

_CORRIDOR_PROBE_M = 0.10
"""Nudge used to test whether a violation's verdict depends on where the corner
is drawn. See ``_verdict_is_ambiguous`` for why the naive form of this test --
asking whether the LABEL is stable -- measures nothing at all."""


def _verdict_is_ambiguous(sign: object, near_x: float, near_y: float) -> bool:
    """Would nudging the sign's corridor label change the wrong-side VERDICT?

    Label instability on its own means nothing: 94.9% of corpus signs sit
    within 10 cm of some corridor boundary, so "is the label unstable" is not
    a selective question and the first version of this check, which asked
    exactly that, returned 100% for violations against a 94.9% base rate --
    a control that was missing until it was measured.

    What matters is whether the instability reaches the answer. A south<->west
    flip also flips the comparison AXIS (``_ROUTING_TABLE`` gives N/S the Y
    axis and E/W the X axis), so it genuinely can. A violation whose verdict
    survives every neighbouring label is one the scorer can be trusted on
    regardless of where the corner is drawn.
    """
    verdicts = set()
    for dx, dy in ((0.0, 0.0), (_CORRIDOR_PROBE_M, 0.0), (-_CORRIDOR_PROBE_M, 0.0), (0.0, _CORRIDOR_PROBE_M), (0.0, -_CORRIDOR_PROBE_M)):
        rule = sign_router_module.outward_lateral_axis(
            corridor_for_position(sign.x + dx, sign.y + dy),  # type: ignore[attr-defined]
            sign.color,  # type: ignore[attr-defined]
        )
        if rule is None:
            continue
        axis, permitted = rule
        robot_lat = near_x if axis == Axis.X else near_y
        sign_lat = sign.x if axis == Axis.X else sign.y  # type: ignore[attr-defined]
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        verdicts.add(side != 0 and side != permitted)
    return len(verdicts) > 1


def _truth_violations(
    metadata: ScenarioMetadata,
    trail: list[tuple[float, float]],
    phases: list[str],
) -> tuple[int, int]:
    """Score every sign the way a judge would: true layout, true trajectory.

    Deliberately consults NOTHING the robot believes. Matching a retirement to
    a true sign by proximity was tried first and is not sound -- the belief
    frame is offset, so the nearest true sign to the believed pose need not be
    the sign that was retired, and the resulting colour-agreement rate came out
    near chance, which is what a random matching looks like rather than a
    finding.

    Instead each TRUE sign is scored at the robot's closest approach along its
    TRUE path -- the point at which the choice of side is actually made, per
    the standing method lesson that a pass must be measured on the polyline
    and not at an index. Signs the run never reached are not scored.

    Returns ``(passed, wrong_side)`` counts for this scenario.
    """
    signs = sign_router_module.signs_from_metadata(metadata)
    if not signs or not trail:
        return 0, 0
    passed = wrong = 0
    for sign in signs:
        near_i = min(range(len(trail)), key=lambda i: math.hypot(sign.x - trail[i][0], sign.y - trail[i][1]))
        near_x, near_y = trail[near_i]
        if math.hypot(sign.x - near_x, sign.y - near_y) > _PASSED_NEAR_M:
            continue
        rule = sign_router_module.outward_lateral_axis(corridor_for_position(sign.x, sign.y), sign.color)
        if rule is None:
            continue
        axis, permitted = rule
        robot_lat = near_x if axis == Axis.X else near_y
        sign_lat = sign.x if axis == Axis.X else sign.y
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        passed += 1
        if side != 0 and side != permitted:
            wrong += 1
            # Margin: how far onto the forbidden side, and whether the corridor
            # this verdict was read through is stable. A violation that is both
            # marginal AND corridor-ambiguous is one the scorer should not be
            # trusted on.
            _VIOLATIONS.append(
                (
                    str(sign.color),
                    str(corridor_for_position(sign.x, sign.y)),
                    abs(robot_lat - sign_lat),
                    _verdict_is_ambiguous(sign, near_x, near_y),
                    phases[near_i] if near_i < len(phases) else "unknown",
                )
            )
    return passed, wrong


def _instrument(true_pose: list[tuple[float, float]]) -> list[tuple[float, bool, float, float, str]]:
    """Patch ``SignRouter`` so every retirement records its along-track geometry.

    ``true_pose`` is a one-element holder the caller refreshes each tick with
    the simulator's ground-truth position, so each retirement can be scored
    against the layout as well as against the robot's belief.

    Returns the list the patched methods append to: one entry per retirement,
    ``(along_track_m, scored_violation, true_x, true_y, believed_colour)``.
    """
    records: list[tuple[float, bool, float, float, str]] = []
    router_cls = sign_router_module.SignRouter
    original_candidates = router_cls._active_sign_candidates  # noqa: SLF001
    original_record = router_cls._record_pass_side  # noqa: SLF001

    def candidates(self, robot_pos, robot_yaw, corridor):  # type: ignore[no-untyped-def]
        # The yaw the retirement happened under is not passed to
        # _record_pass_side, so latch it on the way past.
        self._diag_yaw = robot_yaw  # noqa: SLF001
        return original_candidates(self, robot_pos, robot_yaw, corridor)

    def record(self, index, robot_pos):  # type: ignore[no-untyped-def]
        before = index in self._wrong_side  # noqa: SLF001
        original_record(self, index, robot_pos)
        after = index in self._wrong_side  # noqa: SLF001
        sign = self._signs[index]  # noqa: SLF001
        yaw = getattr(self, "_diag_yaw", 0.0)
        dx, dy = sign.x - robot_pos.x, sign.y - robot_pos.y
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        tx, ty = true_pose[0]
        records.append((along, after and not before, tx, ty, str(sign.color)))

    router_cls._active_sign_candidates = candidates  # type: ignore[assignment]  # noqa: SLF001
    router_cls._record_pass_side = record  # type: ignore[assignment]  # noqa: SLF001
    return records


def _disable_termination() -> None:
    """Stop the simulator ending a run on the router's pass-side verdict.

    Without this the ground-truth rate is measured on trajectories the verdict
    itself truncated -- the run stops at the first believed violation, so the
    signs counted are only the ones reached before it, and mostly the earliest
    ones. Letting the run continue measures the rate over the whole three laps
    the challenge actually scores.
    """
    ScenarioSimulator._check_pass_side_violation = lambda self, nav: None  # type: ignore[assignment]  # noqa: ARG005, SLF001


def _run_one(args_tuple: tuple[str, bool]) -> tuple[Counter[str], list[float], list[str]]:
    """Run one scenario and cross-tabulate each retirement's two verdicts.

    Returns ``(counts, alongs, colour_disagreements)`` where ``counts`` keys
    are ``"<router>/<truth>"`` over ``ok``/``wrong``, plus ``retreat`` and
    ``pass`` for the along-track split.
    """
    path_str, no_terminate = args_tuple
    logging.disable(logging.CRITICAL)
    if no_terminate:
        _disable_termination()
    true_pose = [(0.0, 0.0)]
    records = _instrument(true_pose)
    _VIOLATIONS.clear()
    trail: list[tuple[float, float]] = []
    phases: list[str] = []
    metadata = ScenarioMetadata.model_validate(json.loads(Path(path_str).read_text()))
    sim = ScenarioSimulator(metadata, num_laps=3, seed=0, blind=True)

    def _on_step(state, _scan) -> None:  # type: ignore[no-untyped-def]
        true_pose[0] = (state.x, state.y)
        trail.append((state.x, state.y))
        phases.append(str(getattr(sim.navigator.debug_snapshot, "phase", "unknown")))

    sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=_on_step)

    counts: Counter[str] = Counter()
    alongs: list[float] = []
    for along, violation, _tx, _ty, _colour in records:
        alongs.append(along)
        counts["retreat" if along > _BEHIND_EPS_M else "pass"] += 1
        counts["router_wrong"] += int(violation)
    truth_passed, truth_wrong = _truth_violations(metadata, trail, phases)
    counts["truth_passed"] += truth_passed
    counts["truth_wrong"] += truth_wrong
    counts["run_ended_by_router"] += int(any(v for _a, v, *_ in records))
    counts["run_truly_violated"] += int(truth_wrong > 0)
    return counts, alongs, list(_VIOLATIONS)


def main() -> None:
    """Aggregate retirement geometry across a scenario directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios-dir", required=True)
    parser.add_argument("--limit", type=int, default=64)
    parser.add_argument("--workers", type=int, default=14)
    parser.add_argument(
        "--no-terminate",
        action="store_true",
        help="let runs continue past a believed pass-side violation (untruncated truth rate)",
    )
    args = parser.parse_args()

    paths = sorted(Path(args.scenarios_dir).glob("*_metadata.json"))[: args.limit]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_run_one, [(str(p), args.no_terminate) for p in paths]))

    counts: Counter[str] = Counter()
    for result in results:
        counts.update(result[0])
    alongs = [a for r in results for a in r[1]]
    violations = [v for r in results for v in r[2]]

    total = counts["retreat"] + counts["pass"]
    print(f"scenarios {len(paths)}  retirements {total}")
    print(f"  along-track: RETREAT (sign ahead) {counts['retreat']:>4}   PASS (sign behind) {counts['pass']:>4}")
    if alongs:
        print(f"    median {statistics.median(alongs):+.2f} m")
    print(f"  ROUTER (believed frame, discovered colour): {counts['router_wrong']:>4} wrong-side of {total} retirements")
    print(f"  TRUTH  (true layout, true trajectory):      {counts['truth_wrong']:>4} wrong-side of {counts['truth_passed']} signs actually passed")
    print("  per-run:")
    print(f"    runs the router ended on pass-side  {counts['run_ended_by_router']:>4}/{len(paths)}")
    print(f"    runs with a REAL wrong-side pass    {counts['run_truly_violated']:>4}/{len(paths)}")
    if violations:
        ambiguous = [v for v in violations if v[3]]
        margins = sorted(v[2] for v in violations)
        print(f"  violation detail ({len(violations)}):")
        print(f"    VERDICT flips under a {_CORRIDOR_PROBE_M:.2f} m corridor nudge  {len(ambiguous)}  ({len(ambiguous) / len(violations):.1%})")
        print(f"    margin onto forbidden side: p10 {margins[len(margins) // 10]:.3f} m  median {statistics.median(margins):.3f} m  p90 {margins[-max(len(margins) // 10, 1)]:.3f} m")
        print(f"    marginal (<0.05 m) {sum(1 for m in margins if m < 0.05)}   by corridor {dict(Counter(v[1] for v in violations))}   by colour {dict(Counter(v[0] for v in violations))}")
        print(f"    phase at closest approach {dict(Counter(v[4] for v in violations).most_common())}")


if __name__ == "__main__":
    main()
