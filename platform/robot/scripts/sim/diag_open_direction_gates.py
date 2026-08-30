"""Show which gate refuses each candidate direction reading.

``infer_direction`` refuses a scan for one of three reasons, and they are not
interchangeable: the span test rejects readings that are not a corridor
measurement at all, the alignment test rejects everything taken while the robot
is turning, and the asymmetry test rejects readings whose open side is not
convincingly further than the closed one. Only a vote that clears all three
counts.

Which one is binding decides what is worth tuning, and guessing gets it wrong.
On go_open_0000 the obvious suspect was asymmetry -- the one accepted vote
cleared the old 0.30 threshold by a centimetre. This tracer showed the dominant
refusal was *alignment* (axis error 0.64-0.74 rad, the robot mid-corner), and
that asymmetry only bound inside the short window where the chassis is square
to the corridor. Eight scans in that window all agreed on the direction and
only one cleared 0.30, which is what set the threshold at 0.20.

## Summary mode

``--summary`` answers the corpus-level question instead: for each fixture, did
the direction ever settle, after how many creep ticks, which gate dominated the
refusals, and which ``follow_corridor`` branch was steering while they were
refused. That last column is the point -- during the creep the robot is NOT on
pure pursuit, so a steering explanation for a settling failure has to name the
branch that was actually driving.

``--yaw-gain`` re-runs the same fixtures against a different chassis yaw
authority (see ``RobotSpecs.YAW_GAIN``, calibrated to 0.55 against a bag on
2026-08-29). Pairing ``--summary`` with two gains is the A/B that says whether a
settling failure is caused by the chassis turning more slowly than the creep
controller assumes.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_direction_gates.py go_open_0000 [--steps 760]
    python scripts/sim/diag_open_direction_gates.py --summary
    python scripts/sim/diag_open_direction_gates.py --summary --yaw-gain 1.0
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import dataclasses
import math
import statistics
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning

from src.navigation import corridor_follower, direction_estimator
from src.navigation.corridor_estimator import classify_width
from src.navigation.utils import _forward_clearance, _nearest_ray, _rear_clearance, axis_error_rad
from src.simulation.scenario_catalog import _OPEN_CHALLENGE_SPACE, all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator, simulator as simulator_module

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.enums import Direction

_DEFAULT_STEPS = 760


def _set_yaw_gain(gain: float) -> None:
    """Re-point the sim's chassis yaw authority at ``gain`` for this process.

    ``_KinematicsConstants`` reads ``RobotSpecs.YAW_GAIN`` once, when the module
    -level default context is built at import, so patching the spec alone would
    be read too late and silently do nothing. Rebuilding the context is what
    makes the override take.
    """
    from src.simulation import kinematics

    RobotSpecs.YAW_GAIN = gain  # type: ignore[misc]
    kinematics._DEFAULT_KINEMATICS_CONTEXT = kinematics.KinematicsContext()


def _set_follower_steer(centering_deg: float | None, corner_deg: float | None) -> None:
    """Re-point the creep's two steering constants for this process.

    Patches ``load_default`` rather than threading a tuning object through,
    because the creep path resolves its tuning through ``get_tuning(None)`` in
    several places and an override that reaches only some of them would compare
    two different controllers.
    """
    updates = {}
    if centering_deg is not None:
        updates["MAX_CENTERING_STEER_DEG"] = centering_deg
    if corner_deg is not None:
        updates["MAX_CORNER_STEER_DEG"] = corner_deg
    if not updates:
        return
    base = NavigationTuning.load_default()
    patched = dataclasses.replace(base, corridor_follower=base.corridor_follower.model_copy(update=updates))
    NavigationTuning.load_default = classmethod(lambda _cls, *a, **k: patched)  # type: ignore[method-assign]


class _GateTracer:
    """Records every scan offered to ``infer_direction`` and its verdict."""

    def __init__(self) -> None:
        self.rows: list[tuple[tuple[float, float], float, float, float, str]] = []
        self.pos: tuple[float, float] = (0.0, 0.0)
        # Creep-phase accounting, for --summary.
        self.branches: collections.Counter[str] = collections.Counter()
        self.creep_ticks = 0
        self.settled_at: int | None = None
        # The thresholds themselves, not copies: this tracer exists to say which
        # gate refused a reading, so it reads the same tuning infer_direction does.
        self.tuning = NavigationTuning.load_default()

    def patch(self) -> None:
        """Wrap ``infer_direction`` where the estimator resolves it."""
        self._real = direction_estimator.infer_direction
        self._real_observe = direction_estimator.DirectionEstimator.observe
        self._real_follow = corridor_follower.follow_corridor
        tracer = self

        def traced(
            ranges_m: Sequence[float],
            angles_rad: Sequence[float],
            yaw: float,
            tuning: Any = None,
        ) -> Direction | None:
            result = tracer._real(ranges_m, angles_rad, yaw, tuning)
            axis_error = axis_error_rad(yaw)
            left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
            right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
            tracer.rows.append(
                (tracer.pos, axis_error, left, right, tracer._verdict(axis_error, left, right, result)),
            )
            return result

        def traced_observe(estimator_self: Any, *args: Any, **kwargs: Any) -> bool:
            settled = tracer._real_observe(estimator_self, *args, **kwargs)
            if settled and tracer.settled_at is None:
                tracer.settled_at = tracer.creep_ticks
            return settled

        def traced_follow(*args: Any, **kwargs: Any) -> Any:
            tracer.creep_ticks += 1
            tracer.branches[tracer._branch(*args, **kwargs)] += 1
            return tracer._real_follow(*args, **kwargs)

        direction_estimator.infer_direction = traced
        # DirectionEstimator.observe resolved the name at import time.
        direction_estimator.DirectionEstimator.observe.__globals__["infer_direction"] = traced
        direction_estimator.DirectionEstimator.observe = traced_observe
        # The simulator imported the name, so rebinding the definition is not enough.
        simulator_module.follow_corridor = traced_follow

    def unpatch(self) -> None:
        """Restore the real ``infer_direction``."""
        direction_estimator.DirectionEstimator.observe = self._real_observe
        direction_estimator.infer_direction = self._real
        self._real_observe.__globals__["infer_direction"] = self._real
        simulator_module.follow_corridor = self._real_follow

    def _branch(
        self,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        speed_mps: float,
        yaw: float | None = None,
        tuning: Any = None,
        forced_turn_side: Any = None,
        believed_width_m: float | None = None,
    ) -> str:
        """Which ``follow_corridor`` exit this call takes.

        Mirrors that function's guards rather than reading a tag off its return
        value, because two branches can emit the same command. Kept in step with
        it by reading the same tuning fields and calling its own ``_way_through``.
        """
        follower = self.tuning.corridor_follower
        turn_clearance = follower.TURN_CLEARANCE_M
        if believed_width_m is not None and classify_width(believed_width_m) == CorridorDimensions.NARROW:
            turn_clearance = follower.NARROW_TURN_CLEARANCE_M

        forward = _forward_clearance(ranges_m, angles_rad, self.tuning)
        left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
        right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)

        if forward < follower.MIN_FORWARD_CLEARANCE_M:
            rear = _rear_clearance(ranges_m, angles_rad, self.tuning)
            if rear is not None and rear > follower.MIN_REVERSE_CLEARANCE_M:
                return "reverse"
            return "pivot" if max(left, right) > turn_clearance else "hold"
        if forward < turn_clearance and not corridor_follower._way_through(ranges_m, angles_rad, self.tuning):
            return "corner"
        if left > CorridorDimensions.WIDE + follower.CORNER_LEAK_MARGIN_M or right > (
            CorridorDimensions.WIDE + follower.CORNER_LEAK_MARGIN_M
        ):
            return "hold-line"
        return "centring"

    def _verdict(self, axis_error: float, left: float, right: float, result: Direction | None) -> str:
        """Name the first gate that refuses this reading, in the order it applies."""
        estimator = self.tuning.direction_estimator
        if left > estimator.MAX_IN_TRACK_RANGE_M or right > estimator.MAX_IN_TRACK_RANGE_M:
            return "dropout"
        if axis_error > estimator.ALIGNMENT_TOLERANCE_RAD:
            return "align-fail"
        if left + right <= estimator.PLAUSIBLE_SPAN_THRESHOLD_M:
            return "span-fail"
        if abs(left - right) < estimator.MIN_ASYMMETRY_M:
            return "ASYM-FAIL"
        return f"VOTE {result.value if result else '-'}"


def _trace(scenario: Any, steps: int) -> tuple[_GateTracer, Any]:
    """Run one fixture blind with the gates traced; return the tracer and result."""
    tracer = _GateTracer()
    tracer.patch()
    try:
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True)
        result = sim.run(max_steps=steps, on_step=lambda st, _s: setattr(tracer, "pos", (st.x, st.y)))
    finally:
        tracer.unpatch()
    return tracer, result


def _report(scenario: Any, steps: int, show_span_fails: bool) -> None:
    """Run one fixture blind and print each reading's gate verdict."""
    tracer, _ = _trace(scenario, steps)

    print(f"\n{scenario.label}")
    print(f"{'pos':>14} {'axErr':>6} {'left':>6} {'right':>6} {'span':>6} {'asym':>6}  verdict")
    for pos, axis_error, left, right, verdict in tracer.rows:
        # Most ticks are mid-corridor with a wall both sides, which says
        # nothing; they drown the readings that were actually candidates.
        if verdict == "span-fail" and not show_span_fails:
            continue
        print(
            f"({pos[0]:5.2f},{pos[1]:5.2f}) {axis_error:6.3f} {left:6.2f} {right:6.2f} "
            f"{left + right:6.2f} {abs(left - right):6.3f}  {verdict}"
        )


def _corpus(name: str) -> list[Any]:
    """The fixture set to run.

    ``committed`` is the 28-fixture unit-test battery. ``open128`` is the
    generated space at ``start_cell == 0`` -- 2 directions x 16 width sets x 4
    sections -- which is the corpus the Open Challenge pass rates in this
    project's history were measured on, and the reason a lateral start is never
    varied there. ``open640`` adds every legal starting cell.
    """
    if name == "committed":
        return all_test_scenarios()
    params = _OPEN_CHALLENGE_SPACE.all_params()
    if name == "open128":
        params = tuple(p for p in params if p.start_cell == 0)
    return [p.to_named_scenario() for p in params]


_WORKER_STEPS = 0


def _worker_init(
    steps: int, yaw_gain: float | None, steer_deg: float | None, corner_deg: float | None = None
) -> None:
    """Apply this arm's overrides inside a fresh worker process.

    The overrides mutate module state, so they have to be re-applied per worker
    rather than inherited -- on Windows the pool spawns rather than forks, and
    a worker that missed them would silently run the shipped configuration and
    contaminate the arm.
    """
    global _WORKER_STEPS
    _WORKER_STEPS = steps
    if yaw_gain is not None:
        _set_yaw_gain(yaw_gain)
    _set_follower_steer(steer_deg, corner_deg)


def _worker_run(scenario: Any) -> tuple[str, str, int | None, int, float, str, str]:
    """Trace one fixture and return only its summary row."""
    tracer, result = _trace(scenario, _WORKER_STEPS)
    refusals = collections.Counter(v for _, _, _, _, v in tracer.rows if not v.startswith("VOTE"))
    top = refusals.most_common(1)
    axis_errors = [ae for _, ae, _, _, _ in tracer.rows]
    return (
        scenario.label,
        _outcome(result),
        tracer.settled_at,
        tracer.creep_ticks,
        statistics.median(axis_errors) if axis_errors else float("nan"),
        top[0][0] if top else "-",
        " ".join(f"{name}:{n}" for name, n in tracer.branches.most_common()),
    )


def _outcome(result: Any) -> str:
    """One word for how the run ended."""
    if result.collided:
        return "collision"
    if result.pass_side_violation:
        return "pass-side"
    if result.stuck:
        return "stuck"
    if result.timed_out:
        return "over-time"
    if result.laps_completed >= result.target_laps:
        return "ok"
    return "incomplete"


def _summary(
    scenarios: list[Any],
    steps: int,
    jobs: int,
    yaw_gain: float | None,
    steer_deg: float | None,
    corner_deg: float | None,
) -> None:
    """One row per fixture: did direction settle, and what was steering if not."""
    follower = NavigationTuning.load_default().corridor_follower
    gain = yaw_gain if yaw_gain is not None else RobotSpecs.YAW_GAIN
    steer = steer_deg if steer_deg is not None else follower.MAX_CENTERING_STEER_DEG
    corner = corner_deg if corner_deg is not None else follower.MAX_CORNER_STEER_DEG
    print(
        f"\nyaw_gain = {gain:.2f}  centering_steer = {steer:.2f} deg  corner_steer = {corner:.2f} deg   "
        f"({len(scenarios)} fixtures, max_steps={steps}, jobs={jobs})"
    )
    header = (
        f"{'fixture':<24} {'outcome':<10} {'settled':>8} {'creep':>6} "
        f"{'axErr p50':>9} {'top refusal':<12} {'creep branches':<40}"
    )
    print(header)
    print("-" * len(header))

    if jobs > 1:
        with concurrent.futures.ProcessPoolExecutor(
            max_workers=jobs, initializer=_worker_init, initargs=(steps, yaw_gain, steer_deg, corner_deg)
        ) as pool:
            results = list(pool.map(_worker_run, scenarios))
    else:
        _worker_init(steps, None, None, None)  # overrides already applied in-process
        results = [_worker_run(s) for s in scenarios]

    rows: list[tuple[str, str, int | None]] = []
    for label, outcome, settled_at, creep, p50, top, branches in results:
        settled = "-" if settled_at is None else str(settled_at)
        print(
            f"{label:<24} {outcome:<10} {settled:>8} {creep:>6} "
            f"{p50:>9.3f} {top:<12} {branches:<40}"
        )
        rows.append((label, outcome, settled_at))

    never = [r for r in rows if r[2] is None]
    print(f"\nsettled {len(rows) - len(never)}/{len(rows)}   never-settled {len(never)}")
    by_outcome: collections.Counter[str] = collections.Counter()
    for _, outcome, settled in rows:
        by_outcome[f"{outcome}/{'settled' if settled is not None else 'NEVER'}"] += 1
    for key, count in sorted(by_outcome.items()):
        print(f"  {key:<24} {count}")


def main() -> None:
    """Print gate verdicts for the fixtures named on argv."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("labels", nargs="*", default=[])
    parser.add_argument("--steps", type=int, default=_DEFAULT_STEPS)
    parser.add_argument("--show-span-fails", action="store_true")
    parser.add_argument("--summary", action="store_true", help="one row per fixture instead of per scan")
    parser.add_argument(
        "--yaw-gain",
        type=float,
        default=None,
        help="override the chassis yaw authority (shipped: RobotSpecs.YAW_GAIN)",
    )
    parser.add_argument(
        "--centering-steer-deg",
        type=float,
        default=None,
        help="override corridor_follower.MAX_CENTERING_STEER_DEG (the heading-damping clamp)",
    )
    parser.add_argument(
        "--corner-steer-deg",
        type=float,
        default=None,
        help="override corridor_follower.MAX_CORNER_STEER_DEG (the corner/back-off turn angle)",
    )
    parser.add_argument(
        "--corpus",
        choices=("committed", "open128", "open640"),
        default="committed",
        help="committed = 28-fixture unit battery; open128 = generated space at start_cell 0",
    )
    parser.add_argument("--jobs", type=int, default=1, help="run fixtures across N processes (--summary only)")
    args = parser.parse_args()

    if args.yaw_gain is not None:
        _set_yaw_gain(args.yaw_gain)
    _set_follower_steer(args.centering_steer_deg, args.corner_steer_deg)

    scenarios = _corpus(args.corpus)
    if args.labels:
        scenarios = [s for s in scenarios if any(w in s.label for w in args.labels)]
    elif not args.summary:
        scenarios = scenarios[:1]

    if args.summary:
        _summary(scenarios, args.steps, args.jobs, args.yaw_gain, args.centering_steer_deg, args.corner_steer_deg)
        return
    for scenario in scenarios:
        _report(scenario, args.steps, args.show_span_fails)


if __name__ == "__main__":
    main()
