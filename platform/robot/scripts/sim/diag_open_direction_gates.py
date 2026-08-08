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

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_direction_gates.py go_open_0000 [--steps 760]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.navigation_tuning import NavigationTuning

from src.navigation import direction_estimator as de
from src.navigation.utils import _nearest_ray, _wrap
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.enums import Direction

_DEFAULT_STEPS = 760


class _GateTracer:
    """Records every scan offered to ``infer_direction`` and its verdict."""

    def __init__(self) -> None:
        self.rows: list[tuple[tuple[float, float], float, float, float, str]] = []
        self.pos: tuple[float, float] = (0.0, 0.0)
        # The thresholds themselves, not copies: this tracer exists to say which
        # gate refused a reading, so it reads the same tuning infer_direction does.
        self.tuning = NavigationTuning.load_default()

    def patch(self) -> None:
        """Wrap ``infer_direction`` where the estimator resolves it."""
        self._real = de.infer_direction
        tracer = self

        def traced(
            ranges_m: Sequence[float],
            angles_rad: Sequence[float],
            yaw: float,
        ) -> Direction | None:
            result = tracer._real(ranges_m, angles_rad, yaw)
            axis_error = abs(_wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
            left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
            right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
            tracer.rows.append(
                (tracer.pos, axis_error, left, right, tracer._verdict(axis_error, left, right, result)),
            )
            return result

        de.infer_direction = traced
        # DirectionEstimator.observe resolved the name at import time.
        de.DirectionEstimator.observe.__globals__["infer_direction"] = traced

    def unpatch(self) -> None:
        """Restore the real ``infer_direction``."""
        de.infer_direction = self._real
        de.DirectionEstimator.observe.__globals__["infer_direction"] = self._real

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


def _report(scenario: Any, steps: int, show_span_fails: bool) -> None:
    """Run one fixture blind and print each reading's gate verdict."""
    tracer = _GateTracer()
    tracer.patch()
    try:
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        sim.run(max_steps=steps, on_step=lambda st, _s: setattr(tracer, "pos", (st.x, st.y)))
    finally:
        tracer.unpatch()

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


def main() -> None:
    """Print gate verdicts for the fixtures named on argv."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("labels", nargs="*", default=["go_open_0000"])
    parser.add_argument("--steps", type=int, default=_DEFAULT_STEPS)
    parser.add_argument("--show-span-fails", action="store_true")
    args = parser.parse_args()

    wanted = args.labels or ["go_open_0000"]
    for scenario in all_test_scenarios():
        if any(w in scenario.label for w in wanted):
            _report(scenario, args.steps, args.show_span_fails)


if __name__ == "__main__":
    main()
