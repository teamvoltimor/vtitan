"""Trace what moves the waypoint index just before a bogus lap credit.

``diag_open_laps.py`` shows that a blind fixture can credit its first lap after
under two metres. That happens right after the direction-inference creep
settles, which is also where ``replace_path`` runs. This logs every index jump
alongside the lap credits so the cause is observed rather than inferred: a jump
to the tail of the waypoint list means the robot was snapped *behind* the path
start, which walks straight off the end into a waypoint wrap.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/sim/diag_open_lap_seam.py go_open_0013
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.navigation.core_navigator import CoreNavigator
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from shared.domain.models import Waypoint

    from src.navigation.race_tracker import LapDetector

_STEP_WIDTH = 5
_POSITION_FORMAT = ".2f"
_DEFAULT_FIXTURES = ("go_open_0010", "go_open_0013")


class _IndexTracer:
    """Records index jumps and lap credits for a single fixture run."""

    def __init__(self) -> None:
        self.events: list[str] = []
        self.step = 0
        self._laps = 0

    def patch(self) -> None:
        """Wrap the navigator methods that can move the waypoint index."""
        self._real_path = CoreNavigator.replace_path
        self._real_detector = CoreNavigator.replace_lap_detector
        tracer = self

        def replace_path(
            self: CoreNavigator,
            waypoints: list[Waypoint],
            robot_xy: Waypoint,
        ) -> None:
            before, before_len = self._waypoint_index, len(self._waypoints)
            tracer._real_path(self, waypoints, robot_xy)
            tracer.events.append(
                f"  step{tracer.step:>{_STEP_WIDTH}} replace_path idx {before}/{before_len} "
                f"-> {self._waypoint_index}/{len(waypoints)} "
                f"at ({robot_xy.x:{_POSITION_FORMAT}},{robot_xy.y:{_POSITION_FORMAT}})"
            )

        def replace_lap_detector(self: CoreNavigator, lap_detector: LapDetector) -> None:
            tracer._real_detector(self, lap_detector)
            tracer.events.append(f"  step{tracer.step:>{_STEP_WIDTH}} replace_lap_detector (direction flipped)")

        CoreNavigator.replace_path = replace_path
        CoreNavigator.replace_lap_detector = replace_lap_detector

    def unpatch(self) -> None:
        """Restore the real navigator methods."""
        CoreNavigator.replace_path = self._real_path
        CoreNavigator.replace_lap_detector = self._real_detector

    def on_step(self, navigator: CoreNavigator) -> None:
        """Advance the step counter and record any lap credited this tick."""
        self.step += 1
        if navigator.laps_completed > self._laps:
            self._laps = navigator.laps_completed
            self.events.append(f"  step{self.step:>{_STEP_WIDTH}} LAP CREDITED -> {self._laps}")


def _report(scenario: Any) -> None:
    """Run one fixture blind and print the run-up to its first lap credit."""
    tracer = _IndexTracer()
    tracer.patch()
    try:
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        navigator = sim.navigator
        result = sim.run(on_step=lambda _state, _scan: tracer.on_step(navigator))
    finally:
        tracer.unpatch()

    print(
        f"\n{scenario.label} laps={result.laps_completed}/{result.target_laps} "
        f"dist={result.distance_m:.2f}m"
    )
    # Only the run-up to the first credit matters; the rest is steady state.
    first = next((i for i, e in enumerate(tracer.events) if "LAP CREDITED" in e), len(tracer.events))
    for event in tracer.events[: first + 1]:
        print(event)


def main() -> None:
    """Trace index jumps and lap credits for the fixtures named on argv."""
    wanted = sys.argv[1:] or list(_DEFAULT_FIXTURES)
    for scenario in all_test_scenarios():
        if any(w in scenario.label for w in wanted):
            _report(scenario)


if __name__ == "__main__":
    main()
