"""Trace what moves the waypoint index just before a bogus lap credit.

``diag_open_laps.py`` shows that two blind fixtures credit their first lap
after under two metres. Both do it right after the direction-inference creep
settles, which is also where ``replace_path`` runs. This logs every index
jump alongside the lap credits so the cause is observed rather than inferred:
a jump to the tail of the waypoint list means the robot was snapped *behind*
the path start, which walks straight off the end into a waypoint wrap.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_lap_seam.py go_open_0013
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.navigation.core_navigator import CoreNavigator
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator


def main() -> None:
    """Trace index jumps and lap credits for the fixtures named on argv."""
    wanted = sys.argv[1:] or ["go_open_0010", "go_open_0013"]

    for scenario in all_test_scenarios():
        if not any(w in scenario.label for w in wanted):
            continue

        events: list[str] = []
        step = {"n": 0}

        real_replace_path = CoreNavigator.replace_path
        real_replace_detector = CoreNavigator.replace_lap_detector

        def replace_path(self, waypoints, robot_xy, _real=real_replace_path):
            before = self._waypoint_index
            before_len = len(self._waypoints)
            _real(self, waypoints, robot_xy)
            events.append(
                f"  step{step['n']:>5} replace_path idx {before}/{before_len} "
                f"-> {self._waypoint_index}/{len(waypoints)} at ({robot_xy[0]:.2f},{robot_xy[1]:.2f})"
            )

        def replace_lap_detector(self, lap_detector, _real=real_replace_detector):
            _real(self, lap_detector)
            events.append(f"  step{step['n']:>5} replace_lap_detector (direction flipped)")

        CoreNavigator.replace_path = replace_path
        CoreNavigator.replace_lap_detector = replace_lap_detector
        try:
            sim = ScenarioSimulator(
                scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
            )
            nav = sim._navigator
            prev_laps = {"n": 0}

            def on_step(_state, _scan) -> None:
                step["n"] += 1
                if nav.laps_completed > prev_laps["n"]:
                    prev_laps["n"] = nav.laps_completed
                    events.append(f"  step{step['n']:>5} LAP CREDITED -> {nav.laps_completed}")

            result = sim.run(on_step=on_step)
        finally:
            CoreNavigator.replace_path = real_replace_path
            CoreNavigator.replace_lap_detector = real_replace_detector

        print(f"\n{scenario.label} laps={result.laps_completed}/{result.target_laps} dist={result.distance_m:.2f}m")
        # Only the run-up to the first credit matters; the rest is steady state.
        first_credit = next((i for i, e in enumerate(events) if "LAP CREDITED" in e), len(events))
        for event in events[: first_credit + 1]:
            print(event)


if __name__ == "__main__":
    main()
