"""Show the scans that vote a wrong travel direction into place.

``diag_open_direction.py`` establishes that three fixtures settle on the wrong
direction. ``infer_direction`` decides on the left/right span, so this records
each accepted vote with the pose and the two side ranges behind it -- enough to
see whether the wrong votes come from one place on the track or are scattered.

Usage (from ``platform/robot``, with PYTHONPATH=".;../shared/src")::

    python scripts/diag_open_direction_votes.py [label ...]
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import math

from src.navigation import direction_estimator as de
from src.simulation.scenario_catalog import all_test_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator


def main() -> None:
    """Print every accepted direction vote for the fixtures named on argv."""
    wanted = sys.argv[1:] or ["go_open_0010", "go_open_0012", "go_open_0013"]

    for scenario in all_test_scenarios():
        if not any(w in scenario.label for w in wanted):
            continue

        votes: list[str] = []
        sim = ScenarioSimulator(
            scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True
        )
        pose_ref = {"p": None}
        real_infer = de.infer_direction

        def traced_infer(ranges_m, angles_rad, yaw, _real=real_infer):
            result = _real(ranges_m, angles_rad, yaw)
            if result is not None:
                left = de._nearest_ray(ranges_m, angles_rad, math.pi / 2)
                right = de._nearest_ray(ranges_m, angles_rad, -math.pi / 2)
                p = pose_ref["p"]
                votes.append(
                    f"  vote={result.value:<17} pos=({p[0]:.2f},{p[1]:.2f}) yaw={yaw:+.2f} "
                    f"left={left:.2f} right={right:.2f} span={left + right:.2f}"
                )
            return result

        de.infer_direction = traced_infer
        # DirectionEstimator resolved the symbol at import time.
        de.DirectionEstimator.observe.__globals__["infer_direction"] = traced_infer
        try:
            sim.run(max_steps=600, on_step=lambda state, _s: pose_ref.__setitem__("p", (state.x, state.y)))
        finally:
            de.infer_direction = real_infer
            de.DirectionEstimator.observe.__globals__["infer_direction"] = real_infer

        truth = scenario.metadata["starting_conditions"]["direction"]
        print(f"\n{scenario.label} truth={truth} votes_needed=5")
        for line in votes[:12]:
            print(line)


if __name__ == "__main__":
    main()
