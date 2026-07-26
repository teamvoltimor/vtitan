"""Per-tick trace of one Obstacles Challenge scenario, around a sign pass.

Every aggregate sweep over the 16 fixtures has come out flat, which says the
sign router's deformation is not reaching the trajectory. This dumps the raw
per-tick evidence for a single scenario: what the lookahead search picked, what
the router turned it into, what the controller commanded, and where the chassis
actually went.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_sign_trace.py 4 --around-sign 2
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import src.navigation.planning.sign_router as sign_router_module
from src.navigation.planning.sign_router import SignRouter, signs_from_metadata
from src.simulation.gateway import ScenarioSimulator
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios

if TYPE_CHECKING:
    from shared.config.enums import Section
    from shared.domain.models import Detection

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

MAX_STEPS = 6000


def main() -> None:
    """Trace one scenario and print the ticks near the chosen sign."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=int, help="index into all_obstacles_demo_scenarios()")
    parser.add_argument("--around-sign", type=int, default=None, help="only print ticks near this sign")
    parser.add_argument("--radius", type=float, default=0.9, help="how near, in metres")
    args = parser.parse_args()

    scenario = all_obstacles_demo_scenarios()[args.scenario]
    signs = signs_from_metadata(scenario.metadata)
    print(f"{scenario.label}")
    for i, s in enumerate(signs):
        print(f"  sign#{i} {s.color:<5} at ({s.x:.2f}, {s.y:.2f})")

    focus = signs[args.around_sign] if args.around_sign is not None else None

    # Capture the router's input and output for the tick being traced.
    rows: list[str] = []
    last: dict[str, object] = {}
    original_deform = sign_router_module.SignRouter.deform_waypoint

    def capturing_deform(
        router: SignRouter,
        waypoint: tuple[float, float],
        robot_pos: tuple[float, float],
        robot_yaw: float,
        corridor: Section,
        detections: list[Detection] | None = None,
    ) -> tuple[float, float]:
        """Stand-in for ``SignRouter.deform_waypoint`` that records its output."""
        result = original_deform(router, waypoint, robot_pos, robot_yaw, corridor, detections)
        last["raw"] = waypoint
        last["deformed"] = result
        last["corridor"] = corridor
        return result

    sign_router_module.SignRouter.deform_waypoint = capturing_deform
    try:
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
        gw = sim.gateway
        step = [0]

        def record(state: AckermannState, _scan: LidarScan) -> None:
            step[0] += 1
            if focus is not None and math.hypot(focus.x - state.x, focus.y - state.y) > args.radius:
                last.clear()
                return
            raw = last.get("raw")
            deformed = last.get("deformed")
            deformed_by = "" if raw == deformed else "DEFORM"
            cmd = gw.last_command
            dist = "" if focus is None else f" d_sign={math.hypot(focus.x - state.x, focus.y - state.y):.3f}"
            rows.append(
                f"t={step[0]:>4} pos=({state.x:.3f},{state.y:.3f}) yaw={math.degrees(state.yaw):7.1f} "
                f"raw={_fmt(raw)} def={_fmt(deformed)} {deformed_by:<6} "
                f"steer={cmd.steering_norm:+.3f} v={cmd.speed_mps:.3f}{dist}"
            )

        result = sim.run(max_steps=MAX_STEPS, on_step=record)
    finally:
        sign_router_module.SignRouter.deform_waypoint = original_deform

    print("\n".join(rows[-args_limit(rows) :]))
    print(
        f"\ncollided={result.collided} laps={result.laps_completed} "
        f"steps={result.steps} final={tuple(round(v, 3) for v in result.final_pose)}"
    )


def args_limit(rows: list[str]) -> int:
    """Cap how many trailing trace rows get printed."""
    return min(len(rows), 200)


def _fmt(pt: object) -> str:
    if pt is None:
        return "(     -,      -)"
    x, y = pt  # type: ignore[misc]
    return f"({x:.3f},{y:.3f})"


if __name__ == "__main__":
    main()
