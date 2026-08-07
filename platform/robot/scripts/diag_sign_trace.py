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
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.navigation_tuning import NavigationTuning

import src.navigation.planning.sign_router as sign_router_module
from src.navigation.planning.sign_router import SignRouter, signs_from_metadata
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from shared.config.enums import Section
    from shared.domain.models import Waypoint

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

MAX_STEPS = 6000

CORPUS_DIR = Path(__file__).resolve().parents[1] / ".corpus" / "obstacles" / "scenarios"
"""Pinned-seed sweep corpus — see ``diag_sign_sweep.SweepConfig.scenarios_dir``."""

_DEFAULT_RADIUS_M = 0.9
_MAX_TRACE_ROWS = 200


def main() -> None:
    """Trace one scenario and print the ticks near the chosen sign."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=int, help="index into all_obstacles_demo_scenarios()")
    parser.add_argument("--around-sign", type=int, default=None, help="only print ticks near this sign")
    parser.add_argument("--radius", type=float, default=_DEFAULT_RADIUS_M, help="how near, in metres")
    parser.add_argument(
        "--activation",
        type=float,
        default=None,
        help="override ACTIVATION_DIST_M (default 0.80); 1.00-1.20 is the measured plateau, 1.30 the cliff",
    )
    parser.add_argument("--corpus", action="store_true", help="trace a corpus scenario instead of the committed 16")
    parser.add_argument(
        "--buffer",
        type=float,
        default=None,
        help="override sign_router._DEFORM_DEPTH_BUFFER (default 0.30); patched on the module that resolves it",
    )
    args = parser.parse_args()

    fixtures = CORPUS_DIR if args.corpus else None
    scenario = all_obstacles_demo_scenarios(fixtures)[args.scenario]
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
        waypoint: Waypoint,
        robot_pos: Waypoint,
        robot_yaw: float,
        corridor: Section,
        *args: object,
        **kwargs: object,
    ) -> Waypoint:
        """Stand-in for ``SignRouter.deform_waypoint`` that records its output.

        The trailing arguments are passed straight through rather than named:
        this wrapper pinned ``detections`` positionally and broke the moment
        ``deform_waypoint`` grew an ``observations`` keyword, which is how a
        diagnostic goes stale without anything failing until you need it.
        """
        result = original_deform(router, waypoint, robot_pos, robot_yaw, corridor, *args, **kwargs)
        last["raw"] = waypoint
        last["deformed"] = result
        last["corridor"] = corridor
        last["committed"] = router._committed  # noqa: SLF001 - a probe, by design
        return result

    sign_router_module.SignRouter.deform_waypoint = capturing_deform
    original_buffer = sign_router_module._DEFORM_DEPTH_BUFFER  # noqa: SLF001
    if args.buffer is not None:
        sign_router_module._DEFORM_DEPTH_BUFFER = args.buffer  # noqa: SLF001
    try:
        tuning = None
        if args.activation is not None:
            base = NavigationTuning()
            tuning = replace(
                base,
                sign_router=base.sign_router.model_copy(update={"ACTIVATION_DIST_M": args.activation}),
            )
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, tuning=tuning)
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
                f"sign={last.get('committed')} "
                f"steer={cmd.steering_norm:+.3f} v={cmd.speed_mps:.3f}{dist}"
            )

        result = sim.run(max_steps=MAX_STEPS, on_step=record)
    finally:
        sign_router_module.SignRouter.deform_waypoint = original_deform
        sign_router_module._DEFORM_DEPTH_BUFFER = original_buffer  # noqa: SLF001

    print("\n".join(rows[-args_limit(rows) :]))
    print(
        f"\ncollided={result.collided} laps={result.laps_completed} "
        f"steps={result.steps} final={tuple(round(v, 3) for v in result.final_pose)}"
    )


def args_limit(rows: list[str]) -> int:
    """Cap how many trailing trace rows get printed."""
    return min(len(rows), _MAX_TRACE_ROWS)


def _fmt(pt: object) -> str:
    if pt is None:
        return "(     -,      -)"
    x, y = pt  # type: ignore[misc]
    return f"({x:.3f},{y:.3f})"


if __name__ == "__main__":
    main()
