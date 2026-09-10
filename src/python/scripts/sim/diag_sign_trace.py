"""Per-tick trace of one Obstacles Challenge scenario, around a sign pass.

Every aggregate sweep over the 16 fixtures has come out flat, which says the
sign router's deformation is not reaching the trajectory. This dumps the raw
per-tick evidence for a single scenario: what the lookahead search picked, what
the router turned it into, what the controller commanded, and where the chassis
actually went.

Usage (from ``src``, with PYTHONPATH=.)::

    python scripts/sim/diag_sign_trace.py 4 --around-sign 2
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.navigation_tuning import NavigationTuning

from scripts.common.sign_router_capture import patched_deform_waypoint
from scripts.common.sim_defaults import CORPUS_DIR, OBSTACLES_MAX_STEPS
from src.navigation.planning.sign_router import SignRouter, signs_from_metadata
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Callable

    from shared.domain.enums import Section
    from shared.domain.models import Waypoint

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState


_DEFAULT_RADIUS_M = 0.9
_MAX_TRACE_ROWS = 200


def _parse_args() -> argparse.Namespace:
    """CLI for the trace."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=int, help="index into all_obstacles_demo_scenarios()")
    parser.add_argument("--around-sign", type=int, default=None, help="only print ticks near this sign")
    parser.add_argument("--radius", type=float, default=_DEFAULT_RADIUS_M, help="how near, in metres")
    parser.add_argument(
        "--activation",
        type=float,
        default=None,
        help="override SignRouterParams.ACTIVATION_DIST_M (default 1.40)",
    )
    parser.add_argument("--corpus", action="store_true", help="trace a corpus scenario instead of the committed 16")
    parser.add_argument(
        "--sighted",
        action="store_true",
        help="hand the robot the track layout (ScenarioSimulator blind=False) instead of the default blind run",
    )
    parser.add_argument(
        "--scenarios-dir",
        default=None,
        help="explicit fixtures dir (e.g. .corpus/obstacles/subset64), overrides --corpus",
    )
    parser.add_argument(
        "--buffer",
        type=float,
        default=None,
        help="override SignRouterParams.DEFORM_DEPTH_BUFFER_M (default 0.5)",
    )
    parser.add_argument(
        "--pin-heading-guard-deg",
        type=float,
        default=None,
        help="enable PIN_HEADING_GUARD at this threshold (default off)",
    )
    return parser.parse_args()


def _tuning_for(args: argparse.Namespace) -> NavigationTuning | None:
    """Build the overridden tuning, or ``None`` to run the shipped values.

    Both overrides are tuning FIELDS, not module attributes. The depth buffer
    moved into ``SignRouterParams`` when the constants were centralised, and
    this script went on patching the old module global — which silently did
    nothing long before it started raising ``AttributeError``.
    """
    overrides = {}
    if args.activation is not None:
        overrides["ACTIVATION_DIST_M"] = args.activation
    if args.buffer is not None:
        overrides["DEFORM_DEPTH_BUFFER_M"] = args.buffer
    if args.pin_heading_guard_deg is not None:
        overrides["PIN_HEADING_GUARD"] = True
        overrides["PIN_HEADING_GUARD_DEG"] = args.pin_heading_guard_deg
    if not overrides:
        return None
    base = NavigationTuning.load_default()
    return replace(base, sign_router=base.sign_router.model_copy(update=overrides))


def main() -> None:
    """Trace one scenario and print the ticks near the chosen sign."""
    args = _parse_args()

    fixtures = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else None)
    scenario = all_obstacles_demo_scenarios(fixtures)[args.scenario]
    signs = signs_from_metadata(scenario.metadata)
    print(f"{scenario.label}")
    for i, s in enumerate(signs):
        print(f"  sign#{i} {s.color:<5} at ({s.x:.2f}, {s.y:.2f})")

    focus = signs[args.around_sign] if args.around_sign is not None else None

    # Capture the router's input and output for the tick being traced.
    rows: list[str] = []
    last: dict[str, object] = {}

    def make_capturing_deform(original_deform: Callable) -> Callable:
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
            committed = router._committed  # noqa: SLF001 - a probe, by design
            last["committed"] = committed
            # The committed sign's OWN corridor and estimated position, which is what
            # picks the deformation's lateral axis. In blind mode both are re-derived
            # every tick from a discovery estimate that keeps moving, so a sign near a
            # corner boundary can change corridor — and therefore axis — tick to tick.
            if committed is not None:
                last["sign_corridor"] = router._sign_corridors[committed]  # noqa: SLF001
                spec = router._signs[committed]  # noqa: SLF001
                last["sign_pos"] = (spec.x, spec.y)
            else:
                last.pop("sign_corridor", None)
                last.pop("sign_pos", None)
            return result

        return capturing_deform

    with patched_deform_waypoint(make_capturing_deform):
        tuning = _tuning_for(args)
        sim = ScenarioSimulator(
            scenario.metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            tuning=tuning,
            blind=not args.sighted,
        )
        gw = sim.gateway
        step = [0]

        def record(state: AckermannState, _scan: LidarScan) -> None:
            step[0] += 1
            if focus is not None and math.hypot(focus.x - state.x, focus.y - state.y) > args.radius:
                last.clear()
                return
            dist = "" if focus is None else math.hypot(focus.x - state.x, focus.y - state.y)
            rows.append(_trace_row(step[0], state, gw.last_command, last, dist))

        result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=record)

    print("\n".join(rows[-args_limit(rows) :]))
    print(
        f"\ncollided={result.collided} laps={result.laps_completed} "
        f"steps={result.steps} final={tuple(round(v, 3) for v in result.final_pose)}"
    )


def _trace_row(
    step: int,
    state: AckermannState,
    cmd: object,
    last: dict[str, object],
    dist: float | str,
) -> str:
    """Format one tick: what the planner asked for, and what the chassis did.

    ``sign=`` reads ``index@(x,y)/CORRIDOR`` — the committed sign's index, the
    estimate currently held for it, and the corridor that estimate resolves to.
    The corridor is on the row because it, not the sign's identity, selects
    which world axis the deformation treats as lateral: two consecutive rows
    naming the same sign under different corridors are commanding orthogonal
    directions.
    """
    raw, deformed = last.get("raw"), last.get("deformed")
    sign_corridor = last.get("sign_corridor")
    d_sign = "" if isinstance(dist, str) else f" d_sign={dist:.3f}"
    return (
        f"t={step:>4} pos=({state.x:.3f},{state.y:.3f}) yaw={math.degrees(state.yaw):7.1f} "
        f"raw={_fmt(raw)} def={_fmt(deformed)} {'' if raw == deformed else 'DEFORM':<6} "
        f"sign={last.get('committed')}@{_fmt(last.get('sign_pos'))}"
        f"/{getattr(sign_corridor, 'name', '-'):<5} "
        f"steer={cmd.steering_norm:+.3f} v={cmd.speed_mps:.3f}{d_sign}"
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
