"""Measure the lateral clearance actually achieved at each traffic-sign pass.

``SignRouter`` commands a fixed ``lateral_offset`` (0.20 m) between the path and
a sign's centre. Whether the chassis ever gets there is a separate question:
the deformation is applied to a lookahead target 0.24-0.40 m ahead and tracked
by a P-controller on bearing error, which trails a step change in lateral
demand rather than matching it.

For every sign in every fixture this reports, at the tick of closest approach:

* ``lat`` — separation between robot centre and sign centre across the
  corridor. This is the number that has to beat the chassis half-width
  (0.10 m) plus the sign half-width (0.025 m).
* ``cmd`` — the lateral separation the router was asking for at that tick,
  i.e. how much of the commanded 0.20 m offset actually survived the wall
  clamp and the taper.
* ``yaw_err`` — heading relative to the corridor axis. Near zero means the
  chassis is square to the sign and presents its half-width; large means it is
  mid-turn and presents its half-diagonal (0.180 m) instead.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_sign_pass.py
"""

from __future__ import annotations

import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.enums import Section  # noqa: E402

import src.navigation.planning.sign_router as sign_router_module  # noqa: E402
from src.navigation.planning.sign_router import corridor_for_position, signs_from_metadata  # noqa: E402
from src.simulation.gateway import ScenarioSimulator  # noqa: E402
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios  # noqa: E402

MAX_STEPS = 6000

_CHASSIS_HALF_WIDTH = 0.10
_CHASSIS_HALF_DIAGONAL = math.hypot(0.15, 0.10)
_SIGN_HALF = 0.025


@dataclass(slots=True)
class PassRecord:
    """Closest approach to one sign during one run."""

    sign_index: int
    color: str
    corridor: Section
    depth: float
    sign_xy: tuple[float, float]
    best_dist: float = math.inf
    lateral: float = math.nan
    longitudinal: float = math.nan
    yaw_err: float = math.nan
    commanded_lat: float = math.nan


def _along_x(corridor: Section) -> bool:
    """SOUTH/NORTH corridors run along x; EAST/WEST run along y."""
    return corridor in (Section.SOUTH, Section.NORTH)


def _analyse(index: int) -> tuple[str, bool, int, list[PassRecord]]:
    scenario = all_obstacles_demo_scenarios()[index]
    signs = signs_from_metadata(scenario.metadata)
    corridors = [corridor_for_position(s.x, s.y) for s in signs]

    records = [
        PassRecord(
            sign_index=i,
            color=s.color,
            corridor=corridors[i],
            depth=s.x if _along_x(corridors[i]) else s.y,
            sign_xy=(s.x, s.y),
        )
        for i, s in enumerate(signs)
    ]

    # The router deforms whichever lookahead target the navigator picked, so the
    # only way to see what it actually commanded is to watch its return value.
    latest_target: list[tuple[float, float] | None] = [None]
    original_deform = sign_router_module.SignRouter.deform_waypoint

    def capturing_deform(self, waypoint, robot_pos, robot_yaw, corridor, detections=None):
        result = original_deform(self, waypoint, robot_pos, robot_yaw, corridor, detections)
        latest_target[0] = result
        return result

    sign_router_module.SignRouter.deform_waypoint = capturing_deform
    try:
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)

        def record(state, _scan) -> None:
            for rec in records:
                sx, sy = rec.sign_xy
                dist = math.hypot(sx - state.x, sy - state.y)
                if dist >= rec.best_dist:
                    continue
                rec.best_dist = dist
                if _along_x(rec.corridor):
                    rec.lateral = abs(state.y - sy)
                    rec.longitudinal = abs(state.x - sx)
                    axis = 0.0
                    target_lat = None if latest_target[0] is None else abs(latest_target[0][1] - sy)
                else:
                    rec.lateral = abs(state.x - sx)
                    rec.longitudinal = abs(state.y - sy)
                    axis = math.pi / 2
                    target_lat = None if latest_target[0] is None else abs(latest_target[0][0] - sx)
                # Heading relative to the corridor axis, folded into [0, pi/2].
                err = abs(math.atan2(math.sin(state.yaw - axis), math.cos(state.yaw - axis)))
                rec.yaw_err = min(err, math.pi - err)
                rec.commanded_lat = math.nan if target_lat is None else target_lat

        result = sim.run(max_steps=MAX_STEPS, on_step=record)
    finally:
        sign_router_module.SignRouter.deform_waypoint = original_deform

    return scenario.label, result.collided, result.laps_completed, records


def main() -> None:
    with ProcessPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(_analyse, range(len(all_obstacles_demo_scenarios()))))

    square_need = _CHASSIS_HALF_WIDTH + _SIGN_HALF
    turning_need = _CHASSIS_HALF_DIAGONAL + _SIGN_HALF
    print(f"lateral needed: square pass {square_need:.3f} m, mid-turn pass {turning_need:.3f} m\n")

    tight_square = 0
    tight_turning = 0
    total = 0
    for label, collided, laps, records in results:
        print(f"{label}  collided={collided} laps={laps}")
        for r in records:
            if not math.isfinite(r.best_dist):
                continue
            total += 1
            if r.lateral < square_need:
                tight_square += 1
            if r.lateral < turning_need:
                tight_turning += 1
            flag = "HIT-SQUARE" if r.lateral < square_need else ("HIT-TURN" if r.lateral < turning_need else "")
            print(
                f"   sign#{r.sign_index} {r.color:<5} depth={r.depth:.1f} "
                f"lat={r.lateral:.3f} lon={r.longitudinal:.3f} "
                f"cmd={r.commanded_lat:.3f} yaw_err={math.degrees(r.yaw_err):5.1f}deg  {flag}"
            )

    print(f"\npasses measured: {total}")
    print(f"  below square-pass need ({square_need:.3f}): {tight_square}")
    print(f"  below mid-turn need    ({turning_need:.3f}): {tight_turning}")


if __name__ == "__main__":
    main()
