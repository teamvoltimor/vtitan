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

Usage (from ``src``, with PYTHONPATH=.)::

    python scripts/sim/diag_sign_pass.py
"""

from __future__ import annotations

import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import Section
from shared.domain.models import Waypoint

from scripts.common.sign_router_capture import patched_deform_waypoint
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from scripts.common.tables import print_table
from src.navigation.planning.sign_router import SignRouter, corridor_for_position, signs_from_metadata
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

_CHASSIS_HALF_WIDTH = 0.10
CHASSIS_HALF_DIAGONAL = math.hypot(0.15, 0.10)
_SIGN_HALF = 0.025


@dataclass(frozen=True, slots=True)
class PassAnalysisResult:
    """Per-scenario analysis of sign passes."""

    label: str
    collided: bool
    laps_completed: int
    records: list[PassRecord]


@dataclass(slots=True)
class PassRecord:
    """Closest approach to one sign during one run."""

    sign_index: int
    color: str
    corridor: Section
    depth: float
    sign_xy: Waypoint
    best_dist: float = math.inf
    lateral: float = math.nan
    longitudinal: float = math.nan
    yaw_err: float = math.nan
    commanded_lat: float = math.nan


def _along_x(corridor: Section) -> bool:
    """SOUTH/NORTH corridors run along x; EAST/WEST run along y."""
    return corridor in (Section.SOUTH, Section.NORTH)


def _analyse(index: int) -> PassAnalysisResult:
    scenario = all_obstacles_demo_scenarios()[index]
    signs = signs_from_metadata(scenario.metadata)
    corridors = [corridor_for_position(s.x, s.y) for s in signs]

    records = [
        PassRecord(
            sign_index=i,
            color=s.color,
            corridor=corridors[i],
            depth=s.x if _along_x(corridors[i]) else s.y,
            sign_xy=Waypoint(s.x, s.y),
        )
        for i, s in enumerate(signs)
    ]

    latest_target: list[Waypoint | None] = [None]

    def make_capturing_deform(original_deform: Callable) -> Callable:
        def capturing_deform(
            router: SignRouter,
            waypoint: Waypoint,
            robot_pos: Waypoint,
            robot_yaw: float,
            corridor: Section,
            *args: object,
            **kwargs: object,
        ) -> tuple[float, float]:
            """Stand-in for ``SignRouter.deform_waypoint`` that records its output.

            Passes trailing arguments straight through rather than pinning
            ``detections`` positionally: that pinned name broke the moment
            ``deform_waypoint`` grew an ``observations`` keyword (see
            ``diag_sign_trace.py``'s identical wrapper for the same reason).
            The real method returns a plain ``(x, y)`` tuple, not a
            ``Waypoint`` -- wrap it before storing, since ``record()`` below
            reads ``.x``/``.y`` off the captured value.
            """
            result = original_deform(router, waypoint, robot_pos, robot_yaw, corridor, *args, **kwargs)
            latest_target[0] = Waypoint(*result)
            return result

        return capturing_deform

    with patched_deform_waypoint(make_capturing_deform):
        sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)

        def record(state: AckermannState, _scan: LidarScan) -> None:
            for rec in records:
                sx, sy = rec.sign_xy.x, rec.sign_xy.y
                dist = math.hypot(sx - state.x, sy - state.y)
                if dist >= rec.best_dist:
                    continue
                rec.best_dist = dist
                if _along_x(rec.corridor):
                    rec.lateral = abs(state.y - sy)
                    rec.longitudinal = abs(state.x - sx)
                    axis = 0.0
                    target_lat = None if latest_target[0] is None else abs(latest_target[0].y - sy)
                else:
                    rec.lateral = abs(state.x - sx)
                    rec.longitudinal = abs(state.y - sy)
                    axis = math.pi / 2
                    target_lat = None if latest_target[0] is None else abs(latest_target[0].x - sx)
                err = abs(math.atan2(math.sin(state.yaw - axis), math.cos(state.yaw - axis)))
                rec.yaw_err = min(err, math.pi - err)
                rec.commanded_lat = math.nan if target_lat is None else target_lat

        result = sim.run(max_steps=OBSTACLES_MAX_STEPS, on_step=record)

    return PassAnalysisResult(
        label=scenario.label,
        collided=result.collided,
        laps_completed=result.laps_completed,
        records=records,
    )


def main() -> None:
    """Run every fixture and print achieved vs commanded clearance per sign."""
    with ProcessPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(_analyse, range(len(all_obstacles_demo_scenarios()))))

    square_need = _CHASSIS_HALF_WIDTH + _SIGN_HALF
    turning_need = CHASSIS_HALF_DIAGONAL + _SIGN_HALF
    print(f"lateral needed: square pass {square_need:.3f} m, mid-turn pass {turning_need:.3f} m\n")

    tight_square = 0
    tight_turning = 0
    total = 0
    table_rows = []
    for result in results:
        print(f"{result.label}  collided={result.collided} laps={result.laps_completed}")
        for r in result.records:
            if not math.isfinite(r.best_dist):
                continue
            total += 1
            if r.lateral < square_need:
                tight_square += 1
            if r.lateral < turning_need:
                tight_turning += 1
            flag = "HIT-SQUARE" if r.lateral < square_need else ("HIT-TURN" if r.lateral < turning_need else "")
            table_rows.append((
                r.sign_index,
                r.color,
                r.depth,
                r.lateral,
                r.longitudinal,
                r.commanded_lat,
                math.degrees(r.yaw_err),
                flag
            ))
    if table_rows:
        print_table(table_rows, ["sign#", "color", "depth", "lat", "lon", "cmd", "yaw_err_deg", "flag"])

    print(f"\npasses measured: {total}")
    print(f"  below square-pass need ({square_need:.3f}): {tight_square}")
    print(f"  below mid-turn need    ({turning_need:.3f}): {tight_turning}")


if __name__ == "__main__":
    main()
