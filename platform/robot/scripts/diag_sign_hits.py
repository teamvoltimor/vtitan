"""Attribute each Obstacles Challenge collision to the specific sign it hit.

The sweep harness (``diag_sign_sweep.py``) counts collisions; this one names
them. For every one of the 16 fixtures it reports the nearest sign to the
collision pose, that sign's colour, its corridor, and its position along and
across that corridor — so claims of the form "every collision is a sign at
grid depth 1.0 or 2.0" can be checked rather than assumed.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_sign_hits.py
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.enums import Section

from src.navigation.planning.sign_router import corridor_for_position, signs_from_metadata
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios

MAX_STEPS = 6000


@dataclass(frozen=True, slots=True)
class HitReport:
    """One scenario's collision, attributed to the nearest sign."""

    label: str
    collided: bool
    laps: int
    steps: int
    sign_index: int
    sign_color: str
    sign_xy: tuple[float, float]
    corridor: str
    depth: float
    """Distance along the sign's own corridor — the axis the WRO 1.0/1.5/2.0
    sign grid is defined on."""
    lateral: float
    """Position across that corridor (the lane axis)."""
    centre_dist: float


def _analyse(index: int) -> HitReport:
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
    result = sim.run(max_steps=MAX_STEPS)

    signs = signs_from_metadata(scenario.metadata)
    x, y, _yaw = result.final_pose
    nearest = min(range(len(signs)), key=lambda i: math.hypot(signs[i].x - x, signs[i].y - y))
    sign = signs[nearest]
    corridor = corridor_for_position(sign.x, sign.y)
    # South/North corridors run along x; East/West run along y. "Depth" is the
    # along-corridor axis the sign grid is defined on.
    along_x = corridor in (Section.SOUTH, Section.NORTH)
    depth, lateral = (sign.x, sign.y) if along_x else (sign.y, sign.x)

    return HitReport(
        label=scenario.label,
        collided=result.collided,
        laps=result.laps_completed,
        steps=result.steps,
        sign_index=nearest,
        sign_color=sign.color,
        sign_xy=(sign.x, sign.y),
        corridor=corridor.value if hasattr(corridor, "value") else str(corridor),
        depth=depth,
        lateral=lateral,
        centre_dist=math.hypot(sign.x - x, sign.y - y),
    )


def main() -> None:
    """Run every fixture and print which sign each collision belongs to."""
    with ProcessPoolExecutor(max_workers=8) as pool:
        reports = list(pool.map(_analyse, range(len(all_obstacles_demo_scenarios()))))

    depths: Counter[float] = Counter()
    for r in reports:
        if not r.collided:
            print(f"{r.label:<40} NO COLLISION laps={r.laps}")
            continue
        depths[round(r.depth, 2)] += 1
        print(
            f"{r.label:<40} sign#{r.sign_index} {r.sign_color:<5} "
            f"at=({r.sign_xy[0]:.2f},{r.sign_xy[1]:.2f}) {r.corridor:<6} "
            f"depth={r.depth:.2f} lateral={r.lateral:.2f} "
            f"gap={r.centre_dist * 100:.1f}cm steps={r.steps}"
        )

    print("\nCollisions by sign depth along its corridor:")
    for depth, count in sorted(depths.items()):
        print(f"  depth {depth:.2f}: {count}")


if __name__ == "__main__":
    main()
