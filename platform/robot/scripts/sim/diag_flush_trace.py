"""Tick-by-tick trace of one narrow-corridor middle-band start.

Every scenario starting in the middle band of a narrow corridor fails: 23 of 23
in the exhaustive sweep, 8 of 8 in ``diag_flush_start``, under both contact
policies and at the corrected chassis width. They do not collide -- they simply
never complete a lap, timing out at 200 s with 0 of 3.

That rules out geometry and harness policy, so the question is what the robot
actually does. This dumps the commanded drive and the resulting motion for one
such run, next to the clearances the navigator is seeing, so the failure can be
read rather than guessed at.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning

from scripts.common.tables import print_table
from src.navigation.utils import _forward_clearance, _nearest_ray
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_simulator import ScenarioSimulator

_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_DEFAULT_CELL = 2
_DEFAULT_TICKS = 400
_DEFAULT_DOWNSAMPLE = 10
_DEFAULT_LAPS = 3
_CONTROL_DT_S = 0.05


def main() -> None:
    """Trace one failing start and print a downsampled per-tick table."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--section", default="south")
    parser.add_argument("--direction", default="counterclockwise")
    parser.add_argument("--cell", type=int, default=_DEFAULT_CELL, help="2 or 3 = the middle band.")
    parser.add_argument("--ticks", type=int, default=_DEFAULT_TICKS, help="How many ticks to trace.")
    parser.add_argument("--every", type=int, default=_DEFAULT_DOWNSAMPLE, help="Print one row per N ticks.")
    args = parser.parse_args()

    section = Section(args.section.upper()) if args.section.isupper() else Section[args.section.upper()]
    direction = Direction[args.direction.upper()]

    meta = build_open_metadata(uniform_widths(_NARROW_MM), section, direction, start_cell=args.cell)
    sim = ScenarioSimulator(meta, num_laps=_DEFAULT_LAPS, seed=0, blind=True)
    tuning = NavigationTuning.load_default()

    start = meta.starting_conditions.position
    print(f"{section.value}/{direction} celda {args.cell}  arranque=({start.x:.3f}, {start.y:.3f})\n")

    # The drive command is what the navigator decided; the state is what the
    # body did with it. Separating the two is the whole point: a robot that
    # commands speed and does not move is pinned, one that commands zero has
    # decided to stop.
    commands: list[tuple[float, float]] = []
    gw = sim.gateway
    real_publish = gw.publish_drive

    def spy(cmd: object) -> None:
        commands.append((cmd.speed_mps, cmd.steering_norm))
        real_publish(cmd)

    gw.publish_drive = spy  # type: ignore[method-assign]

    rows: list[str] = []

    def on_step(state: object, scan: object) -> None:
        i = len(rows)
        if i >= args.ticks:
            return
        ranges, angles = scan.ranges_m, scan.angles_rad
        fwd = _forward_clearance(ranges, angles, tuning)
        left = _nearest_ray(ranges, angles, math.pi / 2)
        right = _nearest_ray(ranges, angles, -math.pi / 2)
        cmd_v, cmd_s = commands[-1] if commands else (float("nan"), float("nan"))
        rows.append(
            f"{i:4d} {i * _CONTROL_DT_S:6.2f}s  pos=({state.x:5.3f},{state.y:5.3f}) yaw={math.degrees(state.yaw):7.2f} "
            f"v={state.v:6.3f} steer={math.degrees(state.steer):6.2f}  "
            f"cmd_v={cmd_v:6.3f} cmd_s={cmd_s:6.3f}  "
            f"fwd={fwd:5.2f} L={left:5.2f} R={right:5.2f}",
        )

    result = sim.run(on_step=on_step)

    for i, row in enumerate(rows):
        if i % args.every == 0:
            print(row, flush=True)

    print(
        f"\nresultado: success={result.success} laps={result.laps_completed}/3 "
        f"collided={result.collided} contactos={result.contact_count} "
        f"t={result.sim_time_s:.1f}s dist={result.distance_m:.2f}m",
    )


if __name__ == "__main__":
    main()
