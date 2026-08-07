"""Does the blind narrow prior drive the robot into the outer wall on a wide track?

The 2026-08-06 hardware rounds all failed the same way: a corridor believed
narrow (0.60) puts the planned path ~0.25-0.30 m from the outer wall, while
``LOOKAHEAD_TRANSITION`` (0.30 m) is the crosstrack at which the corrective
short lookahead engages. Subtract the chassis half-width and only 0.15-0.20 m
of crosstrack is available before contact -- so the correction is armed to fire
only after the wall has already been reached. Measured on the CW bag: crosstrack
ran 0.09 -> 0.15 through the corner, never crossed 0.30, and the robot ended up
0.10 m from the wall.

That is a geometric argument, so it should reproduce without any of the sensor
noise the simulator is known not to model (see the 2026-08-03 findings, where
sim could not reproduce the hardware control bugs). The condition it needs is a
track that is genuinely WIDE while the blind prior says narrow -- a uniformly
narrow track is not a reproduction, because there the prior is simply right.

Reports wall-contact episodes, not just pass/fail: a run that scrapes and
recovers still demonstrates the mechanism, and ``contact_count`` is the only
field that distinguishes it from a clean one.

Usage:
    pixi run -e dev python scripts/diag_wide_wall_hug.py [--width-mm 1000] [--jobs 8]
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import print_table
from shared.config.constants import CompetitionSpecs, CorridorDimensions
from shared.config.enums import Direction, Section

from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.simulated_hardware_gateway import SensorErrors

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_STARTS = list(product(Section, Direction))

_REAL_ERRORS = SensorErrors(
    # Post-626a011 the start pose is measured off the scan rather than assumed,
    # and landed within 3.6-4.8 cm on all three real bags -- so this is the
    # residual that survives the measurement, not the old assumption's 0.35-0.80 m.
    start_pos_error_m=0.05,
    yaw_bias_rad=0.03,
    imu_drift_rad_per_s=0.000145,  # BNO085's quoted 0.5 deg/min
    gyro_scale_error=0.005,
    imu_noise_rad=0.005,
)
"""Perturbations sized from the real hardware, for asking whether the wall-hug
becomes a failure once the pose is not perfect."""


@dataclass(frozen=True, slots=True)
class _WideWallResult:
    """Result from running a wide-track wall-hug diagnostic."""

    label: str
    success: bool
    contact_count: int
    contact_time_s: float
    min_lidar_range_m: float
    laps_completed: int


def _run(args: tuple[int, int, bool]) -> _WideWallResult:
    """Run one start and report how close it came to the walls."""
    width_mm, index, with_errors = args
    section, direction = _STARTS[index]
    meta = build_open_metadata(uniform_widths(width_mm), section, direction)
    result = ScenarioSimulator(
        meta,
        num_laps=_N_LAPS,
        sensor_errors=_REAL_ERRORS if with_errors else SensorErrors(),
    ).run()
    label = f"{section.value:>5}/{'CW' if direction is Direction.CLOCKWISE else 'CCW':<3}"
    return _WideWallResult(
        label=label,
        success=result.success,
        contact_count=result.contact_count,
        contact_time_s=result.contact_time_s,
        min_lidar_range_m=result.min_lidar_range_m,
        laps_completed=result.laps_completed,
    )


def main() -> None:
    """Run every start on a wide track and report wall contact."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--width-mm", type=int, default=int(CorridorDimensions.WIDE * 1000))
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--errors", action="store_true", help="apply the real-hardware sensor error profile")
    args = parser.parse_args()

    narrow_mm = int(CorridorDimensions.NARROW * 1000)
    print(
        f"true corridor width: {args.width_mm} mm   blind prior: {narrow_mm} mm   "
        f"{len(_STARTS)} starts   sensor errors: {'on' if args.errors else 'off'}"
    )

    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(_run, [(args.width_mm, i, args.errors) for i in range(len(_STARTS))]))

    contacted = 0
    passed = 0
    table_rows = []
    for r in rows:
        passed += r.success
        contacted += r.contact_count > 0
        table_rows.append((r.label, "yes" if r.success else "NO", r.laps_completed, r.contact_count, r.contact_time_s, r.min_lidar_range_m))
    print_table(table_rows, ["start", "pass", "laps", "contacts", "contact_s", "min_rng"], floatfmt=[None, None, None, None, ".2f", ".3f"])
    print(f"\n{passed}/{len(rows)} passed   {contacted}/{len(rows)} touched a wall at least once")


if __name__ == "__main__":
    main()
