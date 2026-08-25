"""Run a blind Open Challenge scenario and record path-tracking error.

Compares the robot's ground-truth pose against both:

- the planned path as the navigator believes it (the RViz-displayed path)
- the planned path against the true track geometry

Outputs one CSV per scenario with per-tick cross-track errors and the current
belief/true corridor widths.

Usage (from ``platform/robot``, with PYTHONPATH=".")::

    python scripts/sim/diag_path_track.py --scenario 450
"""

from __future__ import annotations

import os

os.environ.setdefault("VTITAN_HARDWARE_PROFILE", "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from typing import TYPE_CHECKING

from shared.domain.enums import Section
from shared.domain.models import ScenarioMetadata, Waypoint

from scripts.common.diag_base import add_tuning_arg, load_tuning
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.track_geometry import project_onto_path
from src.simulation.scenario_catalog import open_scenario_by_index
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState


def _transform_point(
    x_b: float,
    y_b: float,
    believed: tuple[float, float, float],
    true: tuple[float, float, float],
) -> tuple[float, float]:
    """Map a point from the believed frame to the true/map frame."""
    bx, by, byaw = believed
    tx, ty, tyaw = true
    delta_yaw = (tyaw - byaw + math.pi) % (2 * math.pi) - math.pi
    cos_d, sin_d = math.cos(delta_yaw), math.sin(delta_yaw)
    offset_x = tx - (bx * cos_d - by * sin_d)
    offset_y = ty - (bx * sin_d + by * cos_d)
    x_m = x_b * cos_d - y_b * sin_d + offset_x
    y_m = x_b * sin_d + y_b * cos_d + offset_y
    return x_m, y_m


def _displayed_path(
    waypoints: list[Waypoint],
    believed: tuple[float, float, float],
    true: tuple[float, float, float],
) -> list[Waypoint]:
    """Return the believed waypoints transformed into the map frame."""
    return [Waypoint(*_transform_point(wp.x, wp.y, believed, true)) for wp in waypoints]


_DEFAULT_OUTPUT_DIR = Path("scripts/sim/output")


def _write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def _safe_width(widths: dict[Section, float] | None, section: Section) -> float | None:
    if widths is None:
        return None
    return widths.get(section)


def run_scenario(
    scenario_index: int,
    *,
    laps: int,
    blind: bool,
    use_lidar_localization: bool,
    tuning_path: str | None,
    max_steps: int,
    output_dir: Path,
) -> tuple[Path, list[dict]]:
    """Simulate one scenario and write a CSV.

    Returns the CSV path and the collected rows.
    """
    scenario = open_scenario_by_index(scenario_index)
    print(f"Scenario {scenario_index}: {scenario.label}")
    print(f"True start: {scenario.metadata['starting_conditions']}")

    tuning = load_tuning(tuning_path)
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=laps,
        blind=blind,
        use_lidar_localization=use_lidar_localization,
        tuning=tuning,
    )
    print(f"Initial belief offset: {sim.belief_offset_poses}")

    meta = ScenarioMetadata.model_validate(scenario.metadata)
    true_path = calculate_waypoints(meta, num_laps=laps, tuning=tuning)
    print(f"True path has {len(true_path)} waypoints")

    rows: list[dict] = []

    def on_step(state: AckermannState, scan: LidarScan) -> None:  # noqa: ARG001 - scan required by callback signature
        believed, true = sim.belief_offset_poses
        # The navigator's current tracked path is private; this is a diagnostic
        # peek, not production code.
        path_map = _displayed_path(sim.navigator._waypoints, believed, true)  # noqa: SLF001
        proj_displayed = project_onto_path(path_map, state.x, state.y)
        proj_true = project_onto_path(true_path, state.x, state.y)
        widths = sim.believed_widths
        rows.append({
            "step": len(rows),
            "x": state.x,
            "y": state.y,
            "believed_x": believed[0],
            "believed_y": believed[1],
            "believed_yaw": believed[2],
            "true_x": true[0],
            "true_y": true[1],
            "true_yaw": true[2],
            "path_x": proj_displayed.x,
            "path_y": proj_displayed.y,
            "cte_displayed_m": proj_displayed.distance_m,
            "signed_offset_displayed_m": proj_displayed.signed_offset_m,
            "cte_true_m": proj_true.distance_m,
            "signed_offset_true_m": proj_true.signed_offset_m,
            "south_m": _safe_width(widths, Section.SOUTH),
            "north_m": _safe_width(widths, Section.NORTH),
            "east_m": _safe_width(widths, Section.EAST),
            "west_m": _safe_width(widths, Section.WEST),
        })

    result = sim.run(on_step=on_step, max_steps=max_steps)
    print(
        f"Result: success={result.success} laps={result.laps_completed} "
        f"collided={result.collided} timed_out={result.timed_out}",
    )

    ctes = [r["cte_displayed_m"] for r in rows if r["cte_displayed_m"] is not None]
    if ctes:
        print(
            f"CTE vs displayed path: min={min(ctes):.3f} max={max(ctes):.3f} "
            f"mean={sum(ctes) / len(ctes):.3f} m",
        )

    csv_path = output_dir / f"diag_path_track_{scenario_index}.csv"
    _write_csv(rows, csv_path)
    print(f"Wrote {csv_path}\n")
    return csv_path, rows


def main() -> None:
    """Parse arguments and run the requested scenario."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=int, default=450, help="Scenario index to run.")
    parser.add_argument("--laps", type=int, default=3, help="Number of laps to simulate.")
    parser.add_argument("--blind", action="store_true", default=True, help="Run blind (default).")
    parser.add_argument("--sighted", action="store_true", help="Override --blind and run sighted.")
    parser.add_argument("--no-lidar", action="store_true", help="Disable LIDAR localisation.")
    parser.add_argument("--max-steps", type=int, default=3000, help="Simulation step budget.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_DEFAULT_OUTPUT_DIR,
        help="Directory for the output CSV.",
    )
    add_tuning_arg(parser)
    args = parser.parse_args()

    run_scenario(
        args.scenario,
        laps=args.laps,
        blind=not args.sighted,
        use_lidar_localization=not args.no_lidar,
        tuning_path=args.tuning,
        max_steps=args.max_steps,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
