"""Run closed-loop Open Challenge scenario(s) live, visualized in RViz.

Drives the exact same ``ScenarioSimulator`` + real ``CoreNavigator`` the
headless test battery (``tests/unit/test_open_challenge_sim.py``) runs, but
paces it to wall-clock speed and publishes pose/LIDAR/track to ROS2 for RViz.

Launch RViz from this same pixi environment — ``task sim:navigate:rviz``
(pre-configured with TF/LaserScan/Odometry/track displays) — not
``task sim:rviz``, which starts a *separate* pixi/ROS2 install under
gazebo/runtime and may fail to discover these topics over DDS.

Usage (from platform/robot, in the pixi ``dev`` env — headless tests never
need this env, only this script does):

    # Ad-hoc scenario (custom widths/section/direction):
    pixi run -e dev visualize-scenario
    pixi run -e dev visualize-scenario -- --south 600 --north 600 --section south --direction cw --rate 2

    # The exact scenarios TestThreeLapSolvability runs, by index or label:
    pixi run -e dev visualize-scenario -- --list
    pixi run -e dev visualize-scenario -- --scenario 0
    pixi run -e dev visualize-scenario -- --scenario symmetric_narrow[South/clockwise]

    # Step through all 28 test scenarios, pausing between each:
    pixi run -e dev visualize-scenario -- --interactive

    # Obstacles Challenge demo scenarios (sign routing + parking — NOT a pytest
    # battery, see scenario_catalog.all_obstacles_demo_scenarios):
    pixi run -e dev visualize-scenario -- --challenge obstacles --list
    pixi run -e dev visualize-scenario -- --challenge obstacles --interactive

    # Real official-scenario metadata from the Go generator (the actual WRO
    # 2026 36-scenario sign table), visualized live instead of the demo layout
    # above. From platform/gazebo/generator:
    #   go run ./cmd/simgen generate --challenge obstacles --num-scenarios 1 --deterministic --output-dir ../training_data
    # Then from platform/robot:
    pixi run -e dev visualize-scenario -- --metadata-file ../training_data/scenarios/scenario_0000_metadata.json
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section

from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.gateway import CONTROL_DT, ScenarioSimulator, SimResult
from src.simulation.live_visualizer import (
    LiveScenarioVisualizer,
    RealTimePacer,
    init_rclpy_once,
)
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_catalog import (
    NamedScenario,
    all_obstacles_demo_scenarios,
    all_test_scenarios,
    find_scenario,
)
from src.simulation.track_model import TrackModel

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

logger = logging.getLogger(__name__)

_WIDE_MM = int(CorridorDimensions.WIDE * 1000)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--south", type=int, default=_WIDE_MM, help="South corridor width (mm).")
    parser.add_argument("--north", type=int, default=_WIDE_MM, help="North corridor width (mm).")
    parser.add_argument("--east", type=int, default=_WIDE_MM, help="East corridor width (mm).")
    parser.add_argument("--west", type=int, default=_WIDE_MM, help="West corridor width (mm).")
    parser.add_argument(
        "--section", choices=["south", "north", "east", "west"], default="south",
        help="Starting section (ignored with --scenario/--interactive).",
    )
    parser.add_argument(
        "--direction", choices=["cw", "ccw"], default="cw",
        help="Travel direction (ignored with --scenario/--interactive).",
    )
    parser.add_argument("--laps", type=int, default=3, help="Target lap count.")
    parser.add_argument(
        "--rate", type=float, default=1.0,
        help="Playback speed multiplier (1.0 = real time, 0 = as fast as possible).",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List the scenarios TestThreeLapSolvability runs, then exit.",
    )
    parser.add_argument(
        "--scenario", metavar="INDEX_OR_LABEL",
        help="Run one named test scenario (see --list) instead of an ad-hoc one.",
    )
    parser.add_argument(
        "--interactive", action="store_true",
        help="Step through every test scenario in order, pausing between each.",
    )
    parser.add_argument(
        "--challenge", choices=["open", "obstacles"], default="open",
        help="Which catalog --list/--scenario/--interactive operate over.",
    )
    parser.add_argument(
        "--metadata-file", metavar="PATH",
        help="Run a *_metadata.json file directly (e.g. from `simgen generate`) "
             "instead of a catalog scenario or the ad-hoc widths above.",
    )
    return parser.parse_args()


def _catalog(challenge: str) -> list[NamedScenario]:
    return all_test_scenarios() if challenge == "open" else all_obstacles_demo_scenarios()


def _track_for(metadata: dict[str, Any]) -> TrackModel:
    """Build just the track geometry — cheaper than a full ``ScenarioSimulator``."""
    return TrackModel(corridor_widths_from_metadata(metadata))


def _set_track(visualizer: LiveScenarioVisualizer, metadata: dict[str, Any], track: TrackModel) -> None:
    visualizer.set_track(
        track, sign_positions=metadata["sign_positions"], parking_lot=metadata.get("parking_lot"),
    )


def _run_one(
    scenario: NamedScenario, visualizer: LiveScenarioVisualizer, rate: float,
) -> SimResult:
    """Run a single named scenario against the live visualizer."""
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed)
    _set_track(visualizer, scenario.metadata, sim.track)
    pacer = RealTimePacer(dt=CONTROL_DT, rate=rate)

    def on_step(state: AckermannState, scan: LidarScan | None) -> None:
        visualizer.publish(state, scan)
        pacer.wait()

    return sim.run(on_step=on_step)


def _log_result(label: str, result: SimResult) -> None:
    status = "SUCCESS" if result.success else "FAILED"
    logger.info(
        "%s | %s | laps=%d/%d collided=%s timeout=%s dist=%.2fm t=%.1fs",
        status, label, result.laps_completed, result.target_laps,
        result.collided, result.timed_out, result.distance_m, result.sim_time_s,
    )


def _run_and_visualize(scenario: NamedScenario, rate: float) -> None:
    scenario_track = _track_for(scenario.metadata)
    visualizer = LiveScenarioVisualizer(scenario_track)
    _set_track(visualizer, scenario.metadata, scenario_track)
    logger.info(
        "Publishing /sim/odom, /scan, /sim/track — run `task sim:navigate:rviz` "
        "in another terminal to watch.",
    )
    result = _run_one(scenario, visualizer, rate)
    _log_result(scenario.label, result)
    visualizer.destroy_node()


def main() -> None:
    """Entry point for `python -m src.simulation.visualize_scenario`."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = _parse_args()

    if args.list:
        for i, s in enumerate(_catalog(args.challenge)):
            logger.info("%2d  %s", i, s.label)
        return

    init_rclpy_once()

    if args.interactive:
        scenarios = _catalog(args.challenge)
        first_track = _track_for(scenarios[0].metadata)
        visualizer = LiveScenarioVisualizer(first_track)
        _set_track(visualizer, scenarios[0].metadata, first_track)
        logger.info(
            "Publishing /sim/odom, /scan, /sim/track — run `task sim:navigate:rviz` "
            "in another terminal to watch. %d scenarios queued.", len(scenarios),
        )
        try:
            for i, scenario in enumerate(scenarios):
                logger.info("--- [%d/%d] %s ---", i + 1, len(scenarios), scenario.label)
                result = _run_one(scenario, visualizer, args.rate)
                _log_result(scenario.label, result)
                if i < len(scenarios) - 1:
                    input("Press Enter for the next scenario (Ctrl+C to stop)... ")
        except KeyboardInterrupt:
            logger.info("Stopped.")
        visualizer.destroy_node()
        return

    if args.metadata_file is not None:
        path = Path(args.metadata_file)
        metadata = json.loads(path.read_text())
        scenario = NamedScenario(label=path.name, metadata=metadata, laps=args.laps, seed=0)
        _run_and_visualize(scenario, args.rate)
        return

    if args.scenario is not None:
        scenario = find_scenario(args.scenario, _catalog(args.challenge))
        _run_and_visualize(scenario, args.rate)
        return

    widths = uniform_widths(_WIDE_MM)
    widths.update(south=args.south, north=args.north, east=args.east, west=args.west)
    section = Section.from_string(args.section)
    direction = Direction.CLOCKWISE if args.direction == "cw" else Direction.COUNTERCLOCKWISE
    metadata = build_open_metadata(widths, section, direction)
    sim = ScenarioSimulator(metadata, num_laps=args.laps)

    visualizer = LiveScenarioVisualizer(sim.track)
    pacer = RealTimePacer(dt=CONTROL_DT, rate=args.rate)

    def on_step(state: AckermannState, scan: LidarScan | None) -> None:
        visualizer.publish(state, scan)
        pacer.wait()

    logger.info(
        "Publishing /sim/odom, /scan, /sim/track — run `task sim:navigate:rviz` "
        "in another terminal to watch (displays pre-configured).",
    )
    result = sim.run(on_step=on_step)
    _log_result("ad-hoc", result)
    visualizer.destroy_node()


if __name__ == "__main__":
    main()
