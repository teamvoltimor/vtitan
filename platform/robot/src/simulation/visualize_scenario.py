"""Run one closed-loop Open Challenge scenario live, visualized in RViz/Gazebo.

Drives the exact same ``ScenarioSimulator`` + real ``CoreNavigator`` the
headless test battery (``tests/unit/test_open_challenge_sim.py``) runs, but
paces it to wall-clock speed and publishes pose/LIDAR/track to ROS2 so RViz
(``task sim:rviz``) — or Gazebo, for the default 1000mm layout that matches
``worlds/wro_track_2026.sdf`` — can show it running.

Usage (from platform/robot, in the pixi ``dev`` env — headless tests never
need this env, only this script does):

    pixi run -e dev python -m src.simulation.visualize_scenario
    pixi run -e dev python -m src.simulation.visualize_scenario \\
        --south 600 --north 600 --section south --direction cw --rate 2
"""

from __future__ import annotations

import argparse
import logging

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section

from src.navigation.ports import LidarScan
from src.simulation.gateway import CONTROL_DT, ScenarioSimulator
from src.simulation.kinematics import AckermannState
from src.simulation.live_visualizer import (
    LiveScenarioVisualizer,
    RealTimePacer,
    init_rclpy_once,
)
from src.simulation.scenario_builder import build_open_metadata, uniform_widths

logger = logging.getLogger(__name__)

_WIDE_MM = int(CorridorDimensions.WIDE * 1000)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--south", type=int, default=_WIDE_MM, help="South corridor width (mm).")
    parser.add_argument("--north", type=int, default=_WIDE_MM, help="North corridor width (mm).")
    parser.add_argument("--east", type=int, default=_WIDE_MM, help="East corridor width (mm).")
    parser.add_argument("--west", type=int, default=_WIDE_MM, help="West corridor width (mm).")
    parser.add_argument(
        "--section", choices=["south", "north", "east", "west"], default="south",
        help="Starting section.",
    )
    parser.add_argument(
        "--direction", choices=["cw", "ccw"], default="cw", help="Travel direction.",
    )
    parser.add_argument("--laps", type=int, default=3, help="Target lap count.")
    parser.add_argument(
        "--rate", type=float, default=1.0,
        help="Playback speed multiplier (1.0 = real time, 0 = as fast as possible).",
    )
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = _parse_args()

    widths = uniform_widths(_WIDE_MM)
    widths.update(south=args.south, north=args.north, east=args.east, west=args.west)
    section = Section.from_string(args.section)
    direction = Direction.CLOCKWISE if args.direction == "cw" else Direction.COUNTERCLOCKWISE
    metadata = build_open_metadata(widths, section, direction)

    sim = ScenarioSimulator(metadata, num_laps=args.laps)

    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(sim.track)
    pacer = RealTimePacer(dt=CONTROL_DT, rate=args.rate)

    def on_step(state: AckermannState, scan: LidarScan | None) -> None:
        visualizer.publish(state, scan)
        pacer.wait()

    logger.info(
        "Publishing /sim/odom, /scan, /sim/track — open RViz (task sim:rviz) and "
        "add TF, LaserScan (topic=/scan), and MarkerArray (topic=/sim/track) displays, "
        "fixed frame = 'map'.",
    )
    result = sim.run(on_step=on_step)

    status = "SUCCESS" if result.success else "FAILED"
    logger.info(
        "%s | laps=%d/%d collided=%s timeout=%s dist=%.2fm t=%.1fs",
        status, result.laps_completed, result.target_laps,
        result.collided, result.timed_out, result.distance_m, result.sim_time_s,
    )

    visualizer.destroy_node()


if __name__ == "__main__":
    main()
