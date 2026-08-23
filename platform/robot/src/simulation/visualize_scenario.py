"""Run closed-loop Open Challenge scenario(s) live, visualized in RViz.

Drives the exact same ``ScenarioSimulator`` + real ``CoreNavigator`` the
headless test battery (``tests/unit/test_open_challenge_sim.py``) runs, but
paces it to wall-clock speed and publishes pose/LIDAR/track to ROS2 for RViz.

Launch RViz with ``task sim:navigate:rviz`` (pre-configured with
TF/LaserScan/Odometry/track/robot-model displays), not ``task sim:rviz``, which
loads no saved config and so comes up as an empty grid.

Usage (from platform/robot, in the pixi ``dev`` env — headless tests never
need this env, only this script does):

    # Ad-hoc scenario (custom widths/section/direction):
    pixi run -e dev visualize-scenario
    pixi run -e dev visualize-scenario -- --south 600 --north 600 --section south --direction cw --rate 2

    # The catalog scenarios, by index or label — omit --challenge to see both
    # catalogs (Open Challenge then Obstacles Challenge); pass --challenge
    # open|obstacles to scope to just one. The Open Challenge catalog is
    # generated on demand (640 legal width/section/direction/cell combos);
    # Obstacles still loads the Go-generated fixtures:
    pixi run -e dev visualize-scenario -- --list
    pixi run -e dev visualize-scenario -- --scenario 0
    pixi run -e dev visualize-scenario -- --challenge open --scenario 123
    pixi run -e dev visualize-scenario -- --challenge open --scenario open_0123[south/cw]
    pixi run -e dev visualize-scenario -- --challenge obstacles --list

    # Step through every catalog scenario, pausing between each. No
    # --challenge means both catalogs back-to-back (Open, then Obstacles);
    # --challenge open|obstacles scopes to just one:
    pixi run -e dev visualize-scenario -- --interactive
    pixi run -e dev visualize-scenario -- --challenge obstacles --interactive

    # Blind (the default): the robot is not handed the corridor widths and
    # estimates them from LIDAR. RViz still draws the true track, so a corridor
    # it has mis-learned shows up as a path hugging the wrong wall.
    #
    # Position is ground truth here unless you ask otherwise -- add --localize
    # to make it steer on its own estimate too, which is what the real robot
    # does and what the headless battery scores:
    pixi run -e dev visualize-scenario -- --challenge open --interactive --localize --rate 5

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
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from shared.domain.enums import Direction, Section
from shared.domain.models import ParkingLot, SignPosition

from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.imu_error_model import SensorErrors
from src.simulation.live_visualizer import (
    LiveScenarioVisualizer,
    RealTimePacer,
    init_rclpy_once,
)
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_catalog import (
    NamedScenario,
    all_obstacles_demo_scenarios,
    all_open_scenarios,
    all_test_scenarios,
    find_scenario,
)
from src.simulation.scenario_constants import WIDE_MM
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.simulated_hardware_gateway import CONTROL_DT
from src.simulation.track_model import TrackModel

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState
    from src.simulation.scenario_result import SimResult

logger = logging.getLogger(__name__)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--south", type=int, default=WIDE_MM, help="South corridor width (mm).")
    parser.add_argument("--north", type=int, default=WIDE_MM, help="North corridor width (mm).")
    parser.add_argument("--east", type=int, default=WIDE_MM, help="East corridor width (mm).")
    parser.add_argument("--west", type=int, default=WIDE_MM, help="West corridor width (mm).")
    parser.add_argument(
        "--section",
        choices=["south", "north", "east", "west"],
        default="south",
        help="Starting section (ignored with --scenario/--interactive).",
    )
    parser.add_argument(
        "--direction",
        choices=["cw", "ccw"],
        default="cw",
        help="Travel direction (ignored with --scenario/--interactive).",
    )
    parser.add_argument("--laps", type=int, default=3, help="Target lap count.")
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = real time, 0 = as fast as possible).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List the scenarios TestThreeLapSolvability runs, then exit.",
    )
    parser.add_argument(
        "--scenario",
        metavar="INDEX_OR_LABEL",
        help="Run one named test scenario (see --list) instead of an ad-hoc one.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Step through every test scenario in order, pausing between each.",
    )
    parser.add_argument(
        "--challenge",
        choices=["open", "obstacles"],
        default=None,
        help="Which catalog --list/--scenario/--interactive operate over. "
        "Omit to run both catalogs (Open Challenge scenarios, then Obstacles Challenge).",
    )
    parser.add_argument(
        "--localize",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Navigate on the LidarLocalizer position estimate instead of ground-truth "
        "pose, matching what the real robot does. OFF by default HERE, unlike the "
        "headless battery: this script exists to watch the planner, and localization "
        "error lands on the same picture as the manoeuvre being studied, so it is opt-in "
        "rather than baked in. The published pose stays ground truth either way, so with "
        "--localize any wandering you see is state-estimation error reaching control. "
        "Turn it on before drawing any conclusion about whether a run would pass.",
    )
    parser.add_argument(
        "--blind",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Withhold the corridor layout too, so the robot estimates the widths from "
        "LIDAR instead of being handed them. On by default, since a round is never "
        "driven knowing the widths; --no-blind hands them over. Does NOT imply "
        "--localize (an earlier version of this help said it did, and nothing ever "
        "enforced it): the layout and the robot's own pose are withheld separately. "
        "RViz shows the true track, so a corridor the robot has mis-learned shows up "
        "as a path hugging the wrong wall.",
    )
    parser.add_argument(
        "--place-error",
        type=float,
        default=0.0,
        metavar="CM",
        help="Withhold the exact starting pose: seed the estimator this far (cm, random "
        "bearing) from where the chassis actually is, as a hand placement in the "
        "starting zone would. Turns --localize on, since it perturbs that estimate.",
    )
    parser.add_argument(
        "--yaw-bias",
        type=float,
        default=0.0,
        metavar="DEG",
        help="Constant offset between the IMU's yaw zero and the world frame. Never "
        "corrected — nothing else observes absolute heading. Turns --localize on.",
    )
    parser.add_argument(
        "--imu-drift",
        type=float,
        default=0.0,
        metavar="DEG_PER_S",
        help="IMU yaw drift rate, accumulated over the run. Turns --localize on.",
    )
    parser.add_argument(
        "--gyro-scale",
        type=float,
        default=0.0,
        metavar="PCT",
        help="Gyro scale-factor error as a percentage (e.g. 0.5). Accumulates per degree "
        "turned rather than per second, so it grows with corners driven. Turns --localize on.",
    )
    parser.add_argument(
        "--imu-noise",
        type=float,
        default=0.0,
        metavar="DEG",
        help="Per-reading Gaussian yaw noise (standard deviation, degrees). Bounded and "
        "self-cancelling, unlike drift and scale. Turns --localize on.",
    )
    parser.add_argument(
        "--recover",
        action="store_true",
        help="Make walls solid and let the robot escape from contact instead of ending "
        "the run at the first touch. The chassis is held where it is rather than "
        "passing through, so the navigator's reversing escape has to actually free "
        "it; the run ends only if it stays pinned for --contact-grace seconds.",
    )
    parser.add_argument(
        "--contact-grace",
        type=float,
        default=5.0,
        metavar="SEC",
        help="With --recover, how long the robot may stay pinned before the run is called a failure (default: 5).",
    )
    parser.add_argument(
        "--metadata-file",
        metavar="PATH",
        help="Run a *_metadata.json file directly (e.g. from `simgen generate`) "
        "instead of a catalog scenario or the ad-hoc widths above.",
    )
    return parser.parse_args()


def _catalog(challenge: str | None) -> list[NamedScenario]:
    """Scenarios for the given challenge, or both catalogs (Open then Obstacles) if omitted."""
    if challenge == "open":
        return all_open_scenarios()
    if challenge == "obstacles":
        return all_obstacles_demo_scenarios()
    return all_open_scenarios() + all_obstacles_demo_scenarios()


def _track_for(metadata: dict[str, Any]) -> TrackModel:
    """Build just the track geometry — cheaper than a full ``ScenarioSimulator``."""
    return TrackModel(corridor_widths_from_metadata(metadata))


def _set_track(visualizer: LiveScenarioVisualizer, metadata: dict[str, Any], track: TrackModel) -> None:
    """Validate the raw metadata into models, then hand those to the visualizer.

    This is the one place scenario JSON crosses into the drawing code, so it is
    where the shape gets checked. Passing the dicts through unvalidated is what
    let whole-number coordinates reach a Point field as ``int`` and serialize
    to a near-zero subnormal -- a sign drawn inside a wall, with nothing
    anywhere raising.
    """
    parking_lot = metadata.get("parking_lot")
    visualizer.set_track(
        track,
        sign_positions=[SignPosition.model_validate(sign) for sign in metadata["sign_positions"]],
        parking_lot=ParkingLot.model_validate(parking_lot) if parking_lot is not None else None,
    )


@dataclass(frozen=True, slots=True)
class _RunOptions:
    """Everything the CLI can vary about how a scenario is driven."""

    rate: float = 1.0
    # `blind` defaults on so the CLI drives the layout the robot will actually
    # meet, rather than the easiest version of it -- pass --no-blind to relax
    # that on purpose.
    #
    # `localize` does NOT, and only in this script. The headless battery keeps
    # it on because it is scoring runs, where ground-truth pose flatters every
    # number (see ScenarioSimulator's 2026-08-01 note). This one is a
    # microscope: it draws the plan and the pose in the same frame, so with
    # localization on, state-estimation error is superimposed on the manoeuvre
    # being read, and the two are indistinguishable by eye. Off by default
    # isolates the planner; --localize puts the error back when the question is
    # whether the run survives rather than what the planner did.
    localize: bool = False
    blind: bool = True
    errors: SensorErrors | None = None
    recover: bool = False
    contact_grace_s: float = 5.0

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> _RunOptions:
        errors = SensorErrors(
            start_pos_error_m=args.place_error / 100.0,
            yaw_bias_rad=math.radians(args.yaw_bias),
            imu_drift_rad_per_s=math.radians(args.imu_drift),
            gyro_scale_error=args.gyro_scale / 100.0,
            imu_noise_rad=math.radians(args.imu_noise),
        )
        # Every sensor error perturbs the ESTIMATE, so with ground-truth pose
        # they are all no-ops. Since `localize` now defaults off, asking for one
        # has to switch it back on -- otherwise `--place-error 5` runs clean and
        # silently answers a question nobody asked.
        return cls(
            rate=args.rate,
            localize=args.localize or errors.any_error,
            blind=args.blind,
            errors=errors,
            recover=args.recover,
            contact_grace_s=args.contact_grace,
        )


def _run_one(
    scenario: NamedScenario,
    visualizer: LiveScenarioVisualizer,
    opts: _RunOptions,
) -> SimResult:
    """Run a single named scenario against the live visualizer."""
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        use_lidar_localization=opts.localize,
        blind=opts.blind,
        sensor_errors=opts.errors,
        solid_walls=opts.recover,
    )
    _set_track(visualizer, scenario.metadata, sim.track)
    # Blind seeds the believed start from a fixed SOUTH guess, so on any
    # scenario that does not actually start there the robot's whole plan lives
    # in a frame rotated by the section-relabelling angle. Registering the
    # offset lets RViz draw the plan over the real track; without it, a WEST
    # start (e.g. go_obstacles_0002) shows a path square to the layout.
    visualizer.set_belief_frame(*sim.belief_offset_poses)
    pacer = RealTimePacer(dt=CONTROL_DT, rate=opts.rate)

    def on_step(state: AckermannState, scan: LidarScan | None) -> None:
        visualizer.publish(state, scan)
        # The believed half, published from the live navigator rather than the
        # metadata: under Obstacles the planned polyline IS the avoidance
        # manoeuvre, and blind runs route around discovery estimates that can
        # sit somewhere other than the true signs already on /sim/track.
        #
        # Refresh the belief frame every tick: in blind mode the corridor-width
        # estimate evolves, and the lateral position of the assumed start shifts
        # with it. Recomputing the transform keeps the drawn plan over the real
        # track instead of letting it drift 10-20 cm behind the estimate.
        visualizer.set_belief_frame(*sim.belief_offset_poses)
        visualizer.publish_belief(sim.navigator)
        pacer.wait()

    return sim.run(
        on_step=on_step,
        contact_grace_s=opts.contact_grace_s if opts.recover else None,
    )


def _log_result(label: str, result: SimResult) -> None:
    status = "SUCCESS" if result.success else "FAILED"
    logger.info(
        "%s | %s | laps=%d/%d collided=%s timeout=%s dist=%.2fm t=%.1fs contacts=%d (%.1fs)",
        status,
        label,
        result.laps_completed,
        result.target_laps,
        result.collided,
        result.timed_out,
        result.distance_m,
        result.sim_time_s,
        result.contact_count,
        result.contact_time_s,
    )


def _run_and_visualize(scenario: NamedScenario, opts: _RunOptions) -> None:
    scenario_track = _track_for(scenario.metadata)
    visualizer = LiveScenarioVisualizer(scenario_track)
    _set_track(visualizer, scenario.metadata, scenario_track)
    logger.info(
        "Publishing /sim/odom, /scan, /sim/track, /sim/robot_model, /sim/plan, "
        "/sim/sign_estimates — run `task sim:navigate:rviz` in another terminal to watch "
        "(NOT `task sim:rviz`: bare RViz, no saved config, so it comes up empty). "
        "`task sim:navigate:visualize:all` does both in one command.",
    )
    result = _run_one(scenario, visualizer, opts)
    _log_result(scenario.label, result)
    visualizer.destroy_node()


def main() -> None:
    """Entry point for `python -m src.simulation.visualize_scenario`."""
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = _parse_args()
    opts = _RunOptions.from_args(args)

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
            "in another terminal to watch. %d scenarios queued.",
            len(scenarios),
        )
        try:
            for i, scenario in enumerate(scenarios):
                logger.info("--- [%d/%d] %s ---", i + 1, len(scenarios), scenario.label)
                result = _run_one(scenario, visualizer, opts)
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
        _run_and_visualize(scenario, opts)
        return

    if args.scenario is not None:
        scenario = find_scenario(args.scenario, _catalog(args.challenge))
        _run_and_visualize(scenario, opts)
        return

    widths = uniform_widths(WIDE_MM)
    widths.update(south=args.south, north=args.north, east=args.east, west=args.west)
    section = Section.from_string(args.section)
    direction = Direction.CLOCKWISE if args.direction == "cw" else Direction.COUNTERCLOCKWISE
    metadata = build_open_metadata(widths, section, direction)
    _run_and_visualize(
        NamedScenario(label="ad-hoc", metadata=metadata.model_dump(), laps=args.laps, seed=0),
        opts,
    )


if __name__ == "__main__":
    main()
