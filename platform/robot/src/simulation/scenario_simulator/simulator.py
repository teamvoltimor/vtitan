"""Closed-loop scenario simulation, run against :class:`~src.simulation.simulated_hardware_gateway.SimulatedHardwareGateway`.

Drives the *real* :class:`~src.navigation.core_navigator.CoreNavigator` --
the exact pure-pursuit controller, collision controller, stuck detector and
``LapDetector`` the car runs -- through the simulated gateway. Each control
tick mirrors the ROS2 node exactly (``platform/robot/src/ros2/navigation/node.py``):

1. ``navigator.step()`` reads pose + LIDAR, computes steering/speed, and
   publishes a ``DriveCommand`` (``speed_mps``, ``steering_norm`` in [-1, 1]).
2. The gateway integrates that command over ``dt`` and regenerates the sensors.

No Gazebo, no ROS2, no physics engine -- pure Python, runs anywhere.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
from shared.config.constants import CorridorDimensions, DictKeys, RobotSpecs, TrafficSignSpecs
from shared.domain.enums import Direction, ScenarioType, Section
from shared.domain.models import CorridorGeometry, Position2D, ScenarioMetadata, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import follow_corridor
from src.navigation.deferred_width_belief import DeferredWidthBelief
from src.navigation.maneuvers.bay_exit import BayExit
from src.navigation.ports import DriveCommand
from src.navigation.utils import _forward_clearance, _nearest_ray, clamp
from src.navigation.direction_estimator import DirectionEstimator, direction_from_parking_bay
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import (
    SignRouter,
    SignRouterConfig,
    SignSpec,
    signs_from_metadata,
)
from src.navigation.planning.waypoints import plan_believed_path
from src.navigation.race_tracker import LapDetector
from src.navigation.start_conditions import assumed_start_conditions, start_pose
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths, corridor_widths_from_metadata
from src.simulation.imu_error_model import SensorErrors
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.scenario_result import (
    TERMINAL_SURFACES,
    ContactTracker,
    PoseDisturbance,
    RunMetrics,
    SimResult,
)
from src.simulation.scenario_simulator.scoring import PassSideScorer
from src.simulation.simulated_hardware_gateway import (
    CONTROL_DT,
    LIDAR_INVALID_RAY_RATE,
    SimulatedHardwareGateway,
)
from src.simulation.track_model import ContactSurface, TrackModel, obstacles_from_metadata

if TYPE_CHECKING:
    from collections.abc import Callable

    from shared.config.navigation_tuning import NavigationTuning

    from src.navigation.ports import LidarScan


@dataclass(frozen=True, slots=True)
class _StartConditions:
    section: Section
    direction: Direction
    x: float
    y: float
    yaw: float


class ScenarioSimulator(PassSideScorer):
    """Builds and runs a closed-loop Open or Obstacles Challenge simulation from metadata.

    Sign routing and parking are wired in exactly like the real ROS2 node
    (``TrackNavigator`` in ``src/ros2/navigation/node.py``): a ``SignRouter`` is
    built from ``metadata["sign_positions"]`` and a ``ParkController`` from
    ``metadata["parking_lot"]``, both ``None`` for the Open Challenge. By default
    camera detections are simulated as always-empty (``SimulatedHardwareGateway``
    has no vision model), so sign colors resolve from scenario-metadata ground
    truth rather than a detector — the same simplification the Open Challenge
    sim already makes for pose (perfect odometry) and LIDAR (ground-truth
    raycasts). Pass ``emit_vision_detections=True`` to instead exercise the real
    camera confirmation path in ``sign_router.py`` via a synthetic emulator
    (``src/simulation/vision_emulator.py``).

    Pose comes from the ``LidarLocalizer`` estimate by default — the same
    position source the real robot uses. Pass ``use_lidar_localization=False``
    for ground-truth pose, which is still useful as a control: the difference
    between the two runs is a direct measure of what state estimation costs.

    The default was ground truth until 2026-08-01, and that flattered every
    number the sim produced. A robot handed perfect odometry is being asked an
    easier question than the one it faces on a track, and the gap only shows up
    where it is expensive to find. Defaults now match the hardware; make the
    sim easier deliberately, not by omission.

    ``blind`` is on by default too, for the same reason: a round the robot
    drives knowing the corridor widths is not the round it will actually be
    given. It withholds the start pose as well — see ``known_start`` below,
    which is the arm that hands it back. A blind run seeds the BELIEVED start
    from ``assumed_start_conditions`` (a fixed guess, always SOUTH) while the
    chassis is physically placed at the scenario's true start, exactly as
    hardware behaves. On 2026-08-01 that gap is what a full afternoon of
    on-track debugging turned out to be chasing.

    What blind does NOT withhold is error in the placement itself: the believed
    start is a fixed guess, not a *perturbed* one, so the robot is wrong in a
    known, repeatable way rather than an unpredictable one. Use
    ``sensor_errors`` to close that; see below.

    ``blind=True`` withholds the *layout*. Normally the
    corridor widths in the metadata reach the robot twice over — the planned
    path is built from them and the localizer matches scans against a wall
    model built from them — which is only honest if someone measured the mat
    first. WRO randomises the inner walls each round, so in blind mode the
    robot starts assuming every corridor is narrow (the safe prior) and
    estimates the real widths from LIDAR as it drives, replanning whenever an
    estimate changes. The metadata is then used only to build the physical
    track the robot is driving on, never to tell it anything.

    ``known_start`` is a DIAGNOSTIC arm, not a mode the hardware can run: it
    keeps blind's layout and sign-discovery handicaps but seeds the believed
    pose from the true start instead of ``assumed_start_conditions``. It exists
    to isolate one variable, because blind bundles three (layout, direction,
    start pose) and the middle arm of ``blind-source`` cannot separate them.
    Measured 2026-08-16: the believed-vs-true offset in ordinary blind is a
    STABLE rigid rotation -- exactly the section-relabelling angle (assumed
    start is always SOUTH, so a NORTH start reads 180 deg, EAST 90, WEST -90),
    holding to within 1.4 deg across a whole run. Since the track is
    4-fold symmetric and the pass-side rule is rotation-invariant, that
    rotation *should* be harmless; this arm is how to find out whether some
    part of the pipeline is nonetheless mixing the believed frame with
    absolute truth.

    ``blind`` covers what the robot knows about the *track*. ``sensor_errors``
    covers what it knows about *itself* — where it was placed and which way it
    is pointing — which the sim otherwise supplies exactly. See
    :class:`SensorErrors`; it implies ``use_lidar_localization`` for the same
    reason ``blind`` does.
    """

    def __init__(
        self,
        metadata: ScenarioMetadata | dict[str, Any],
        num_laps: int = 3,
        tuning: NavigationTuning | None = None,
        lidar_noise_std: float = RobotSpecs.LIDAR_NOISE_STDDEV,
        kinematics: AckermannKinematics | None = None,
        seed: int = 0,
        emit_vision_detections: bool = False,
        use_lidar_localization: bool = True,
        blind: bool = True,
        known_start: bool = False,
        sensor_errors: SensorErrors | None = None,
        solid_walls: bool = False,
        infer_direction: bool | None = None,
        lidar_hz: float = RobotSpecs.LIDAR_UPDATE_RATE,
        lidar_invalid_rate: float = LIDAR_INVALID_RAY_RATE,
        wall_heading: bool = True,
        park: bool = True,
        allow_sign_nudge: bool = True,
    ) -> None:
        if isinstance(metadata, dict):
            metadata = ScenarioMetadata.model_validate(metadata)
        self._metadata = metadata
        self._num_laps = num_laps
        # Blind mode implies LIDAR localization: navigating on ground-truth
        # pose while pretending not to know the layout would be incoherent.
        self._blind = blind
        # Sensor error is only observable through the estimate, so it implies
        # localization for the same reason blind mode does: on ground-truth pose
        # a mis-seeded estimator is never consulted and the run is unaffected.
        #
        # Read the polarity carefully -- the name invites the opposite reading.
        # use_lidar_localization=True is the HARDER condition (the robot works
        # out where it is); False hands it ground truth and is the easier
        # control arm. Either of blind or sensor_errors forces it True, so
        # passing False alongside them is silently ignored rather than honoured.
        self._errors = sensor_errors or SensorErrors()
        use_lidar_localization = use_lidar_localization or blind or self._errors.any_error

        true_geometry = corridor_widths_from_metadata(metadata)
        start = _start_conditions(metadata)

        # Where the robot *believes* it is standing, which is not the same thing
        # as where it is. Blind used to withhold only the layout and still hand
        # over the exact start pose and section, so the robot began every run
        # knowing precisely where it was -- a luxury the hardware does not have.
        # There it falls back to assumed_start_conditions, a fixed guess of the
        # canonical section at (1.50, 0.25); place the robot anywhere else and it
        # plans a lap from a position a metre from the truth. On 2026-08-01 that
        # was an afternoon of on-track debugging, and no simulated scenario could
        # have caught it, because none of them ever started the robot anywhere
        # but where it thought it was.
        #
        # The physical placement stays at `start`; the whole belief system moves
        # together -- plan, estimator seed, IMU zero, lap line, park controller.
        # Moving only some of them is the tempting shortcut and it is wrong: a
        # run that steers in the true frame while counting laps in the believed
        # one spends a partial lap reaching a finish line it never started at,
        # which reads as a slow robot and is really just two frames disagreeing.
        # That mistake cost a measured quarter-to-half lap per run before this
        # was made consistent.
        # Both challenges run the same tuning. An Obstacles-specific profile
        # (shorter lookahead + capped top speed) used to be applied here; it was
        # removed once re-measurement showed it changed nothing — see the note in
        # ``NavigationTuning`` where ``for_obstacles()`` used to be.
        #
        # Assigned before the blind branch below, which reads it.
        self._tuning = get_tuning(tuning)
        challenge = metadata.challenge_type
        is_open_challenge = challenge == ScenarioType.OPEN
        # Obstacles carries its own centreline bias, tuned separately from
        # Open's -- and measured HIGHER, not lower, than the geometry argues
        # for. See WaypointParams.OBSTACLES_CENTER_BIAS_M for the sweep and
        # why it is compensating for the tracker's outward drift.
        #
        self._center_bias_m = None if is_open_challenge else self._tuning.waypoints.OBSTACLES_CENTER_BIAS_M
        # Hoisted above the blind branch below, which reads the challenge too.
        #
        # The ASSUMED START uses a different bias from the PLAN, and only on
        # Obstacles. Open passes None and so takes the narrow/wide split, which
        # is what its planner now does -- a blind Open robot assumes all-narrow
        # corridors, plans them centred, and must assume a start on that same
        # centreline. Obstacles instead pins the pre-split WIDE magnitude,
        # because its scenarios are calibrated around that exact assumed pose:
        # measured 2026-08-29, the split's 0.0 and its own planning bias of
        # 0.15 each failed six scenarios that pass at 0.10. Its assumed start
        # therefore does not sit on its own planned line -- a pre-existing
        # inconsistency, left alone rather than fixed in passing, since
        # OBSTACLES_CENTER_BIAS_M is a separately swept value.
        assumed_bias_m = None if is_open_challenge else self._tuning.waypoints.WIDE_CENTER_BIAS_M
        believed_start = start
        if blind and not known_start:
            # No widths passed, so it falls back to the all-narrow prior --
            # which is exactly what the hardware does at startup, before any
            # corridor has been measured.
            assumed = assumed_start_conditions(start.direction, tuning=self._tuning, center_bias_m=assumed_bias_m)
            believed_start = _StartConditions(
                section=Section.from_string(assumed[DictKeys.SECTION]),
                direction=start.direction,
                x=float(assumed[DictKeys.POSITION][DictKeys.X]),
                y=float(assumed[DictKeys.POSITION][DictKeys.Y]),
                yaw=float(assumed[DictKeys.YAW]),
            )
        self._terminal_surfaces = TERMINAL_SURFACES[ScenarioType.OPEN if is_open_challenge else ScenarioType.OBSTACLES]
        # Traffic signs and parking blocks are real objects: the chassis can hit
        # them and the LIDAR can see them. Without them in the track model the
        # run reports success while driving straight through every sign.
        self._track = TrackModel(
            true_geometry,
            obstacles=obstacles_from_metadata(metadata.model_dump(), tuning=self._tuning),
            tuning=self._tuning,
        )

        # ``None`` restores the old all-or-nothing scoring, where any contact
        # with a pillar ends the run. Kept switchable because every figure
        # recorded before 2026-08-01 was measured that way, and comparing
        # against them needs the same rule.
        self._max_sign_push: float | None = TrafficSignSpecs.MAX_LEGAL_DISPLACEMENT_M if allow_sign_nudge else None
        self._sign_push: dict[int, float] = {}
        self._prev_contact_xy: Waypoint = Waypoint(start.x, start.y)
        self._true_geometry = true_geometry

        # What the robot is allowed to believe about the layout. Sighted runs
        # get the truth (as the ROS2 node does, from its metadata file); blind
        # runs start from the prior their challenge allows and correct it from
        # LIDAR as they go.
        #
        # The Obstacles Challenge fixes every corridor at 1.0 m, so that is
        # prior knowledge, not a guess -- confirmed across the fixture set,
        # where all 64 corridors are wide. The Open Challenge's are
        # independently 60 or 100 cm, so it keeps the narrow (fail-safe) prior.
        self._arc_radius = self._tuning.waypoints.ARC_RADIUS
        self._width_estimator = (
            CorridorWidthEstimator(
                assumed_width=CorridorDimensions.NARROW if is_open_challenge else CorridorDimensions.OBSTACLES_WIDTH,
                tuning=self._tuning,
                # Obstacles corridors are 1.0 m by rule, not by discovery -- a
                # sign/pillar hugging a wall can otherwise feed the voting a
                # run of falsely-narrow readings with nothing to correct it
                # back. See CorridorWidthEstimator's own docstring.
                fixed=not is_open_challenge,
            )
            if blind
            else None
        )
        # OPEN CHALLENGE ONLY, and None elsewhere rather than merely disabled.
        # The gate replans whenever confirmed-ness moves, not only when a width
        # does -- which is the point on Open, where confirming a corridor
        # releases UNCONFIRMED_WIDTH_INNER_BIAS_M. On Obstacles the estimator is
        # fixed=True and the bias comes from an explicit override, so that same
        # trigger would rebuild a byte-identical path and re-seek the waypoint
        # index for nothing. See DeferredWidthBelief.
        self._width_gate = (
            DeferredWidthBelief(enabled=self._tuning.waypoints.DEFER_CURRENT_CORRIDOR_REPLAN)
            if is_open_challenge
            else None
        )
        # Travel direction is inferred from LIDAR too when asked. Until it
        # settles there is no usable plan -- the path for the wrong direction
        # runs the opposite way down this same corridor -- so the robot follows
        # the corridor reactively and only then plans.
        # Blind implies inferring the direction. The round's direction is drawn
        # at random on the day, so a "blind" run that is handed it is not blind
        # -- it measures a robot with information no robot has. Pass
        # ``infer_direction=False`` explicitly only to keep the told-direction
        # control condition for comparison.
        if infer_direction is None:
            infer_direction = blind
        self._direction_estimator = DirectionEstimator(tuning=self._tuning) if infer_direction else None
        # Was the robot PLACED inside the parking bay? Answered once, on the
        # first scan, and never revisited -- see _resolve_direction.
        self._bay_start_checked = False
        self._exiting_bay = False
        self._bay_exit_ticks = 0
        self._bay_exit = BayExit()
        # Speed for the blind corridor-follow that runs before the travel
        # direction settles. Named _creep_speed until 2026-08-09, which was
        # doubly misleading: it is not the creep tier, and it never was --
        # it read the slow tier. The medium tier is the closest match to the
        # 0.150 m/s this phase actually ran at, so keeping it here avoids
        # slowing every race start as a side effect of grading the ladder.
        self._blind_follow_speed = self._tuning.speed.medium_mps()
        self._creep_widths: list[tuple[float, float]] = []
        """(yaw, measured width) taken before the direction was known."""
        self._start = start
        self._believed_start = believed_start
        # Provisional until inference settles. Everything built from it -- the
        # path and the lap detector's finish-line normal -- is rebuilt then.
        self._direction = start.direction
        believed_geometry = (
            CorridorGeometry.from_width_dict(self._width_estimator.widths) if self._width_estimator else true_geometry
        )

        # Mirror node.py: a single canonical lap, repeated num_laps times by the
        # navigator's waypoint-wrap + LapDetector lap counting.
        self._waypoints = self._plan(believed_geometry)

        signs: list[SignSpec] = [] if is_open_challenge else signs_from_metadata(metadata.model_dump())

        # Ground truth for the pass-side rule, kept by the SIMULATOR rather than
        # read back off the navigator. Scoring a rule from the robot's own
        # belief lets better self-deception pass for better driving: measured
        # 2026-08-24, the router's believed-frame verdict flagged 46% of passes
        # where the true layout says 21%, and ended 45 of 64 runs where 38
        # genuinely offended. A judge watches the mat, so this does too.
        self._true_signs = signs
        self._pass_side_closest: dict[int, tuple[float, Waypoint]] = {}
        self._pass_side_engaged: set[int] = set()
        self._pass_side_scored: set[int] = set()
        self._pass_side_wrong: list[int] = []

        # Where the signs are is drawn at random on the day and no scenario file
        # exists on the mat, so a blind run cannot be handed the sign layout any
        # more than it can be handed the corridor widths. The router discovers
        # them from the camera instead (see sign_discovery). ``signs`` still
        # reaches the *gateway*, because that is the emulated camera's own view
        # of the world -- the sensor's ground truth, not the navigator's
        # knowledge -- and a mocked vision node with nothing to project would
        # make blind mode measure a robot with no perception at all rather than
        # one that has to use it.
        discover_signs = blind and not is_open_challenge
        emit_vision_detections = emit_vision_detections or discover_signs

        self._gateway = SimulatedHardwareGateway(
            track=self._track,
            initial_state=AckermannState(x=start.x, y=start.y, yaw=start.yaw),
            believed_start=AckermannState(
                x=believed_start.x,
                y=believed_start.y,
                yaw=believed_start.yaw,
            ),
            kinematics=kinematics,
            lidar_noise_std=lidar_noise_std,
            rng=np.random.default_rng(seed),
            signs=signs if emit_vision_detections else None,
            localize=use_lidar_localization,
            sensor_errors=self._errors,
            solid_walls=solid_walls,
            # A surface that no longer ends the run has to stop the chassis
            # instead, or the robot simply drives through the inner block and
            # goes on counting laps. Nothing enforced this before because every
            # contact was terminal on the tick it happened, so a pass-through
            # could never be observed.
            solid_surfaces=frozenset(ContactSurface) - {ContactSurface.NONE} - self._terminal_surfaces,
            lidar_hz=lidar_hz,
            lidar_invalid_rate=lidar_invalid_rate,
            wall_heading=wall_heading,
        )
        if blind:
            if self._width_estimator:
                self._gateway.set_believed_walls(TrackWalls(believed_geometry))
            else:
                self._gateway.set_believed_walls(TrackWalls(true_geometry))

        # believed_start, not start: the lap line is part of the robot's plan,
        # so it belongs in the frame the robot thinks it is driving in.
        lap_detector = LapDetector(
            start_pos=Waypoint(believed_start.x, believed_start.y),
            start_section=believed_start.section,
            direction=believed_start.direction,
        )

        sign_router: SignRouter | None = None
        if signs:
            sign_router = SignRouter(
                [] if discover_signs else signs,
                config=SignRouterConfig.from_tuning(self._tuning.sign_router),
                direction=start.direction,
                discover=discover_signs,
                discovery_config=self._tuning.sign_discovery,
                tuning=self._tuning,
            )

        # ``park=False`` runs an Obstacles scenario as laps-only: the signs, the
        # parking blocks and their collision geometry all stay on the mat, but
        # no parking maneuver is attempted and the run is scored purely on
        # completing its laps without a collision. Sign avoidance is the open
        # problem and parking is a separate one downstream of it; with the
        # controller engaged, a run that drove three clean laps still ends in a
        # ParkController give-up, which buries the signal being measured.
        self._park_controller: ParkController | None = None
        if not is_open_challenge and park:
            self._park_controller = park_controller_from_metadata(
                metadata.model_dump(),
                believed_start.section,
                believed_start.direction,
                tuning=self._tuning,
            )

        self._navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=self._waypoints,
            num_laps=num_laps,
            tuning=self._tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=self._park_controller,
            direction=self._direction,
        )

    def _plan(self, geometry: CorridorGeometry, unconfirmed: frozenset[Section] | None = None) -> list[Waypoint]:
        """Build a one-lap path for the layout the robot believes it is on.

        The believed start, not the true one: a path is built from where the
        robot thinks it is, and on hardware that is the assumed pose. Planning
        from the true start while the estimator runs in the believed frame
        would hand the robot a route to a place it does not think it is.
        """
        believed = self._believed_start
        return plan_believed_path(
            self._metadata,
            geometry,
            direction=self._direction,
            believed_section=believed.section,
            believed_position=Position2D(x=believed.x, y=believed.y),
            believed_yaw=believed.yaw,
            arc_radius=self._arc_radius,
            tuning=self._tuning,
            center_bias_m=self._center_bias_m,
            # Taken from the width gate when it has an opinion, so the bias and
            # the width it belongs to are always the same vintage -- see
            # DeferredWidthBelief on why gating one without the other leaks a
            # 0.05 m step. Empty when sighted: no estimator means the widths
            # were told rather than discovered, and a told width is confirmed by
            # definition. See WaypointParams.UNCONFIRMED_WIDTH_INNER_BIAS_M.
            unconfirmed_sections=(
                unconfirmed
                if unconfirmed is not None
                else (
                    frozenset(Section) - self._width_estimator.observed_sections
                    if self._width_estimator is not None
                    else frozenset()
                )
            ),
        )

    def _bay_exit_command(self, scan: LidarScan) -> DriveCommand:
        """Delegate to the navigation-layer bay exit.

        The manoeuvre itself lived here until 2026-08-31, which meant the
        simulator drove the pocket exit while the real robot had no bay-exit
        path at all. It now lives in :mod:`src.navigation.maneuvers.bay_exit`
        and both this and the ROS2 node call it, so an in-bay simulation
        exercises the code the robot runs.
        """
        return self._bay_exit.command(
            scan.ranges_m,
            scan.angles_rad,
            self._gateway.get_wheel_odometry().distance_m,
            self._blind_follow_speed,
            self._tuning,
        )

    def _resolve_direction(self) -> bool:
        """Creep along the corridor until the travel direction is inferable.

        Returns:
            ``True`` while the direction is still unknown, meaning the caller
            drove the corridor follower this tick instead of the navigator.
        """
        estimator = self._direction_estimator
        if estimator is None:
            return False

        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return True if not estimator.is_settled else False

        # Settle the direction EARLY, hand over control LATE. Boxed in the
        # parking bay the geometry names the direction outright, but the
        # planner's path runs from the BELIEVED start -- the assumed centreline,
        # not the pocket -- so handing over while still boxed drives straight
        # into a marker. Measured: settling without this guard took the in-bay
        # probe from 0.33-14.06 m back down to 0.18 m, every run collided.
        #
        # So keep the corridor follower driving until the robot is actually out,
        # and let the planner take over only once forward is clear. Checked
        # before `is_settled` on purpose: the normal guard returns as soon as a
        # direction exists, which is exactly the handover being deferred here.
        # Evaluated ONCE, on the first scan, because it is a claim about where
        # the robot was PLACED. Re-testing it every tick lets it fire mid-creep
        # at a corner -- forward blocked, one side close, the other open reads
        # the same -- and settle the direction off geometry that is not a bay at
        # all. Measured: that changed parallel-start runs that must be
        # untouched, one going 22.40 m -> 3.42 m.
        if not self._bay_start_checked:
            self._bay_start_checked = True
            bay = direction_from_parking_bay(scan.ranges_m, scan.angles_rad, self._tuning)
            if bay is not None:
                estimator.settle(bay)
                self._exiting_bay = True

        just_exited = False
        # A budget expiry releases the maneuver on exactly the same path as a
        # clean exit, so the fall-through below still rebuilds the plan. Without
        # a budget `is_clear` is the ONLY release, and it gates on forward
        # clearance the pocket cannot provide -- measured: 600/600 ticks held,
        # `CoreNavigator` never stepped once, so no escape behaviour was ever
        # reachable from an in-bay start.
        budget = self._tuning.corridor_follower.BAY_EXIT_MAX_FRAMES
        if self._exiting_bay:
            self._bay_exit_ticks += 1
        bay_exit_spent = bool(budget) and self._bay_exit_ticks > budget
        if self._exiting_bay and (bay_exit_spent or BayExit.is_clear(scan.ranges_m, scan.angles_rad, self._tuning)):
            # Out of the pocket. Fall THROUGH to the settle block rather than
            # returning: that block is what rebuilds the path for the committed
            # direction and calls replace_path, and skipping it hands the
            # planner a stale plan still pointing at waypoint 0 while the robot
            # has driven out of the bay. Measured: it drove straight back into a
            # marker, 0.24-0.30 m every run.
            self._exiting_bay = False
            just_exited = True

        if self._exiting_bay:
            self._gateway.publish_drive(self._bay_exit_command(scan))
            return True

        if estimator.is_settled and not just_exited:
            return False

        # Take width readings during the creep as well. They cannot be filed
        # under a corridor yet -- that needs the direction -- but they are the
        # cleanest readings of the whole round, taken driving straight down a
        # corridor. Discarding them leaves the first surviving readings to be
        # taken at a corner, where the side rays span the next corridor and get
        # attributed to this one. Measured: that alone mislearned the starting
        # corridor on fixtures whose direction was inferred perfectly.
        if self._width_estimator is not None:
            m = measure_corridor_width(scan.ranges_m, scan.angles_rad, pose.yaw)
            if m is not None:
                self._creep_widths.append((pose.yaw, m.width_m))

        # Reached only once clear of the bay, so the bay case is already settled
        # above and this is the ordinary vote-based path.
        if just_exited or estimator.observe(scan.ranges_m, scan.angles_rad, pose.yaw, self._tuning):
            inferred = estimator.direction
            if inferred is not None and inferred is not self._direction:
                # The path runs the other way round the loop and the finish
                # line's normal is inverted, so both are rebuilt.
                self._direction = inferred
                self._waypoints = self._plan(
                    CorridorGeometry.from_width_dict(self._width_estimator.widths)
                    if self._width_estimator
                    else self._true_geometry,
                )
                self._navigator.replace_lap_detector(
                    LapDetector(
                        # Believed, not true: the real node has no ground truth to
                        # leak here at all, only its belief, and the finish line
                        # lives in whatever frame the rest of the plan is in.
                        start_pos=Waypoint(self._believed_start.x, self._believed_start.y),
                        start_section=self._believed_start.section,
                        direction=inferred,
                    ),
                )
                self._navigator.set_travel_direction(inferred)
            # Replay the buffered widths now that they can be attributed.
            if self._width_estimator is not None and inferred is not None:
                for buffered_yaw, buffered_width in self._creep_widths:
                    self._width_estimator.observe_measurement(
                        section_from_heading(buffered_yaw, inferred),
                        buffered_width,
                    )
                self._creep_widths.clear()
                self._waypoints = self._plan(
                    CorridorGeometry.from_width_dict(self._width_estimator.widths),
                )
                self._gateway.set_believed_walls(
                    TrackWalls(corridor_geometry_from_widths(self._width_estimator.widths))
                )

            # Resync unconditionally, including when the inference agreed with
            # the provisional direction and the path is unchanged. The
            # navigator did not step during the creep, so its waypoint index is
            # still 0 while the robot has driven a metre past it -- it would
            # resume by chasing a waypoint behind itself. Measured: this alone
            # cost fixtures that had inferred the direction perfectly.
            self._navigator.replace_path(self._waypoints, (pose.x, pose.y), pose.yaw)
            return False

        self._gateway.publish_drive(
            follow_corridor(
                scan.ranges_m,
                scan.angles_rad,
                self._blind_follow_speed,
                pose.yaw,
                self._tuning,
                believed_width_m=statistics.fmean(w for _, w in self._creep_widths) if self._creep_widths else None,
            ),
        )
        return True

    def _creep_telemetry(
        self,
        prev_xy: Waypoint,
        on_step: Callable[[AckermannState, LidarScan], None] | None,
    ) -> float:
        """Publish and measure a creep tick; return the distance it covered."""
        scan = self._gateway.get_lidar_scan()
        if on_step is not None and scan is not None:
            on_step(self._gateway.state, scan)
        state = self._gateway.state
        return math.hypot(state.x - prev_xy.x, state.y - prev_xy.y)

    def _update_layout_belief(self) -> bool:
        """Fold the latest scan into the width estimate; replan if it moved.

        Returns:
            ``True`` if the belief changed and the path was rebuilt.
        """
        estimator = self._width_estimator
        if estimator is None:
            return False
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return False
        # Attribute the reading by HEADING, not position. Position would be
        # circular — it comes from matching against a wall model built from the
        # very widths being estimated, so a wrong belief mis-attributes the
        # reading that would have corrected it and the error locks in (measured:
        # 8 of 28 fixtures learned a wrong layout and drove into a wall, all
        # with ~40 cm of position error). Heading comes from the IMU and owes
        # nothing to the map.
        section = section_from_heading(pose.yaw, self._direction)
        observed_change = estimator.observe(section, scan.ranges_m, scan.angles_rad, pose.yaw)
        if self._width_gate is None:
            # Not the Open Challenge -- keep the pre-gate semantics exactly.
            if not observed_change:
                return False
            widths, unconfirmed = estimator.widths, None
        else:
            # Gated every tick rather than only when observe() reports a change:
            # a belief held back is released by the robot LEAVING the corridor,
            # not by a new reading, so the tick that finally applies it is
            # usually one the estimator had nothing to say about.
            widths, unconfirmed, changed = self._width_gate.update(
                estimator.widths, estimator.observed_sections, section
            )
            if not changed:
                return False

        believed = CorridorGeometry.from_width_dict(widths)
        self._waypoints = self._plan(believed, unconfirmed)
        self._gateway.set_believed_walls(TrackWalls(believed))
        self._navigator.replace_path(self._waypoints, (pose.x, pose.y))
        return True

    @property
    def believed_widths(self) -> dict[Section, float] | None:
        """What the robot currently thinks the layout is, or ``None`` if told."""
        return self._width_estimator.widths if self._width_estimator else None

    @property
    def belief_offset_poses(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """``(believed_start, true_start)`` as ``(x, y, yaw)``, for visualization.

        These two differ only in a blind run, where the believed start is
        ``assumed_start_conditions``' fixed SOUTH guess and the chassis is
        placed at the scenario's true start. Everything the navigator plans is
        expressed against the first; the track it is actually driving is the
        second. Exposed so a viewer can show one over the other -- see
        ``LiveScenarioVisualizer.set_belief_frame`` -- rather than drawing the
        plan on a track it does not correspond to.

        The poses used here are the **corridor-centreline** poses, not the
        starting-zone poses stored in ``_believed_start`` / ``_start``. The
        planned path is a centreline racing line; aligning the frame by the
        starting-zone position instead shoves the whole drawn path into the
        starting zone, which is why the robot looked like it was driving
        alongside its own plan even though it was tracking the true centreline
        accurately.

        In blind mode the corridor-width estimate evolves, so the lateral
        position of the believed centreline shifts and the frame has to be
        recomputed from the current belief.

        Equal in a sighted run, which makes the offset identity.
        """
        believed_widths = (
            self._width_estimator.widths if self._width_estimator else self._true_geometry.to_widths_dict()
        )
        bx, by, byaw = _centreline_pose(
            self._believed_start.section, self._believed_start.direction, believed_widths, self._tuning
        )
        believed = _StartConditions(
            section=self._believed_start.section,
            direction=self._believed_start.direction,
            x=bx,
            y=by,
            yaw=byaw,
        )
        tx, ty, tyaw = _centreline_pose(
            self._start.section, self._start.direction, self._true_geometry.to_widths_dict(), self._tuning
        )
        true = _StartConditions(
            section=self._start.section,
            direction=self._start.direction,
            x=tx,
            y=ty,
            yaw=tyaw,
        )
        return (
            (believed.x, believed.y, believed.yaw),
            (true.x, true.y, true.yaw),
        )

    @property
    def bay_exit_ticks(self) -> int:
        """Control ticks the bay-exit manoeuvre held before handing over.

        Stops rising at the handover, so this is how long the pocket exit was
        driven -- not how long the run lasted. Exposed because the release
        gate (``BayExit.is_clear``, forward clearance) can be satisfied by a
        nose that has merely rotated toward open space while the chassis is
        still inside the pocket, and a run that hands over there looks from
        the outside like one that escaped and then failed.
        """
        return self._bay_exit_ticks

    @property
    def bay_exit(self) -> BayExit:
        """The bay-exit manoeuvre this scenario drives, for diagnostics."""
        return self._bay_exit

    @property
    def track(self) -> TrackModel:
        """The track geometry model for this scenario."""
        return self._track

    @property
    def navigator(self) -> CoreNavigator:
        """The navigator this scenario drives, for diagnostics to inspect."""
        return self._navigator

    @property
    def direction_estimator(self) -> DirectionEstimator | None:
        """The travel-direction estimator, or ``None`` when told the direction."""
        return self._direction_estimator

    @property
    def gateway(self) -> SimulatedHardwareGateway:
        """The simulated hardware this scenario drives."""
        return self._gateway

    @property
    def waypoints(self) -> list[Waypoint]:
        """The single-lap canonical waypoint path fed to the navigator."""
        return self._waypoints

    @property
    def routed_signs(self) -> list[SignSpec]:
        """Signs the router is currently routing around.

        In a blind run this starts empty and fills in as the camera finds them,
        so it is the counterpart of :attr:`believed_widths`: what the robot has
        worked out for itself, not what the scenario file says.
        """
        router = self._navigator.sign_router
        return router.signs if router else []

    def run(
        self,
        max_steps: int = 4000,
        dt: float = CONTROL_DT,
        on_step: Callable[[AckermannState, LidarScan], None] | None = None,
        disturb_at_step: int | None = None,
        disturbance: PoseDisturbance | None = None,
        start_collision_window_s: float | None = None,
        start_collision_grace_s: float | None = None,
        contact_grace_s: float | None = None,
    ) -> SimResult:
        """Run the control loop until all laps finish, a wall is hit, or timeout.

        Args:
            max_steps: Safety budget on control ticks (4000 ≈ 200 s at 20 Hz).
            dt: Control interval (seconds).
            on_step: Optional callback invoked with ``(state, lidar_scan)`` after
                every tick — used by the live Gazebo/RViz visualizer to publish
                the ground-truth pose and LIDAR sweep. ``None`` in the headless
                test battery, so it costs nothing there beyond one attribute check.
            disturb_at_step: If set together with ``disturbance``, the control
                tick at which to apply a one-time pose kick — for testing the
                navigator's ability to recover from drift.
            disturbance: The pose kick to apply at ``disturb_at_step``.
            start_collision_window_s: A collision streak that *begins* within
                this many seconds of the run's start is judged under the grace
                policy below, rather than failing the run immediately — an
                official starting zone can legally place the chassis right at
                a wall, and the robot needs a moment to react. A streak that
                begins later (a real driving mistake, not a starting
                position) still fails immediately, same as before.
                Defaults to tuning.simulation.START_COLLISION_WINDOW_S.
            start_collision_grace_s: How long a start-window collision streak
                may continue before it's judged a real, terminal failure
                rather than "still working on steering clear."
                Defaults to tuning.simulation.START_COLLISION_GRACE_S.
            contact_grace_s: Opt in to treating wall contact as *recoverable*
                anywhere in the run, not only at the start. A streak then ends
                the run only if the robot fails to free itself within this many
                seconds; brief contact it escapes from is recorded in
                ``contact_count`` instead. Touching a wall does not end a real
                round, and the navigator already has a reversing escape, but
                the default policy breaks the loop on the first contacting tick
                so that escape can never be observed. Only meaningful with
                ``solid_walls`` — otherwise the chassis passes through the wall
                and "recovery" measures nothing. ``None`` keeps the strict
                policy, under which every prior pass rate was measured.

        Returns:
            A populated :class:`SimResult`.
        """
        gw = self._gateway
        nav = self._navigator

        # Load defaults from tuning
        if start_collision_window_s is None:
            start_collision_window_s = self._tuning.simulation.START_COLLISION_WINDOW_S
        if start_collision_grace_s is None:
            start_collision_grace_s = self._tuning.simulation.START_COLLISION_GRACE_S

        prev_xy = Waypoint(gw.state.x, gw.state.y)
        metrics = RunMetrics()
        prev_laps = 0
        lap_steps: list[int] = []
        terminal_collision = False
        stuck = False
        pass_side_violation = False
        violation_signs: list[int] | None = []
        contacts = ContactTracker(
            dt=dt,
            start_window_s=start_collision_window_s,
            start_grace_s=start_collision_grace_s,
            grace_s=contact_grace_s,
            forbidden=self._terminal_surfaces,
        )

        # No-progress bailout: a run that has netted less than
        # NO_PROGRESS_DISPLACEMENT_M of straight-line travel from this anchor
        # for NO_PROGRESS_WINDOW_S is never going to finish either, so there
        # is nothing left for the remaining budget to prove. The anchor
        # resets on any real net travel, so an active (even if slow) escape
        # maneuver is never mistaken for a permanently wedged chassis.
        progress_anchor_xy = prev_xy
        progress_anchor_step = 0
        no_progress_window_steps = round(self._tuning.simulation.NO_PROGRESS_WINDOW_S / dt)
        no_progress_displacement_m = self._tuning.simulation.NO_PROGRESS_DISPLACEMENT_M

        def _no_progress(current_step: int, x: float, y: float) -> bool:
            nonlocal progress_anchor_xy, progress_anchor_step
            if math.hypot(x - progress_anchor_xy.x, y - progress_anchor_xy.y) >= no_progress_displacement_m:
                progress_anchor_xy = Waypoint(x, y)
                progress_anchor_step = current_step
                return False
            return (current_step - progress_anchor_step) >= no_progress_window_steps

        step = 0
        while step < max_steps:
            if self._resolve_direction():
                # Direction still unknown: the corridor follower published this
                # tick's command, and there is no usable plan to step yet. The
                # tick is otherwise accounted for exactly like a driving one --
                # telemetry, distance and contact all still apply, and skipping
                # them hides the creep from the visualizer and every diagnostic.
                gw.advance(dt)
                step += 1
                metrics.observe(gw, self._creep_telemetry(prev_xy, on_step))
                prev_xy = Waypoint(gw.state.x, gw.state.y)
                if contacts.update(step, gw.contact_surface if (gw.collided or gw.blocked) else ContactSurface.NONE):
                    terminal_collision = True
                    break
                if _no_progress(step, gw.state.x, gw.state.y):
                    stuck = True
                    break
                continue
            if self._blind:
                self._update_layout_belief()
            nav.step()
            gw.advance(dt)
            step += 1

            if disturbance is not None and step == disturb_at_step:
                gw.apply_disturbance(disturbance.lateral_m, disturbance.heading_rad)

            scan = gw.get_lidar_scan()
            if on_step is not None and scan is not None:
                on_step(gw.state, scan)

            sx, sy = gw.state.x, gw.state.y
            metrics.observe(gw, math.hypot(sx - prev_xy.x, sy - prev_xy.y))
            prev_xy = Waypoint(sx, sy)

            if nav.laps_completed > prev_laps:
                lap_steps.append(step)
                prev_laps = nav.laps_completed
                # Each lap passes every sign again and is judged on its own, so
                # a sign cleared correctly on lap 1 must still be scored on lap
                # 2. Mirrors SignRouter.reset_for_new_lap.
                self._pass_side_closest.clear()
                self._pass_side_engaged.clear()
                self._pass_side_scored.clear()

            surface = gw.contact_surface if (gw.collided or gw.blocked) else ContactSurface.NONE
            surface = self._score_obstacle_contact(surface, gw.state)
            if contacts.update(step, surface):
                terminal_collision = True
                break

            # Pass-side rule (Obstacles Challenge): a red obstacle must be
            # cleared OUTWARD and a green INWARD. Scored from the true layout
            # against the true pose; that is a scored failure, enforced exactly
            # like a forbidden wall contact — the run stops here.
            violation_signs = self._check_pass_side_violation(gw.state)
            if violation_signs is not None:
                pass_side_violation = True
                break

            if nav.laps_completed >= self._num_laps and (
                self._park_controller is None or self._park_controller.is_done
            ):
                break

            if _no_progress(step, sx, sy):
                stuck = True
                break

        return self._build_result(
            step=step,
            dt=dt,
            max_steps=max_steps,
            collided=terminal_collision,
            stuck=stuck,
            contacts=contacts,
            metrics=metrics,
            lap_steps=lap_steps,
            pass_side_violation=pass_side_violation,
            violation_signs=violation_signs,
        )

    def _build_result(
        self,
        *,
        step: int,
        dt: float,
        max_steps: int,
        collided: bool,
        stuck: bool,
        contacts: ContactTracker,
        metrics: RunMetrics,
        lap_steps: list[int],
        pass_side_violation: bool = False,
        violation_signs: list[int] | None = None,
    ) -> SimResult:
        """Assemble the run outcome from the loop's accumulators."""
        gw = self._gateway
        nav = self._navigator
        pc = self._park_controller
        parked = None if pc is None else (pc.is_done and not pc.is_timed_out)
        timed_out = step >= max_steps and (nav.laps_completed < self._num_laps or (pc is not None and not pc.is_done))
        return SimResult(
            target_laps=self._num_laps,
            laps_completed=nav.laps_completed,
            collided=collided,
            timed_out=timed_out,
            steps=step,
            sim_time_s=step * dt,
            distance_m=metrics.distance,
            max_speed_mps=metrics.max_speed,
            avg_speed_mps=metrics.avg_speed(step),
            min_lidar_range_m=metrics.min_range_or_zero(),
            collision_xy=gw.collision_xy,
            contact_count=contacts.count,
            contact_time_s=contacts.time_s,
            terminal_surface=contacts.surface,
            parked=parked,
            final_pose=(gw.state.x, gw.state.y, gw.state.yaw),
            lap_step_indices=lap_steps,
            stuck=stuck,
            pass_side_violation=pass_side_violation,
            pass_side_violation_signs=violation_signs or [],
        )


def _start_conditions(metadata: ScenarioMetadata) -> _StartConditions:
    sc = metadata.starting_conditions
    if sc.direction is None:
        msg = (
            "scenario metadata.starting_conditions must have a resolved direction; "
            "the sim always knows its own ground truth, so a None here means the scenario "
            "was built without it"
        )
        raise ValueError(msg)
    # Whole-number JSON metadata values parse as Python int, not float. Coerce here so a
    # downstream int never reaches a ROS message field, where CDR serialization would
    # corrupt it (bit-reinterpreted as float64 instead of converted — see live_visualizer's
    # _sign_marker for the same class of bug with sign/parking coordinates).
    return _StartConditions(
        section=sc.section,
        direction=sc.direction,
        x=float(sc.position.x),
        y=float(sc.position.y),
        yaw=float(sc.yaw),
    )


def _centreline_pose(
    section: Section,
    direction: Direction,
    widths_m: dict[Section, float],
    tuning: NavigationTuning | None,
) -> tuple[float, float, float]:
    """Centreline pose for a section/direction/width set.

    ``start_pose`` returns the biased corridor centreline, which is what the
    planned path is built from. Using it for the visualization frame aligns the
    drawn plan with the track centreline rather than with the starting-zone
    cell the robot happens to be placed in.
    """
    by_name = {s.value.lower(): w for s, w in widths_m.items()}
    return start_pose(section, direction, by_name, tuning)
