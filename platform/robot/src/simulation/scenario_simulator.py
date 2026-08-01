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
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np
from shared.config.constants import CompetitionSpecs, CorridorDimensions, RobotSpecs
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import ScenarioMetadata

from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import follow_corridor
from src.navigation.direction_estimator import DirectionEstimator
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import LidarScan
from src.navigation.race_tracker import LapDetector
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths, corridor_widths_from_metadata
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.simulated_hardware_gateway import (
    CONTROL_DT,
    LIDAR_INVALID_RAY_RATE,
    LIDAR_SCAN_HZ,
    SensorErrors,
    SimulatedHardwareGateway,
)
from src.simulation.track_model import ContactSurface, TrackModel, obstacles_from_metadata

if TYPE_CHECKING:
    from collections.abc import Callable

START_COLLISION_WINDOW_S = 2.0
"""A collision streak beginning within this long of run start is judged as a
starting-position issue (see ``ScenarioSimulator.run``), not a driving mistake."""

START_COLLISION_GRACE_S = 15.0
"""How long a starting-position collision streak may continue before it's a real failure."""

TERMINAL_SURFACES: dict[ScenarioType, frozenset[ContactSurface]] = {
    ScenarioType.OPEN: frozenset({ContactSurface.OUTER_WALL}),
    ScenarioType.OBSTACLES: frozenset({ContactSurface.INNER_WALL, ContactSurface.OBSTACLE}),
}
"""Which contacts end a run, per challenge.

Each challenge forbids one wall: the Open Challenge the *outer* one, the
Obstacles Challenge the *inner* one. Contact with the other wall is still
recorded in ``SimResult.contact_count`` but does not end the run, so a scrape
the robot drives out of no longer scores the same as failing to complete.

Obstacles are grouped with the inner wall rather than given a rule of their
own: knocking a traffic sign or parking block over is a scored failure and only
exists in the Obstacles Challenge. The user's rule covers walls, so this half
is an assumption -- if a sign is meant to be a survivable penalty instead, this
is the one line to change.
"""


@dataclass(slots=True)
class SimResult:
    """Outcome of one closed-loop scenario run."""

    target_laps: int
    laps_completed: int
    collided: bool
    timed_out: bool
    steps: int
    sim_time_s: float
    distance_m: float
    max_speed_mps: float
    avg_speed_mps: float
    min_lidar_range_m: float
    collision_xy: tuple[float, float] | None
    final_pose: tuple[float, float, float]
    contact_count: int = 0
    """Distinct wall-contact episodes, whether or not any was terminal.

    Always recorded, so a run that recovers from contact is not scored as if
    it never touched anything — under ``contact_grace_s`` this is the number
    that stands in for a penalty."""

    contact_time_s: float = 0.0
    """Total time spent in contact with a run-ending surface."""

    terminal_surface: ContactSurface = ContactSurface.NONE
    """Which surface ended the run, or ``NONE`` if contact did not end it."""

    lap_step_indices: list[int] = field(default_factory=list)
    parked: bool | None = None
    """``None`` when the scenario has no parking lot; else whether parking finished cleanly
    (as opposed to giving up on its frame budget — see ``ParkController.is_timed_out``)."""

    @property
    def over_time(self) -> bool:
        """Exceeded the official 3-minute round limit.

        Derived from ``sim_time_s`` rather than stored, so it cannot drift from
        the time actually simulated. Distinct from ``timed_out``, which only
        says the run hit the harness's ``max_steps`` budget -- that budget is
        200 s, more generous than the rule, so a run could finish its laps at
        196 s and be scored a clean pass for something the judges would not
        have let finish.
        """
        return self.sim_time_s > CompetitionSpecs.ROUND_TIME_LIMIT_S

    @property
    def success(self) -> bool:
        """Completed all target laps in time, without a wall contact (and parked cleanly, if required)."""
        return (
            self.laps_completed >= self.target_laps
            and not self.collided
            and not self.over_time
            and self.parked is not False
        )


@dataclass(frozen=True, slots=True)
class PoseDisturbance:
    """A one-time pose kick applied mid-run, e.g. to test recovery from drift.

    ``lateral_m`` offsets perpendicular to the current heading (positive =
    left of travel direction); ``heading_rad`` adds to yaw.
    """

    lateral_m: float
    heading_rad: float = 0.0


@dataclass(slots=True)
class _RunMetrics:
    """Per-tick accumulators for the run's distance, speed and clearance.

    One object rather than four locals because the control loop accounts for a
    tick in two places -- once for a creep tick taken before the travel
    direction is known, once for a normal driving tick -- and those had drifted
    apart. The creep branch fed distance and the step count but not speed, so
    ``avg_speed`` (which divides by the full step count) was understated on
    every blind run, and a run that collided before the direction settled
    reported ``vmax=vavg=0.00`` alongside a non-zero distance for the same
    ticks. Both branches now call :meth:`observe`, so a metric cannot be added
    to one and forgotten in the other.
    """

    distance: float = 0.0
    max_speed: float = 0.0
    speed_sum: float = 0.0
    min_range: float = math.inf

    def observe(self, gateway: SimulatedHardwareGateway, distance_increment: float) -> None:
        """Fold one tick of motion into the accumulators."""
        self.distance += distance_increment
        speed = abs(gateway.state.v)
        self.max_speed = max(self.max_speed, speed)
        self.speed_sum += speed
        self.min_range = min(self.min_range, gateway.last_min_range)

    def avg_speed(self, steps: int) -> float:
        """Mean speed over every tick of the run, creep included."""
        return (self.speed_sum / steps) if steps else 0.0

    def min_range_or_zero(self) -> float:
        """Closest LIDAR return seen, or 0.0 if no scan ever reported one."""
        return self.min_range if math.isfinite(self.min_range) else 0.0


class _ContactTracker:
    """Decides when a wall-contact streak stops being survivable and ends the run.

    Two policies. By default contact is terminal on the first tick, except
    within the opening seconds, where a legal starting position may already sit
    against a wall and the robot is allowed a grace period to steer clear.
    Setting ``grace_s`` switches to treating contact as recoverable everywhere:
    a streak ends the run only if the robot cannot free itself in time, which
    is the only policy under which the navigator's reversing escape is
    observable at all.
    """

    def __init__(
        self,
        dt: float,
        start_window_s: float,
        start_grace_s: float,
        grace_s: float | None,
        forbidden: frozenset[ContactSurface],
    ) -> None:
        self._dt = dt
        self._start_window_s = start_window_s
        self._start_grace_s = start_grace_s
        self._grace_s = grace_s
        self._forbidden = forbidden
        self._streak_start_step: int | None = None
        self._in_contact = False
        self.count = 0
        self.time_s = 0.0
        self.surface = ContactSurface.NONE
        """The surface that ended the run, once :meth:`update` has returned True."""

    def update(self, step: int, surface: ContactSurface) -> bool:
        """Fold in one tick's contact; return True if the run should end.

        Every contact counts toward :attr:`count`, but only a forbidden surface
        can end the run or accrue :attr:`time_s` — touching the wall this
        challenge permits is recorded, not punished.
        """
        touching = surface is not ContactSurface.NONE
        if touching and not self._in_contact:
            self.count += 1
        self._in_contact = touching

        if surface not in self._forbidden:
            self._streak_start_step = None
            return False

        if self._streak_start_step is None:
            self._streak_start_step = step
        self.time_s += self._dt
        streak_s = (step - self._streak_start_step) * self._dt

        if self._grace_s is not None:
            ended = streak_s >= self._grace_s
        else:
            began_at_start = (self._streak_start_step - 1) * self._dt <= self._start_window_s
            ended = not began_at_start or streak_s >= self._start_grace_s
        if ended:
            self.surface = surface
        return ended


@dataclass(frozen=True, slots=True)
class _StartConditions:
    section: Section
    direction: Direction
    x: float
    y: float
    yaw: float


class ScenarioSimulator:
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

    Pose is ground truth by default. Pass ``use_lidar_localization=True`` to
    navigate on the ``LidarLocalizer`` estimate instead — the same position
    source the real robot uses, and otherwise exercised by no closed-loop test
    at all. Keeping it opt-in preserves the perfect-odometry runs as a control,
    so the difference between the two is a direct measure of what state
    estimation costs.

    ``blind=True`` goes further and withholds the *layout*. Normally the
    corridor widths in the metadata reach the robot twice over — the planned
    path is built from them and the localizer matches scans against a wall
    model built from them — which is only honest if someone measured the mat
    first. WRO randomises the inner walls each round, so in blind mode the
    robot starts assuming every corridor is narrow (the safe prior) and
    estimates the real widths from LIDAR as it drives, replanning whenever an
    estimate changes. The metadata is then used only to build the physical
    track the robot is driving on, never to tell it anything.

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
        use_lidar_localization: bool = False,
        blind: bool = False,
        sensor_errors: SensorErrors | None = None,
        solid_walls: bool = False,
        infer_direction: bool | None = None,
        lidar_hz: float = LIDAR_SCAN_HZ,
        lidar_invalid_rate: float = LIDAR_INVALID_RAY_RATE,
        wall_heading: bool = True,
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
        self._errors = sensor_errors or SensorErrors()
        use_lidar_localization = use_lidar_localization or blind or self._errors.any_error

        true_geometry = corridor_widths_from_metadata(metadata)
        start = _start_conditions(metadata)
        challenge = metadata.challenge_type
        is_open_challenge = challenge == ScenarioType.OPEN
        self._terminal_surfaces = TERMINAL_SURFACES[ScenarioType.OPEN if is_open_challenge else ScenarioType.OBSTACLES]
        # Both challenges run the same tuning. An Obstacles-specific profile
        # (shorter lookahead + capped top speed) used to be applied here; it was
        # removed once re-measurement showed it changed nothing — see the note in
        # ``NavigationTuning`` where ``for_obstacles()`` used to be.
        nav_tuning = tuning if tuning is not None else NavigationTuning.load_default()
        # Traffic signs and parking blocks are real objects: the chassis can hit
        # them and the LIDAR can see them. Without them in the track model the
        # run reports success while driving straight through every sign.
        self._track = TrackModel(true_geometry, obstacles=obstacles_from_metadata(metadata.model_dump()))
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
        self._arc_radius = nav_tuning.waypoints.ARC_RADIUS
        self._width_estimator = (
            CorridorWidthEstimator(
                assumed_width=CorridorDimensions.NARROW
                if is_open_challenge
                else CorridorDimensions.OBSTACLES_WIDTH,
            )
            if blind
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
        self._direction_estimator = DirectionEstimator() if infer_direction else None
        self._creep_speed = nav_tuning.speed.SLOW_SPEED
        self._creep_widths: list[tuple[float, float]] = []
        """(yaw, measured width) taken before the direction was known."""
        self._start = start
        # Provisional until inference settles. Everything built from it -- the
        # path and the lap detector's finish-line normal -- is rebuilt then.
        self._direction = start.direction
        believed_dict = self._width_estimator.widths if self._width_estimator else true_geometry.to_widths_dict()

        # Mirror node.py: a single canonical lap, repeated num_laps times by the
        # navigator's waypoint-wrap + LapDetector lap counting.
        self._waypoints = self._plan(believed_dict)

        signs: list[SignSpec] = [] if is_open_challenge else signs_from_metadata(metadata.model_dump())

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
                self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(believed_dict)))
            else:
                self._gateway.set_believed_walls(TrackWalls(true_geometry))

        lap_detector = LapDetector(
            start_pos=(start.x, start.y),
            start_section=start.section,
            direction=start.direction,
        )

        sign_router: SignRouter | None = None
        if signs:
            sign_router = SignRouter(
                [] if discover_signs else signs,
                config=SignRouterConfig.from_tuning(nav_tuning.sign_router),
                direction=start.direction,
                discover=discover_signs,
                discovery_config=nav_tuning.sign_discovery,
            )

        self._park_controller: ParkController | None = None
        if not is_open_challenge:
            self._park_controller = park_controller_from_metadata(
                metadata.model_dump(), start.section, start.direction, tuning=nav_tuning,
            )

        self._navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=self._waypoints,
            num_laps=num_laps,
            tuning=nav_tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=self._park_controller,
        )

    def _plan(self, widths: dict[Section, float]) -> list[tuple[float, float]]:
        """Build a one-lap path for the layout the robot believes it is on."""
        from shared.domain.models import CorridorWidthEntry, CorridorWidths

        new_widths = CorridorWidths(
            **{s.value: CorridorWidthEntry(width_mm=round(width * 1000)) for s, width in widths.items()},
        )
        new_starting = self._metadata.starting_conditions.model_copy(
            update={"direction": str(self._direction)},
        )
        planning_metadata = self._metadata.model_copy(
            update={
                "corridor_widths": new_widths,
                "starting_conditions": new_starting,
            },
        )
        return calculate_waypoints(planning_metadata, num_laps=1, arc_radius=self._arc_radius)

    def _resolve_direction(self) -> bool:
        """Creep along the corridor until the travel direction is inferable.

        Returns:
            ``True`` while the direction is still unknown, meaning the caller
            drove the corridor follower this tick instead of the navigator.
        """
        estimator = self._direction_estimator
        if estimator is None or estimator.is_settled:
            return False

        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return True

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

        if estimator.observe(scan.ranges_m, scan.angles_rad, pose.yaw):
            inferred = estimator.direction
            if inferred is not None and inferred is not self._direction:
                # The path runs the other way round the loop and the finish
                # line's normal is inverted, so both are rebuilt.
                self._direction = inferred
                self._waypoints = self._plan(
                    self._width_estimator.widths if self._width_estimator else self._true_geometry.to_widths_dict(),
                )
                self._navigator.replace_lap_detector(
                    LapDetector(
                        start_pos=(self._start.x, self._start.y),
                        start_section=self._start.section,
                        direction=inferred,
                    ),
                )
            # Replay the buffered widths now that they can be attributed.
            if self._width_estimator is not None and inferred is not None:
                for buffered_yaw, buffered_width in self._creep_widths:
                    self._width_estimator.observe_measurement(
                        section_from_heading(buffered_yaw, inferred),
                        buffered_width,
                    )
                self._creep_widths.clear()
                self._waypoints = self._plan(self._width_estimator.widths)
                self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(self._width_estimator.widths)))

            # Resync unconditionally, including when the inference agreed with
            # the provisional direction and the path is unchanged. The
            # navigator did not step during the creep, so its waypoint index is
            # still 0 while the robot has driven a metre past it -- it would
            # resume by chasing a waypoint behind itself. Measured: this alone
            # cost fixtures that had inferred the direction perfectly.
            self._navigator.replace_path(self._waypoints, (pose.x, pose.y))
            return False

        self._gateway.publish_drive(
            follow_corridor(scan.ranges_m, scan.angles_rad, self._creep_speed),
        )
        return True

    def _creep_telemetry(
        self,
        prev_xy: tuple[float, float],
        on_step: Callable[[AckermannState, LidarScan], None] | None,
    ) -> float:
        """Publish and measure a creep tick; return the distance it covered."""
        scan = self._gateway.get_lidar_scan()
        if on_step is not None and scan is not None:
            on_step(self._gateway.state, scan)
        state = self._gateway.state
        return math.hypot(state.x - prev_xy[0], state.y - prev_xy[1])

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
        if not estimator.observe(section, scan.ranges_m, scan.angles_rad, pose.yaw):
            return False

        believed = estimator.widths
        self._waypoints = self._plan(believed)
        self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(believed)))
        self._navigator.replace_path(self._waypoints, (pose.x, pose.y))
        return True

    @property
    def believed_widths(self) -> dict[Section, float] | None:
        """What the robot currently thinks the layout is, or ``None`` if told."""
        return self._width_estimator.widths if self._width_estimator else None

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
    def waypoints(self) -> list[tuple[float, float]]:
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
        start_collision_window_s: float = START_COLLISION_WINDOW_S,
        start_collision_grace_s: float = START_COLLISION_GRACE_S,
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
            start_collision_grace_s: How long a start-window collision streak
                may continue before it's judged a real, terminal failure
                rather than "still working on steering clear."
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

        prev_xy = (gw.state.x, gw.state.y)
        metrics = _RunMetrics()
        prev_laps = 0
        lap_steps: list[int] = []
        terminal_collision = False
        contacts = _ContactTracker(
            dt=dt,
            start_window_s=start_collision_window_s,
            start_grace_s=start_collision_grace_s,
            grace_s=contact_grace_s,
            forbidden=self._terminal_surfaces,
        )

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
                prev_xy = (gw.state.x, gw.state.y)
                if contacts.update(step, gw.contact_surface if (gw.collided or gw.blocked) else ContactSurface.NONE):
                    terminal_collision = True
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
            metrics.observe(gw, math.hypot(sx - prev_xy[0], sy - prev_xy[1]))
            prev_xy = (sx, sy)

            if nav.laps_completed > prev_laps:
                lap_steps.append(step)
                prev_laps = nav.laps_completed

            surface = gw.contact_surface if (gw.collided or gw.blocked) else ContactSurface.NONE
            if contacts.update(step, surface):
                terminal_collision = True
                break

            if nav.laps_completed >= self._num_laps and (
                self._park_controller is None or self._park_controller.is_done
            ):
                break

        return self._build_result(
            step=step,
            dt=dt,
            max_steps=max_steps,
            collided=terminal_collision,
            contacts=contacts,
            metrics=metrics,
            lap_steps=lap_steps,
        )

    def _build_result(
        self,
        *,
        step: int,
        dt: float,
        max_steps: int,
        collided: bool,
        contacts: _ContactTracker,
        metrics: _RunMetrics,
        lap_steps: list[int],
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
        )


def _start_conditions(metadata: ScenarioMetadata) -> _StartConditions:
    sc = metadata.starting_conditions
    # Whole-number JSON metadata values parse as Python int, not float. Coerce here so a
    # downstream int never reaches a ROS message field, where CDR serialization would
    # corrupt it (bit-reinterpreted as float64 instead of converted — see live_visualizer's
    # _sign_marker for the same class of bug with sign/parking coordinates).
    return _StartConditions(
        section=Section.from_string(sc.section),
        direction=Direction.from_string(sc.direction),
        x=float(sc.position.x),
        y=float(sc.position.y),
        yaw=float(sc.yaw),
    )
