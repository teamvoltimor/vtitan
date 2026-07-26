"""Headless closed-loop simulation of the WRO Open Challenge.

Drives the *real* :class:`~src.navigation.core_navigator.CoreNavigator` — the
exact pure-pursuit controller, collision controller, stuck detector and
``LapDetector`` the car runs — through a simulated :class:`HardwareGateway`.

The gateway owns an Ackermann bicycle-model body (:mod:`kinematics`) and a
raycast LIDAR + collision model (:mod:`track_model`). Each control tick mirrors
the ROS2 node exactly (``platform/robot/src/ros2/navigation/node.py``):

1. ``navigator.step()`` reads pose + LIDAR, computes steering/speed, and
   publishes a ``DriveCommand`` (``speed_mps``, ``steering_norm`` in [-1, 1]).
2. The gateway integrates that command over ``dt`` and regenerates the sensors.

No Gazebo, no ROS2, no physics engine — pure Python, runs anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any

import numpy as np
from shared.config.constants import DictKeys, RobotSpecs
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.core_navigator import CoreNavigator
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, SignSpec, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.race_tracker import LapDetector
from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import TrackModel, obstacles_from_metadata
from src.simulation.vision_emulator import emulate_sign_detections

if TYPE_CHECKING:
    from collections.abc import Callable

CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

START_COLLISION_WINDOW_S = 2.0
"""A collision streak beginning within this long of run start is judged as a
starting-position issue (see ``ScenarioSimulator.run``), not a driving mistake."""

START_COLLISION_GRACE_S = 15.0
"""How long a starting-position collision streak may continue before it's a real failure."""


class SimulatedHardwareGateway:
    """A :class:`HardwareGateway` backed by a kinematic body and raycast LIDAR.

    Pose is reported as ground truth (perfect odometry); LIDAR carries realistic
    Gaussian range noise. This isolates *navigation/control* behaviour from
    state-estimation error, which the real robot fuses separately.
    """

    def __init__(
        self,
        track: TrackModel,
        initial_state: AckermannState,
        kinematics: AckermannKinematics | None = None,
        lidar_rays: int = RobotSpecs.LIDAR_SAMPLES,
        lidar_noise_std: float = RobotSpecs.LIDAR_NOISE_STDDEV,
        rng: np.random.Generator | None = None,
        signs: list[SignSpec] | None = None,
    ) -> None:
        self._track = track
        self._state = initial_state
        self._kin = kinematics or AckermannKinematics()
        self._rng = rng or np.random.default_rng(0)
        self._lidar_noise_std = lidar_noise_std
        self._signs = signs

        # Full 360 sweep, robot frame, 0 = forward, +pi/2 = left, -pi/2 = right.
        self._angles = np.linspace(-math.pi, math.pi, lidar_rays)
        self._angles_list = self._angles.tolist()

        self._command = DriveCommand(speed_mps=0.0, steering_norm=0.0)
        self._scan_ranges: list[float] = []
        self.collided = False
        self.collision_xy: tuple[float, float] | None = None
        self._refresh_sensors()

    # HardwareGateway protocol

    def publish_drive(self, command: DriveCommand) -> None:
        """Store the latest command; applied on the next :meth:`advance`."""
        self._command = command

    def get_current_pose(self) -> Pose | None:
        """Return the ground-truth pose (perfect odometry)."""
        return Pose(x=self._state.x, y=self._state.y, yaw=self._state.yaw)

    def get_lidar_scan(self) -> LidarScan | None:
        """Return the most recent simulated LIDAR sweep (ranges, robot-frame angles)."""
        return LidarScan(ranges_m=tuple(self._scan_ranges), angles_rad=tuple(self._angles_list))

    def get_imu_reading(self) -> IMUReading | None:
        """Return the IMU yaw from ground truth (pitch/roll are zero on a flat mat)."""
        return IMUReading(yaw=self._state.yaw, pitch=0.0, roll=0.0)

    def get_vision_detections(self) -> list[Detection]:
        """Return synthetic detections for ``signs``, or ``[]`` if none were provided."""
        if not self._signs:
            return []
        return emulate_sign_detections(self._signs, (self._state.x, self._state.y), self._state.yaw)

    # Simulation stepping

    @property
    def state(self) -> AckermannState:
        """Current kinematic state of the simulated body."""
        return self._state

    def advance(self, dt: float = CONTROL_DT) -> None:
        """Integrate the last command over ``dt`` and regenerate the sensors.

        ``collided`` reflects the *current* tick only (re-evaluated every
        call, not latched) — a scenario placed at a legally tight starting
        position can be in wall contact before it's moved at all, and needs a
        moment to steer clear; whether that streak counts as a real failure
        is a run-level policy (see ``ScenarioSimulator.run``'s start-collision
        grace), not something the gateway itself should decide by freezing
        the flag the instant contact first occurs.
        """
        self._state = self._kin.step(
            self._state,
            target_speed=self._command.speed_mps,
            target_steer_norm=self._command.steering_norm,
            dt=dt,
        )
        self.collided = self._track.footprint_collides(
            self._state.x,
            self._state.y,
            self._state.yaw,
        )
        if self.collided:
            self.collision_xy = (self._state.x, self._state.y)
        self._refresh_sensors()

    def apply_disturbance(self, lateral_m: float, heading_rad: float = 0.0) -> None:
        """Kick the body sideways and/or off-heading, e.g. to test recovery from drift.

        ``lateral_m`` offsets the pose perpendicular to the current heading
        (positive = left of travel direction); ``heading_rad`` adds to yaw.
        Speed is left untouched — this models a pose disturbance, not a
        velocity change.
        """
        s = self._state
        nx = s.x - lateral_m * math.sin(s.yaw)
        ny = s.y + lateral_m * math.cos(s.yaw)
        self._state = replace(s, x=nx, y=ny, yaw=s.yaw + heading_rad)
        self._refresh_sensors()

    def _refresh_sensors(self) -> None:
        ranges = self._track.raycast_scan(
            self._state.x,
            self._state.y,
            self._state.yaw,
            self._angles,
        )
        if self._lidar_noise_std > 0.0:
            ranges = ranges + self._rng.normal(0.0, self._lidar_noise_std, ranges.shape)
            ranges = np.clip(ranges, RobotSpecs.LIDAR_MIN_RANGE, RobotSpecs.LIDAR_MAX_RANGE)
        self._scan_ranges = ranges.tolist()
        self._last_min_range = float(np.min(ranges))

    @property
    def last_min_range(self) -> float:
        """Closest range in the most recent LIDAR sweep (metres)."""
        return self._last_min_range


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
    lap_step_indices: list[int] = field(default_factory=list)
    parked: bool | None = None
    """``None`` when the scenario has no parking lot; else whether parking finished cleanly
    (as opposed to giving up on its frame budget — see ``ParkController.is_timed_out``)."""

    @property
    def success(self) -> bool:
        """Completed all target laps without a wall contact (and parked cleanly, if required)."""
        return self.laps_completed >= self.target_laps and not self.collided and self.parked is not False


@dataclass(frozen=True, slots=True)
class PoseDisturbance:
    """A one-time pose kick applied mid-run, e.g. to test recovery from drift.

    ``lateral_m`` offsets perpendicular to the current heading (positive =
    left of travel direction); ``heading_rad`` adds to yaw.
    """

    lateral_m: float
    heading_rad: float = 0.0


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
    """

    def __init__(
        self,
        metadata: dict[str, Any],
        num_laps: int = 3,
        tuning: NavigationTuning | None = None,
        lidar_noise_std: float = RobotSpecs.LIDAR_NOISE_STDDEV,
        kinematics: AckermannKinematics | None = None,
        seed: int = 0,
        emit_vision_detections: bool = False,
    ) -> None:
        self._metadata = metadata
        self._num_laps = num_laps

        widths = corridor_widths_from_metadata(metadata)
        start = _start_conditions(metadata)
        is_open_challenge = metadata.get(DictKeys.CHALLENGE_TYPE, ScenarioType.OPEN) == ScenarioType.OPEN
        # Obstacles needs tighter path tracking than Open: the margin for
        # threading past a sign is far smaller than the corridor the Open
        # Challenge drives, so it defaults to the shorter-lookahead profile.
        # An explicit `tuning` argument still wins.
        if tuning is not None:
            nav_tuning = tuning
        else:
            nav_tuning = NavigationTuning() if is_open_challenge else NavigationTuning.for_obstacles()
        # Traffic signs and parking blocks are real objects: the chassis can hit
        # them and the LIDAR can see them. Without them in the track model the
        # run reports success while driving straight through every sign.
        self._track = TrackModel(widths, obstacles=obstacles_from_metadata(metadata))

        # Mirror node.py: a single canonical lap, repeated num_laps times by the
        # navigator's waypoint-wrap + LapDetector lap counting.
        self._waypoints = calculate_waypoints(metadata, num_laps=1, arc_radius=nav_tuning.waypoints.ARC_RADIUS)

        signs: list[SignSpec] = [] if is_open_challenge else signs_from_metadata(metadata)

        self._gateway = SimulatedHardwareGateway(
            track=self._track,
            initial_state=AckermannState(x=start.x, y=start.y, yaw=start.yaw),
            kinematics=kinematics,
            lidar_noise_std=lidar_noise_std,
            rng=np.random.default_rng(seed),
            signs=signs if emit_vision_detections else None,
        )

        lap_detector = LapDetector(
            start_pos=(start.x, start.y),
            start_section=start.section,
            direction=start.direction,
        )

        sign_router: SignRouter | None = None
        if signs:
            sign_router = SignRouter(signs, direction=start.direction)

        self._park_controller: ParkController | None = None
        if not is_open_challenge:
            self._park_controller = park_controller_from_metadata(metadata, start.section, start.direction)

        self._navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=self._waypoints,
            num_laps=num_laps,
            tuning=nav_tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=self._park_controller,
        )

    @property
    def track(self) -> TrackModel:
        """The track geometry model for this scenario."""
        return self._track

    @property
    def waypoints(self) -> list[tuple[float, float]]:
        """The single-lap canonical waypoint path fed to the navigator."""
        return self._waypoints

    def run(
        self,
        max_steps: int = 4000,
        dt: float = CONTROL_DT,
        on_step: Callable[[AckermannState, LidarScan], None] | None = None,
        disturb_at_step: int | None = None,
        disturbance: PoseDisturbance | None = None,
        start_collision_window_s: float = START_COLLISION_WINDOW_S,
        start_collision_grace_s: float = START_COLLISION_GRACE_S,
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

        Returns:
            A populated :class:`SimResult`.
        """
        gw = self._gateway
        nav = self._navigator

        prev_xy = (gw.state.x, gw.state.y)
        distance = 0.0
        max_speed = 0.0
        speed_sum = 0.0
        min_range = math.inf
        prev_laps = 0
        lap_steps: list[int] = []
        collision_streak_start_step: int | None = None
        terminal_collision = False

        step = 0
        while step < max_steps:
            nav.step()
            gw.advance(dt)
            step += 1

            if disturbance is not None and step == disturb_at_step:
                gw.apply_disturbance(disturbance.lateral_m, disturbance.heading_rad)

            scan = gw.get_lidar_scan()
            if on_step is not None and scan is not None:
                on_step(gw.state, scan)

            sx, sy = gw.state.x, gw.state.y
            distance += math.hypot(sx - prev_xy[0], sy - prev_xy[1])
            prev_xy = (sx, sy)
            speed = abs(gw.state.v)
            max_speed = max(max_speed, speed)
            speed_sum += speed
            min_range = min(min_range, gw.last_min_range)

            if nav.laps_completed > prev_laps:
                lap_steps.append(step)
                prev_laps = nav.laps_completed

            if gw.collided:
                if collision_streak_start_step is None:
                    collision_streak_start_step = step
                streak_started_at_start = (collision_streak_start_step - 1) * dt <= start_collision_window_s
                streak_duration_s = (step - collision_streak_start_step) * dt
                if not streak_started_at_start or streak_duration_s >= start_collision_grace_s:
                    terminal_collision = True
                    break
            else:
                collision_streak_start_step = None

            if nav.laps_completed >= self._num_laps and (
                self._park_controller is None or self._park_controller.is_done
            ):
                break

        pc = self._park_controller
        parked = None if pc is None else (pc.is_done and not pc.is_timed_out)
        timed_out = step >= max_steps and (nav.laps_completed < self._num_laps or (pc is not None and not pc.is_done))
        return SimResult(
            target_laps=self._num_laps,
            laps_completed=nav.laps_completed,
            collided=terminal_collision,
            timed_out=timed_out,
            steps=step,
            sim_time_s=step * dt,
            distance_m=distance,
            max_speed_mps=max_speed,
            avg_speed_mps=(speed_sum / step) if step else 0.0,
            min_lidar_range_m=(min_range if math.isfinite(min_range) else 0.0),
            collision_xy=gw.collision_xy,
            parked=parked,
            final_pose=(gw.state.x, gw.state.y, gw.state.yaw),
            lap_step_indices=lap_steps,
        )


def _start_conditions(metadata: dict[str, Any]) -> _StartConditions:
    sc = metadata[DictKeys.STARTING_CONDITIONS]
    pos = sc[DictKeys.POSITION]
    # Whole-number JSON metadata values parse as Python int, not float. Coerce here so a
    # downstream int never reaches a ROS message field, where CDR serialization would
    # corrupt it (bit-reinterpreted as float64 instead of converted — see live_visualizer's
    # _sign_marker for the same class of bug with sign/parking coordinates).
    return _StartConditions(
        section=Section.from_string(sc[DictKeys.SECTION]),
        direction=Direction.from_string(sc[DictKeys.DIRECTION]),
        x=float(pos[DictKeys.X]),
        y=float(pos[DictKeys.Y]),
        yaw=float(sc[DictKeys.YAW]),
    )
