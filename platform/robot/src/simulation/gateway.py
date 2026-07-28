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
from shared.config.constants import RobotSpecs
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import IMUReading, Pose, ScenarioMetadata, TrafficSignObservation

from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import follow_corridor
from src.navigation.direction_estimator import DirectionEstimator
from src.navigation.localization import LidarLocalizer
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand, LidarScan, WheelOdometry
from src.navigation.race_tracker import LapDetector
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths, corridor_widths_from_metadata
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import ContactSurface, TrackModel, obstacles_from_metadata
from src.simulation.geometry import _wrap_angle
from src.simulation.vision_emulator import emulate_sign_observations
from src.state_machine.estimator import StateEstimator

if TYPE_CHECKING:
    from collections.abc import Callable

CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

START_COLLISION_WINDOW_S = 2.0
"""A collision streak beginning within this long of run start is judged as a
starting-position issue (see ``ScenarioSimulator.run``), not a driving mistake."""

START_COLLISION_GRACE_S = 15.0
"""How long a starting-position collision streak may continue before it's a real failure."""

LIDAR_SCAN_HZ = 10.0
"""Sweep rate of the Slamtec C1, which is what the robot actually has.

The simulator previously regenerated the scan on every control tick, so the
navigator saw a fresh position fix at 20 Hz with no age. Real scans arrive at
half that rate and asynchronously, so most control ticks act on a fix up to a
scan period old -- during which the chassis has moved up to 1.6 cm at full
speed. Pass rates measured against a perfectly fresh scan are optimistic by
however much that staleness costs.

Set to 0 to restore the old always-fresh behaviour.
"""

LIDAR_INVALID_RAY_RATE = 0.01
"""Fraction of rays returning no measurement, as NaN/inf.

Slamtec drivers emit these off dark or shallow-incidence surfaces, and
``ROS2HardwareGateway._lidar_callback`` has always substituted max range for
them -- a filter that had never seen a value it was written for, because this
simulator only ever produced finite ranges. 1% is a placeholder: the real rate
depends on the mat's surface and is worth measuring from a recorded bag rather
than guessed at.
"""

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


def _sanitize_ranges(ranges: np.ndarray) -> list[float]:
    """Replace no-return rays with max range, exactly as the ROS2 node does.

    Mirrors ``ROS2HardwareGateway._lidar_callback``: NaN silently drops out of
    every downstream mask and inf reads as "far away", so both become max range
    before the scan is handed to navigation. Duplicated here deliberately --
    the point of emitting invalid returns in simulation is that navigation
    receives the same *sanitised* scan it would on the robot, so the two paths
    have to agree on what sanitised means.
    """
    cleaned = np.where(np.isfinite(ranges), ranges, RobotSpecs.LIDAR_MAX_RANGE)
    return np.clip(cleaned, 0.0, RobotSpecs.LIDAR_MAX_RANGE).tolist()


@dataclass(frozen=True, slots=True)
class SensorErrors:
    """Imperfections in what the robot knows about itself, as opposed to the track.

    The layout is withheld by ``blind``; this withholds the two things the sim
    otherwise hands over for free about the *robot*:

    Where it starts. The estimator is normally seeded with the exact pose the
    body was placed at, which no operator can supply — the robot is set down
    by hand somewhere inside a starting zone, not on a surveyed point. The
    localizer can only correct that error by matching scans, so a bad seed is
    a real search problem, not a bookkeeping one.

    Which way it is pointing. IMU yaw is otherwise ground truth forever. A
    BNO085 drifts, and blind mode leans on heading harder than anything else
    does: ``corridor_estimator.section_from_heading`` attributes every width
    reading by heading, so yaw error does not merely steer badly, it can
    file a measurement under the wrong corridor.

    All heading error is modelled on the *reading*, never as a one-off seed of
    the estimator. The estimator takes yaw from the IMU on every update, so a
    seeded yaw offset would be overwritten on the first tick and measure
    nothing. That is also the physical truth: the BNO085's yaw zero is fixed at
    boot, so a chassis set down askew is wrong by that angle for the whole
    round rather than converging out of it.

    All values are magnitudes; the sign and bearing are drawn from the run's
    seeded RNG, so a scenario perturbs the same way every time it is run while
    different scenarios perturb differently.

    Attributes:
        start_pos_error_m: Distance between where the body is and where the
            estimator is told it is (random bearing). The localizer can work
            this off by matching scans; it is a search problem, not a fixed
            handicap.
        yaw_bias_rad: Constant offset between the IMU's yaw zero and the world
            frame -- the chassis set down askew, or the IMU zeroed askew.
            Never corrected, because nothing else observes absolute heading.
        imu_drift_rad_per_s: Yaw drift rate accumulated over elapsed time
            (random sign). This is the BNO085's quoted 0.5 deg/min figure.
        gyro_scale_error: Fractional error in how much rotation the gyro
            reports, e.g. ``0.005`` for 0.5%. Accumulates per *degree turned*
            rather than per second, which is why it is modelled separately from
            drift: a lap-driving robot turns 12 corners of 90 degrees in three
            laps, so it banks over 1080 degrees of deliberate rotation and the
            error scales with the course rather than the clock. A robot vacuum
            wanders and largely cancels this out; this one does not.
        imu_noise_rad: Per-reading Gaussian yaw noise.
    """

    start_pos_error_m: float = 0.0
    yaw_bias_rad: float = 0.0
    imu_drift_rad_per_s: float = 0.0
    gyro_scale_error: float = 0.0
    imu_noise_rad: float = 0.0

    @property
    def any_error(self) -> bool:
        """True if this configures any perturbation at all."""
        return bool(
            self.start_pos_error_m
            or self.yaw_bias_rad
            or self.imu_drift_rad_per_s
            or self.gyro_scale_error
            or self.imu_noise_rad,
        )


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
        localize: bool = False,
        sensor_errors: SensorErrors | None = None,
        solid_walls: bool = False,
        solid_surfaces: frozenset[ContactSurface] | None = None,
        lidar_hz: float = LIDAR_SCAN_HZ,
        lidar_invalid_rate: float = LIDAR_INVALID_RAY_RATE,
        wall_heading: bool = True,
    ) -> None:
        self._track = track
        # Which surfaces physically stop the chassis. ``solid_walls`` makes all
        # of them solid; ``solid_surfaces`` names a subset, which is how a
        # surface that no longer ends the run is kept from being driven through.
        self._solid_surfaces = (
            frozenset(ContactSurface) - {ContactSurface.NONE} if solid_walls else (solid_surfaces or frozenset())
        )
        self._state = initial_state
        self._kin = kinematics or AckermannKinematics()
        self._rng = rng or np.random.default_rng(0)
        self._lidar_noise_std = lidar_noise_std
        self._signs = signs

        # Full 360 sweep, robot frame, 0 = forward, +pi/2 = left, -pi/2 = right.
        self._angles = np.linspace(-math.pi, math.pi, lidar_rays)
        self._angles_list = self._angles.tolist()

        # Position estimation, mirroring ``ROS2HardwareGateway`` in
        # ``src/ros2/navigation/node.py``: heading from the IMU, position from
        # matching each LIDAR sweep against the known wall geometry, seeded
        # from the previous estimate. Off by default so the existing battery
        # keeps its perfect-odometry control condition.
        self._localize = localize
        self._errors = sensor_errors or SensorErrors()
        # A stream of its own, spawned from the run seed. Drawing these from
        # ``_rng`` would shift the LIDAR noise sequence and silently change every
        # existing result, including the unperturbed control runs.
        self._error_rng = np.random.default_rng(self._rng.bit_generator.seed_seq.spawn(1)[0])
        # Both signs are fixed per run, not re-rolled per tick: a gyro bias is a
        # constant, and a sign that wandered would average itself out and
        # understate the damage.
        self._drift_sign = float(self._error_rng.choice([-1.0, 1.0]))
        self._bias_sign = float(self._error_rng.choice([-1.0, 1.0]))
        self._scale_sign = float(self._error_rng.choice([-1.0, 1.0]))
        self._elapsed_s = 0.0
        # Signed rotation the body has actually turned through, unwrapped, so
        # three laps of one-way cornering accumulate rather than cancel.
        self._rotation_rad = 0.0
        self._prev_true_yaw = initial_state.yaw
        # Path length covered, which is what a wheel encoder integrates. Note
        # this accumulates while blocked by a solid wall only if the body
        # actually moved -- a chassis held against a wall reports no travel,
        # matching an encoder on a stalled but not slipping wheel.
        self._wheel_distance_m = 0.0
        # Seed the estimator where the robot *thinks* it was placed. Offset at a
        # random bearing so the error is not systematically along-track (which
        # the localizer finds far easier to correct than a lateral one).
        seed_bearing = float(self._error_rng.uniform(-math.pi, math.pi))
        self._estimator = StateEstimator(
            initial_state.x + self._errors.start_pos_error_m * math.cos(seed_bearing),
            initial_state.y + self._errors.start_pos_error_m * math.sin(seed_bearing),
            self._imu_yaw(),
        )
        # Uses LidarLocalizer's own defaults, which match NavigationTuning.localization's
        # defaults -- not threaded through a tuning param here (this __init__ has no
        # existing tuning-injection path, unlike the real-hardware ROS2HardwareGateway).
        self._localizer = LidarLocalizer(track.walls) if localize else None
        self._believed_walls: TrackWalls | None = None

        self._wall_heading = wall_heading
        self._lidar_period_s = (1.0 / lidar_hz) if lidar_hz > 0 else 0.0
        self._lidar_invalid_rate = lidar_invalid_rate
        self._last_scan_s = 0.0

        self._command = DriveCommand(speed_mps=0.0, steering_norm=0.0)
        self._scan_ranges: list[float] = []
        self.collided = False
        self.contact_surface = ContactSurface.NONE
        """Which surface the chassis is against this tick, blocked or penetrating."""
        self.blocked = False
        """Last :meth:`advance` was refused because it would have entered a wall.

        Only ever True with ``solid_walls``, and it — not ``collided`` — is the
        contact signal in that mode: a refused move leaves the chassis at its
        last legal pose, which by construction is *not* penetrating, so
        ``collided`` stays False however hard the robot pushes."""
        self.collision_xy: tuple[float, float] | None = None
        self._refresh_sensors()

    # HardwareGateway protocol

    def publish_drive(self, command: DriveCommand) -> None:
        """Store the latest command; applied on the next :meth:`advance`."""
        self._command = command

    def get_current_pose(self) -> Pose | None:
        """Return the pose the navigator gets to see.

        Ground truth by default (perfect odometry), which isolates control
        behaviour from state-estimation error. With ``localize=True`` this is
        instead the LIDAR-matched estimate the real robot actually navigates
        on, so the error the localizer makes reaches the controller.
        """
        if self._localize:
            return self._estimator.estimate_pose()
        return Pose(x=self._state.x, y=self._state.y, yaw=self._state.yaw)

    def set_believed_walls(self, walls: TrackWalls) -> None:
        """Re-point the localizer at the layout the robot currently *believes* in.

        Without this the localizer matches against ``track.walls`` — the true
        geometry — which quietly hands the robot the map it is supposed to be
        working out for itself. In blind mode the belief comes from
        :class:`~src.navigation.corridor_estimator.CorridorWidthEstimator`, so
        a wrong belief produces a wrong position fix, exactly as it would on
        the real mat.
        """
        self._believed_walls = walls
        if self._localize:
            self._localizer = LidarLocalizer(walls)

    @property
    def position_error_m(self) -> float:
        """How far the pose the navigator sees is from ground truth (metres).

        0.0 without ``localize`` — the navigator is handed ground truth, so by
        definition it sees no error. (The estimator still exists in that mode
        but is never fed, so its own state is meaningless and deliberately not
        reported here.) The point of the flag is to make this non-zero and
        measure what state estimation costs.
        """
        if not self._localize:
            return 0.0
        pose = self._estimator.estimate_pose()
        return math.hypot(pose.x - self._state.x, pose.y - self._state.y)

    def get_lidar_scan(self) -> LidarScan | None:
        """Return the most recent simulated LIDAR sweep (ranges, robot-frame angles)."""
        return LidarScan(ranges_m=tuple(self._scan_ranges), angles_rad=tuple(self._angles_list))

    def get_imu_reading(self) -> IMUReading | None:
        """Return the IMU yaw (pitch/roll are zero on a flat mat).

        Ground truth unless ``SensorErrors`` configures drift or noise.
        """
        return IMUReading(yaw=self._imu_yaw(), pitch=0.0, roll=0.0)

    def _imu_yaw(self) -> float:
        """Heading as the IMU reports it: truth plus accumulated drift and noise.

        Every consumer must go through here rather than reading ``state.yaw``,
        or the robot would navigate on a corrupted heading while some other
        part of the loop quietly used the true one.
        """
        errors = self._errors
        yaw = (
            self._state.yaw
            + self._bias_sign * errors.yaw_bias_rad
            + self._drift_sign * errors.imu_drift_rad_per_s * self._elapsed_s
            + self._scale_sign * errors.gyro_scale_error * self._rotation_rad
        )
        if errors.imu_noise_rad > 0.0:
            yaw += float(self._error_rng.normal(0.0, errors.imu_noise_rad))
        return yaw

    @property
    def heading_error_rad(self) -> float:
        """Signed difference between the IMU heading and ground truth."""
        return _wrap_angle(self._imu_yaw() - self._state.yaw)

    @property
    def rotation_rad(self) -> float:
        """Signed rotation the body has turned through since the run started.

        Unwrapped, so three laps of one-way cornering accumulate. This is the
        quantity a gyro scale-factor error multiplies.
        """
        return self._rotation_rad

    def get_wheel_odometry(self) -> WheelOdometry:
        """Wheel travel and speed, as an encoder on this chassis would report it.

        Distance is the path length the body has actually covered, which is
        what a wheel encoder measures -- not displacement from the start, and
        not anything that knows the heading.

        Perfect by default. The real encoder was calibrated against a tape at
        two distances that agreed to ~1%, and the agreement across speeds is
        itself evidence that slip is negligible on this surface, so a noiseless
        encoder is a defensible model. It is still a model: on a mat with
        different grip the same argument would have to be re-made.
        """
        return WheelOdometry(
            distance_m=self._wheel_distance_m,
            speed_mps=self._state.v,
            stamp_s=self._elapsed_s,
        )

    def get_vision_detections(self) -> list[TrafficSignObservation]:
        """Return synthetic sign observations, or ``[]`` if none were provided."""
        if not self._signs:
            return []
        return emulate_sign_observations(self._signs, (self._state.x, self._state.y), self._state.yaw)

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
        self._elapsed_s += dt
        prev_x, prev_y, prev_yaw = self._state.x, self._state.y, self._state.yaw
        candidate = self._kin.step(
            self._state,
            target_speed=self._command.speed_mps,
            target_steer_norm=self._command.steering_norm,
            dt=dt,
        )
        # With solid walls a move that would put the chassis inside one is
        # refused outright and the body stops where it is, so driving into a
        # wall makes no progress and only a move that clears the wall is
        # allowed through. That is what makes reversing out a real escape
        # rather than a cosmetic one: without it the chassis passes straight
        # through and "recovered" would mean "drove through the wall".
        blocked_surface = self._track.contact_surface(candidate.x, candidate.y, candidate.yaw)
        if blocked_surface in self._solid_surfaces:
            self._state = replace(self._state, v=0.0)
            self.blocked = True
        else:
            self._state = candidate
            self.blocked = False
            blocked_surface = ContactSurface.NONE
        # Unwrapped so a lap's worth of same-sign cornering adds up instead of
        # wrapping back to zero at +/-pi.
        self._rotation_rad += _wrap_angle(self._state.yaw - self._prev_true_yaw)
        self._prev_true_yaw = self._state.yaw
        # Signed along the heading, not unsigned path length: a quadrature
        # encoder counts down in reverse, so counts_to_distance() decreases and
        # the real distance_m is signed. Accumulating hypot() here would make a
        # reversing robot report travel forwards, and any motion prior built on
        # it would push the position estimate the wrong way during exactly the
        # manoeuvre -- the escape reverse -- where the robot is already in
        # trouble.
        self._wheel_distance_m += (self._state.x - prev_x) * math.cos(prev_yaw) + (
            self._state.y - prev_y
        ) * math.sin(prev_yaw)

        settled = self._track.contact_surface(self._state.x, self._state.y, self._state.yaw)
        # With solid walls the refused move names the surface, since the pose
        # actually held is by construction clear of it.
        self.contact_surface = settled if settled is not ContactSurface.NONE else blocked_surface
        self.collided = settled is not ContactSurface.NONE
        if self.collided:
            self.collision_xy = (self._state.x, self._state.y)

        # The IMU is fused every tick; the LIDAR only when a sweep completes.
        # Real hardware runs them an order of magnitude apart -- BNO085 at
        # 100 Hz against a C1 spinning near 10 Hz -- so with a 20 Hz control
        # loop most ticks steer on a position fix that is up to a scan period
        # old while the heading is current. Refreshing both every tick, as this
        # did, gives the navigator a perfectly fresh position it will never
        # have, and leaves no interval for wheel odometry to fill.
        if self._localizer is not None:
            self._estimator.update_imu(IMUReading(yaw=self._imu_yaw(), pitch=0.0, roll=0.0))
        if self._elapsed_s - self._last_scan_s >= self._lidar_period_s:
            self._last_scan_s = self._elapsed_s
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
        if self._lidar_invalid_rate > 0.0:
            # Slamtec drivers emit no-return rays as NaN/inf, most often off
            # dark or shallow-incidence surfaces. ``_lidar_callback`` in the
            # ROS2 node substitutes max range for them -- a filter that has
            # never once seen a value it was written for, because this
            # simulator produced only finite ranges.
            invalid = self._rng.random(ranges.shape) < self._lidar_invalid_rate
            ranges = np.where(invalid, np.inf, ranges)
        self._scan_ranges = _sanitize_ranges(ranges)
        self._last_min_range = min(self._scan_ranges)

        if self._localizer is None:
            return
        # Same order as the node's LIDAR callback: the localizer takes yaw as
        # given, then searches for the position that best explains this sweep,
        # seeded from the previous estimate. Heading itself is fused every
        # control tick in ``advance``, not here -- the BNO085 runs an order of
        # magnitude faster than the LIDAR, so gating it on a scan would make
        # the sim's heading staler than the robot's.
        # Correct heading against the walls before solving for position: the
        # localizer takes yaw as given, so a better yaw makes a better fix.
        # This is the only bound on heading -- the IMU has no absolute
        # reference, so without it drift and scale error accumulate for the
        # whole round.
        if self._wall_heading:
            measured = estimate_yaw_from_walls(self._scan_ranges, self._angles_list, prior_yaw=self._estimator.estimate_pose().yaw)
            if measured is not None:
                self._estimator.correct_yaw(measured)

        prior = self._estimator.estimate_pose()
        est_x, est_y = self._localizer.estimate_position(
            (prior.x, prior.y),
            prior.yaw,
            self._scan_ranges,
            self._angles_list,
        )
        self._estimator.update_position(est_x, est_y)

    @property
    def last_min_range(self) -> float:
        """Closest range in the most recent LIDAR sweep (metres)."""
        return self._last_min_range

    @property
    def last_command(self) -> DriveCommand:
        """The most recent :class:`DriveCommand`, as published by the navigator.

        Read-only view for diagnostics that need the *commanded* speed/steering
        rather than the integrated result — the two differ whenever the
        acceleration or steering-slew limits bind.
        """
        return self._command


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
        # runs start with every corridor assumed narrow and correct it from
        # LIDAR as they go.
        self._arc_radius = nav_tuning.waypoints.ARC_RADIUS
        self._width_estimator = CorridorWidthEstimator() if blind else None
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
        distance = 0.0
        max_speed = 0.0
        speed_sum = 0.0
        min_range = math.inf
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
                distance += self._creep_telemetry(prev_xy, on_step)
                prev_xy = (gw.state.x, gw.state.y)
                min_range = min(min_range, gw.last_min_range)
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
            distance += math.hypot(sx - prev_xy[0], sy - prev_xy[1])
            prev_xy = (sx, sy)
            speed = abs(gw.state.v)
            max_speed = max(max_speed, speed)
            speed_sum += speed
            min_range = min(min_range, gw.last_min_range)

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
            distance=distance,
            max_speed=max_speed,
            speed_sum=speed_sum,
            min_range=min_range,
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
        distance: float,
        max_speed: float,
        speed_sum: float,
        min_range: float,
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
            distance_m=distance,
            max_speed_mps=max_speed,
            avg_speed_mps=(speed_sum / step) if step else 0.0,
            min_lidar_range_m=(min_range if math.isfinite(min_range) else 0.0),
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
