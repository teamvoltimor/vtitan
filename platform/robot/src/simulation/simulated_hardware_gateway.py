"""Simulated hardware layer: Ackermann body + raycast LIDAR, no ROS2/Gazebo.

Backs a :class:`~src.navigation.ports.HardwareGateway` with a kinematic body
(:mod:`src.simulation.kinematics`) and a raycast LIDAR + collision model
(:mod:`src.simulation.track_model`), so :class:`ScenarioSimulator`
(:mod:`src.simulation.scenario_simulator`) can drive the real navigator
without Gazebo, ROS2, or a physics engine.
"""

from __future__ import annotations

from shared.domain.enums import Section

import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, cast

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.models import IMUReading, LocalizerInputs, Pose, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.localization import make_localizer
from src.navigation.ports import DriveCommand, LidarScan, WheelOdometry, sanitize_lidar_ranges
from src.navigation.utils import wrap_angle as _wrap_angle
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.simulation.collision_stepping import allowed_step
from src.simulation.imu_error_model import ImuErrorModel, SensorErrors
from src.simulation.kinematics import AckermannKinematics, AckermannState, KinematicsContext
from src.simulation.track_model import ContactSurface, TrackModel
from src.navigation.planning.sign_discovery import detection_to_observation
from src.simulation.vision_emulator import emulate_sign_detections, emulate_sign_observations
from src.state_machine.estimator import StateEstimator

if TYPE_CHECKING:
    from numpy.random import SeedSequence
    from shared.config.navigation_tuning import NavigationTuning

    from src.navigation.planning.sign_router import SignSpec
    from src.navigation.track_geometry import TrackWalls


@dataclass(frozen=True, slots=True)
class _SimulatorConstants:
    """Tuning-derived simulator constants, computed on-demand instead of frozen at module level."""

    control_hz: float
    control_dt: float
    lidar_invalid_ray_rate: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning | None = None) -> _SimulatorConstants:
        tuning = get_tuning(tuning)
        control_hz = tuning.control.CONTROL_HZ
        return cls(
            control_hz=control_hz,
            control_dt=1.0 / control_hz,
            lidar_invalid_ray_rate=tuning.simulation.LIDAR_INVALID_RAY_RATE,
        )


class SimulatorContext(TuningContext[_SimulatorConstants]):
    """Context holding tuning-derived simulator constants."""

    _constants_cls = _SimulatorConstants


_DEFAULT_SIMULATOR_CONTEXT = SimulatorContext()

# Backward-compatible exports for existing imports.
CONTROL_DT = _DEFAULT_SIMULATOR_CONTEXT.constants.control_dt
LIDAR_INVALID_RAY_RATE = _DEFAULT_SIMULATOR_CONTEXT.constants.lidar_invalid_ray_rate

_SCRUB_STANDSTILL_MPS = 1e-3
"""Commanded speed below which the chassis counts as stationary for scrub.

A millimetre per second: three orders under creep, so only a deliberate
standstill qualifies and no moving tick picks up a scrub bonus."""


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
        believed_start: AckermannState | None = None,
        kinematics: AckermannKinematics | None = None,
        lidar_rays: int = RobotSpecs.LIDAR_SAMPLES,
        lidar_noise_std: float = RobotSpecs.LIDAR_NOISE_STDDEV,
        rng: np.random.Generator | None = None,
        signs: list[SignSpec] | None = None,
        localize: bool = True,
        sensor_errors: SensorErrors | None = None,
        solid_walls: bool = False,
        solid_surfaces: frozenset[ContactSurface] | None = None,
        slide_on_contact: bool = False,
        scrub_yaw_gain: float = 0.0,
        # Sweep rate of the Slamtec C1, which is what the robot actually has.
        # Read from the sensor's own spec rather than restated, so the
        # simulated scan rate cannot drift from the rate the rest of the
        # stack assumes.
        #
        # The simulator previously regenerated the scan on every control
        # tick, so the navigator saw a fresh position fix at 20 Hz with no
        # age. Real scans arrive at half that rate and asynchronously, so
        # most control ticks act on a fix up to a scan period old -- during
        # which the chassis has moved up to 1.6 cm at full speed. Pass rates
        # measured against a perfectly fresh scan are optimistic by however
        # much that staleness costs.
        #
        # Set to 0 to restore the old always-fresh behaviour.
        lidar_hz: float = RobotSpecs.LIDAR_UPDATE_RATE,
        lidar_invalid_rate: float | None = None,
        wall_heading: bool = True,
        tuning: NavigationTuning | None = None,
        context: SimulatorContext | None = None,
    ) -> None:
        if context is None:
            context = _DEFAULT_SIMULATOR_CONTEXT
        self._context = context
        self.tuning = get_tuning(tuning)
        if lidar_invalid_rate is None:
            lidar_invalid_rate = context.constants.lidar_invalid_ray_rate
        self._track = track
        # Which surfaces physically stop the chassis. ``solid_walls`` makes all
        # of them solid; ``solid_surfaces`` names a subset, which is how a
        # surface that no longer ends the run is kept from being driven through.
        self._solid_surfaces = (
            frozenset(ContactSurface) - {ContactSurface.NONE} if solid_walls else (solid_surfaces or frozenset())
        )
        # Whether a blocked translation may slide along the surface instead of
        # being scaled to nothing. OFF by default: every contact-dependent
        # baseline in the repo was measured without it, so turning it on is a
        # re-baselining decision, not a default.
        self._slide_on_contact = slide_on_contact
        self._scrub_yaw_gain = scrub_yaw_gain
        self._state = initial_state
        # Build the kinematics from THIS simulator's tuning, not the process
        # default. Constructing it bare made every tuning-sourced kinematics
        # field inert under an override, which is why a --min-turn-radius
        # sweep produced output byte-identical to its control.
        self._kin = kinematics or AckermannKinematics(context=KinematicsContext(self.tuning))
        self._rng = rng or np.random.default_rng(0)
        self._lidar_noise_std = lidar_noise_std
        self._signs = signs

        # Full 360 sweep, robot frame, 0 = forward, +pi/2 = left, -pi/2 = right.
        self._angles = np.linspace(-math.pi, math.pi, lidar_rays)
        self._angles_list = self._angles.tolist()
        # The bearings never change, so the tuple every LidarScan carries is
        # built once here rather than per tick. Besides saving the rebuild, it
        # gives consumers a STABLE object to key a per-fan cache on -- which is
        # what lets ``_nearest_ray`` memoise its bearing lookup (a fresh tuple
        # each tick would miss that cache every time). Safe to share because a
        # tuple is immutable.
        self._angles_tuple = tuple(self._angles_list)

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
        # numpy types Generator.bit_generator.seed_seq as the narrow
        # ISeedSequence interface (no .spawn()), but np.random.default_rng
        # always builds a PCG64 bit generator backed by a concrete
        # SeedSequence, which does have it -- a stub gap, not a real
        # possibility of some other ISeedSequence implementation showing up.
        seed_seq = cast("SeedSequence", self._rng.bit_generator.seed_seq)
        self._error_rng = np.random.default_rng(seed_seq.spawn(1)[0])
        self._imu_model = ImuErrorModel(self._errors, self._error_rng)
        # Its own stream too, for the same reason -- see _detectable_signs.
        self._vision_rng = np.random.default_rng(seed_seq.spawn(1)[0])
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
        # The frame the robot believes it is standing in. A robot that assumes a
        # start pose it was not placed at does not perceive a corrected world --
        # its whole belief system is displaced together, heading included, and
        # the IMU zero goes with it. Keeping the offset here rather than at each
        # consumer is what stops the plan, the estimator and the lap line from
        # ending up in different frames, which is an incoherent state no robot
        # is ever in.
        believed = believed_start or initial_state
        self._localizer_inputs: LocalizerInputs | None = None
        self._yaw_offset = _wrap_angle(believed.yaw - initial_state.yaw)
        # Seed the estimator where the robot *thinks* it was placed. Offset at a
        # random bearing so the error is not systematically along-track (which
        # the localizer finds far easier to correct than a lateral one).
        seed_bearing = float(self._error_rng.uniform(-math.pi, math.pi))
        self._estimator = StateEstimator(
            believed.x + self._errors.start_pos_error_m * math.cos(seed_bearing),
            believed.y + self._errors.start_pos_error_m * math.sin(seed_bearing),
            self._reported_yaw(),
        )
        # Threaded from self.tuning.localization, matching the real-hardware
        # ROS2HardwareGateway (which threads the same LocalizationParams
        # fields) -- previously used LidarLocalizer's own hardcoded defaults
        # even though this __init__ already accepts and stores `tuning`.
        loc = self.tuning.localization
        self._localizer = make_localizer(track.walls, loc) if localize else None
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

        The LIDAR-matched estimate by default — the position source the real
        robot actually navigates on, so the error the localizer makes reaches
        the controller exactly as it does on hardware.

        ``localize=False`` gives ground truth (perfect odometry) instead, which
        isolates control behaviour from state-estimation error. That is a useful
        control to run deliberately; it used to be the default, which meant
        every result quietly assumed an accuracy the robot does not have.
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
            loc = self.tuning.localization
            self._localizer = make_localizer(walls, loc)

    def reset_position(self, x: float, y: float) -> None:
        """Re-seed the estimator's position, mirroring ``ROS2HardwareGateway``.

        No scenario currently exercises a mid-run reset (the sim harness runs
        one race per instance), but ``CoreNavigator`` is typed against
        ``HardwareGateway`` and constructed with this gateway, so it has to
        satisfy the same protocol the real one does.
        """
        self._estimator.reset_position(x, y)
        if self._localizer is not None:
            self._localizer.reset_tracking()

    def reset_heading_reference(self) -> None:
        """Re-zero the estimator's heading against the next IMU reading."""
        self._estimator.reset_heading_reference()

    def correct_heading_for_direction_change(self, delta_rad: float) -> None:
        """Shift the estimator's heading by a known amount, applied in full."""
        self._estimator.apply_yaw_correction(delta_rad)

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
        return pose.to_waypoint().distance_to(Waypoint(self._state.x, self._state.y))

    def get_lidar_scan(self) -> LidarScan | None:
        """Return the most recent simulated LIDAR sweep (ranges, robot-frame angles)."""
        return LidarScan(ranges_m=tuple(self._scan_ranges), angles_rad=self._angles_tuple)

    def get_imu_reading(self) -> IMUReading | None:
        """Return the IMU yaw (pitch/roll are zero on a flat mat).

        Ground truth unless ``SensorErrors`` configures drift or noise, and
        expressed in the frame the robot believes it started in.
        """
        return IMUReading(yaw=self._reported_yaw(), pitch=0.0, roll=0.0)

    def _imu_yaw(self) -> float:
        """Heading as the IMU reports it: truth plus accumulated drift and noise.

        Every consumer must go through here rather than reading ``state.yaw``,
        or the robot would navigate on a corrupted heading while some other
        part of the loop quietly used the true one.
        """
        return self._imu_model.yaw(self._state.yaw, self._elapsed_s, self._rotation_rad)

    def _reported_yaw(self) -> float:
        """The IMU heading as the *navigator* receives it, in the believed frame.

        Identical to :meth:`_imu_yaw` unless the robot was seeded believing it
        started somewhere other than where it was placed, in which case the
        constant frame offset rides along -- a heading is only meaningful
        relative to the frame the robot thinks it is driving in.

        Kept separate from :meth:`_imu_yaw` so ``heading_error_rad`` still
        measures what the sensor got wrong, not where the robot thinks it is.
        """
        return _wrap_angle(self._imu_yaw() + self._yaw_offset)

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


    def _detectable_signs(self) -> list[SignSpec]:
        """The signs the camera actually resolves THIS frame.

        Off (the default) this is every sign, and the emulator's own
        ``CAMERA_FAR_CLIP`` visibility check is the only range limit -- 10 m,
        against a real detector whose measured median detection range is 0.70 m.

        On, each sign is drawn independently per frame against a logistic in its
        TRUE range, so a distant pillar is seen intermittently and a far one
        effectively never. Drawn from a stream of its own rather than ``_rng``:
        consuming from the LIDAR's generator would shift its noise sequence and
        silently change every scan in the run, which is the same trap the IMU
        error model is spawned apart to avoid.
        """
        if not self.tuning.simulation.VISION_RANGE_MODEL:
            return list(self._signs or [])
        sim = self.tuning.simulation
        kept: list[SignSpec] = []
        for sign in self._signs or []:
            distance = math.hypot(sign.x - self._state.x, sign.y - self._state.y)
            p_detect = 1.0 / (1.0 + math.exp((distance - sim.VISION_DETECT_R50_M) / sim.VISION_DETECT_FALLOFF_M))
            if self._vision_rng.random() < p_detect:
                kept.append(sign)
        return kept

    def get_vision_detections(self, current_corridor: Section | None = None) -> list[TrafficSignObservation]:
        """Return synthetic sign observations, or ``[]`` if none were provided.

        Visibility (is a sign in frame, how far away) is judged from the TRUE
        pose -- that is what a real camera actually sees from. The reported
        world position is reprojected through the BELIEVED pose instead: a
        real perception pipeline has no access to ground truth, only the
        robot's own (possibly diverged) pose estimate, to place a detection
        in world coordinates. Falls back to the true pose when there is no
        separate estimate yet (``get_current_pose()`` returns ``None`` before
        the first scan), which reproduces zero belief error rather than
        silently dropping every sign that tick.
        """
        if not self._signs:
            return []
        signs = self._detectable_signs()
        if not signs:
            return []
        believed = self.get_current_pose()
        if self.tuning.simulation.VISION_THROUGH_PINHOLE:
            # Boxes decoded by the SHIPPED perception code, so the corpus
            # exercises the pinhole, the bearing formula and the discovery gates
            # instead of being handed the answer. See
            # SimulationParams.VISION_THROUGH_PINHOLE.
            pose = believed if believed is not None else Pose(x=self._state.x, y=self._state.y, yaw=self._state.yaw)
            observations = [
                detection_to_observation(det, pose, tuning=self.tuning)
                for det in emulate_sign_detections(
                    signs,
                    Waypoint(self._state.x, self._state.y),
                    self._state.yaw,
                    tuning=self.tuning,
                )
            ]
            return [obs for obs in observations if obs is not None]
        return emulate_sign_observations(
            signs,
            Waypoint(self._state.x, self._state.y),
            self._state.yaw,
            tuning=self.tuning,
            believed_pos=Waypoint(believed.x, believed.y) if believed is not None else None,
            believed_yaw=believed.yaw if believed is not None else None,
        )

    # Simulation stepping

    @property
    def state(self) -> AckermannState:
        """Current kinematic state of the simulated body."""
        return self._state

    def _scrub(self, candidate: AckermannState) -> AckermannState:
        """Yaw the chassis while the servo turns the wheels at a standstill.

        The kinematic model has ``yaw`` scaling with ``v``, so a stationary
        chassis rotates by exactly zero however hard the servo pushes. That is
        wrong on this robot by a wide margin: the steering servo delivers
        2.84 N-m (29 kg-cm at 5 V), which is 35-70x the moment needed to scrub a
        wheel in place and 10-30x the force needed to slide the whole 1.5 kg
        chassis sideways. In a pocket whose entire margin is 6 mm, scrub may be
        the dominant actuator rather than a second-order effect.

        DELIBERATELY OPTIMISTIC. The yaw is applied in the direction that helps,
        proportional to how far the wheels turned this tick, with no friction
        threshold and no cost. It is a sensitivity probe, not a physics model:
        the useful result is the THRESHOLD gain at which an outcome changes, so
        a bench measurement can be compared against it. A negative result under
        this model is therefore strong, and a positive one only says "possible".

        Off (0.0) by default; contact still clips the result via
        ``allowed_step``, so the chassis cannot scrub through a wall.
        """
        if not self._scrub_yaw_gain or abs(self._command.speed_mps) > _SCRUB_STANDSTILL_MPS:
            return candidate
        turned_rad = candidate.steer - self._state.steer
        return replace(candidate, yaw=_wrap_angle(candidate.yaw + self._scrub_yaw_gain * turned_rad))

    def advance(self, dt: float | None = None) -> None:
        """Integrate the last command over ``dt`` and regenerate the sensors.

        ``collided`` reflects the *current* tick only (re-evaluated every
        call, not latched) — a scenario placed at a legally tight starting
        position can be in wall contact before it's moved at all, and needs a
        moment to steer clear; whether that streak counts as a real failure
        is a run-level policy (see ``ScenarioSimulator.run``'s start-collision
        grace), not something the gateway itself should decide by freezing
        the flag the instant contact first occurs.
        """
        if dt is None:
            dt = self._context.constants.control_dt
        self._elapsed_s += dt
        prev_x, prev_y, prev_yaw = self._state.x, self._state.y, self._state.yaw
        candidate = self._kin.step(
            self._state,
            target_speed=self._command.speed_mps,
            target_steer_norm=self._command.steering_norm,
            dt=dt,
        )
        candidate = self._scrub(candidate)
        # A solid wall constrains the POSE, not the motion: the chassis
        # advances as far along the commanded step as fits and stops there,
        # rather than the whole step being refused.
        #
        # All-or-nothing refusal made contact absorbing. Every candidate that
        # kept any overlap was rejected, including the one that would have
        # slid the chassis free, so a robot that once touched a wall could
        # never move again -- it sat commanding 0.15 m/s with 0.76 m clear
        # ahead and travelled 0.00 m for the whole round. That is also why a
        # start in the narrow corridor's middle band failed 23 times out of
        # 23: with 6 mm of lateral clearance the chassis may yaw only 2.3
        # degrees before a corner reaches the block, and the navigator's very
        # first steering command asks for more.
        #
        # Grazing along a surface is what the real robot does to centre itself.
        # Scaling the step keeps the original guarantee intact -- driving
        # squarely into a wall still yields a scale of ~0 and makes no
        # progress, so reversing out remains a real escape rather than a
        # cosmetic one.
        allowed = allowed_step(self._track, self._solid_surfaces, self._state, candidate, slide=self._slide_on_contact)
        self.blocked = allowed is not candidate
        self._state = replace(self._state, v=0.0) if allowed is None else allowed
        # Whenever the step had to be cut short the chassis is against the
        # surface the full step would have entered, so that is what gets
        # scored -- the limited pose itself is clear by construction, and
        # reading the surface from it would report no contact at all.
        blocked_surface = (
            self._track.contact_surface(candidate.x, candidate.y, candidate.yaw)
            if self.blocked
            else ContactSurface.NONE
        )
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
        self._wheel_distance_m += (self._state.x - prev_x) * math.cos(prev_yaw) + (self._state.y - prev_y) * math.sin(
            prev_yaw
        )

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
            self._estimator.update_imu(IMUReading(yaw=self._reported_yaw(), pitch=0.0, roll=0.0))
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
        # Rays leave the SENSOR, not the chassis centre. `_state.x/y` is the
        # centre (kinematics: "(x, y) tracks the chassis centre") while the real
        # C1 sits LIDAR_MOUNT_X_OFFSET = 0.1222 m forward of it, flush with the
        # bumper. Casting from the centre made simulated forward ranges ~12 cm
        # longer than hardware would report -- larger than the whole 0.10 m
        # contact zone the navigator gates on. Side rays against a parallel wall
        # were unaffected, which is why blind corridor-WIDTH estimation kept
        # working and hid this for so long.
        #
        # Fixed 2026-08-21 together with the matching prediction in
        # `navigation/localization.py`, which had the same omission. Either one
        # alone would have desynchronised the estimator from its own sensor
        # model: they were consistently wrong with each other, and only the
        # hardware disagreed.
        sensor = Pose(self._state.x, self._state.y, self._state.yaw).sensor_origin(RobotSpecs.LIDAR_MOUNT_X_OFFSET)
        sensor_x = sensor.x
        sensor_y = sensor.y
        ranges = self._track.raycast_scan(
            sensor_x,
            sensor_y,
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
        self._scan_ranges = sanitize_lidar_ranges(ranges)
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
            measured = estimate_yaw_from_walls(
                self._scan_ranges, self._angles_list, prior_yaw=self._estimator.estimate_pose().yaw
            )
            if measured is not None:
                self._estimator.correct_yaw(measured)

        prior = self._estimator.estimate_pose()
        self._localizer_inputs = LocalizerInputs(prior.yaw, prior.x, prior.y)
        est_x, est_y = self._localizer.estimate_position(
            (prior.x, prior.y),
            prior.yaw,
            self._scan_ranges,
            self._angles_list,
            now_s=self._elapsed_s,
        )
        self._estimator.update_position(est_x, est_y)

    def get_localizer_inputs(self) -> LocalizerInputs | None:
        """(yaw, prior_x, prior_y) handed to the localizer on the last scan."""
        return self._localizer_inputs

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
