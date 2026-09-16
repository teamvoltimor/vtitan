"""Simulated hardware layer: Ackermann body + raycast LIDAR, no ROS2/Gazebo.

Backs a :class:`~src.navigation.ports.HardwareGateway` with a kinematic body
(:mod:`src.simulation.kinematics`) and a raycast LIDAR + collision model
(:mod:`src.simulation.track_model`), so :class:`ScenarioSimulator`
(:mod:`src.simulation.scenario_simulator`) can drive the real navigator
without Gazebo, ROS2, or a physics engine.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from collections import deque
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, cast

import numpy as np
from shared.config.constants import RobotSpecs
from shared.domain.enums import Section
from shared.domain.models import (
    IMUReading,
    LocalizerHealth,
    LocalizerInputs,
    Pose,
    SignColor,
    TrafficSignObservation,
    Waypoint,
)

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.localization import make_localizer
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.ports import DriveCommand, LidarScan, WheelOdometry, sanitize_lidar_ranges
from src.navigation.utils import wrap_angle as _wrap_angle
from src.navigation.wall_heading import estimate_yaw_from_walls
from src.simulation.collision_stepping import allowed_step
from src.simulation.imu_error_model import ImuErrorModel, SensorErrors
from src.simulation.kinematics import AckermannKinematics, AckermannState, KinematicsContext
from src.simulation.track_model import ContactSurface, TrackModel
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
    lidar_occlusion_min_rad: float
    lidar_occlusion_max_rad: float
    lidar_occlusion_dropout_rate: float
    lidar_occlusion_self_return_m: float
    lidar_occlusion_self_return_std_m: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning | None = None) -> _SimulatorConstants:
        tuning = get_tuning(tuning)
        control_hz = tuning.control.control_hz
        sim = tuning.simulation
        return cls(
            control_hz=control_hz,
            control_dt=1.0 / control_hz,
            lidar_invalid_ray_rate=sim.lidar_invalid_ray_rate,
            lidar_occlusion_min_rad=math.radians(sim.lidar_occlusion_min_deg),
            lidar_occlusion_max_rad=math.radians(sim.lidar_occlusion_max_deg),
            lidar_occlusion_dropout_rate=sim.lidar_occlusion_dropout_rate,
            lidar_occlusion_self_return_m=sim.lidar_occlusion_self_return_m,
            lidar_occlusion_self_return_std_m=sim.lidar_occlusion_self_return_std_m,
        )


class SimulatorContext(TuningContext[_SimulatorConstants]):
    """Context holding tuning-derived simulator constants."""

    _constants_cls = _SimulatorConstants


_DEFAULT_SIMULATOR_CONTEXT = SimulatorContext()

# Backward-compatible exports for existing imports.
CONTROL_DT = _DEFAULT_SIMULATOR_CONTEXT.constants.control_dt
LIDAR_INVALID_RAY_RATE = _DEFAULT_SIMULATOR_CONTEXT.constants.lidar_invalid_ray_rate

_OPPOSITE_SIGN_COLOR: dict[SignColor, SignColor] = {
    SignColor.RED: SignColor.GREEN,
    SignColor.GREEN: SignColor.RED,
}
"""The colour a confused detection reports instead of the true one.

Only the two routing colours swap. Anything else (a magenta barrier that got
this far, say) is left alone rather than mapped into a routing colour, because
inventing a route from a non-routing detection is a different failure from
confusing the two that do route."""

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
        self._init_lidar_occlusion()
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
        # Detections that have been "captured" but have not yet finished the
        # perception pipeline. The real one takes a MEASURED 0.85 s end to end
        # (sign_discovery.toml vision_latency_s, which the hardware gateway
        # compensates by backing the pose up before projecting). This emulator
        # reported from the CURRENT tick's true state until 2026-09-14, i.e.
        # with zero lag, handing the planner ~0.26 m of anticipation at
        # 0.3 m/s that it does not have on the mat.
        self._vision_pipeline: deque[tuple[float, list[TrafficSignObservation]]] = deque()
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
        sim = self.tuning.simulation
        # A residual detector miss, applied on TOP of the range model and
        # independently of it. The range model alone leaves the emulated camera
        # carrying a detection on ~54.5% of ticks; hardware carries one on
        # 11.6% (measured over 125 bags). That remaining gap is not explained by
        # capture rate -- 15 fps against a 20 Hz control loop would account for
        # about a quarter of it -- so it is modelled here as the detector simply
        # failing to fire, which is what the bags show.
        miss = sim.vision_frame_miss_rate
        candidates = list(self._signs or [])
        if sim.vision_range_model:
            ranged: list[SignSpec] = []
            for sign in candidates:
                distance = math.hypot(sign.x - self._state.x, sign.y - self._state.y)
                p_detect = 1.0 / (1.0 + math.exp((distance - sim.vision_detect_r50_m) / sim.vision_detect_falloff_m))
                if self._vision_rng.random() < p_detect:
                    ranged.append(sign)
            candidates = ranged
        if miss <= 0.0:
            return candidates
        return [sign for sign in candidates if self._vision_rng.random() >= miss]

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
        fresh = self._capture_vision_detections()
        return self._through_vision_pipeline(fresh)

    def _corrupt_detections(
        self, observations: list[TrafficSignObservation], origin: Pose | None = None
    ) -> list[TrafficSignObservation]:
        """Apply the camera errors the emulator otherwise has none of.

        Two of them, both MEASURED 2026-09-15 against the operator's stated
        layout on the three rounds whose pillar map reconstructs to it:

        * **Colour** -- 111 of 2,162 detections carry the opposite colour, 5.1%.
          The emulator copies ground-truth colour, so without this the biggest
          real perception failure (the magenta parking barrier arriving as a RED
          pillar) cannot be screened here at all and the aspect-gate defence
          built against it is dead code. NOTE the real errors are CONCENTRATED
          -- most pillars near 0%, one at 47% -- while this flip is i.i.d.; the
          marginal rate is honest, its structure is not.
        * **Bearing** -- sigma 0.232 rad (13.3 deg), from an interquartile range
          of -9.36 to +8.59 deg over 2,588 detections. The emulator projects
          from the TRUE bearing, so the only angular error a simulated run
          carried was the pose estimate's ~1.7 deg. At 1.5 m the real scatter is
          0.35 m of lateral miss, WIDER than the radii meant to contain it
          (association_dist_m 0.25, detection_match_dist_m 0.30).

        Bearing is applied FIRST and colour second: they are independent
        failures of the same frame, and rotating an already-flipped observation
        is the same thing as flipping a rotated one.

        ``origin`` is the frame the observation's world point was projected
        from; the scatter is a rotation about it, which is what a bearing error
        physically is. Without it the rotation would be about the world origin,
        which is not an error any camera can make.
        """
        sim = self.tuning.simulation
        quantiles = sim.vision_confidence_quantiles
        if quantiles:
            observations = [
                replace(obs, confidence=self._sample_confidence(quantiles, sim.vision_confidence_levels))
                for obs in observations
            ]
        scatter = sim.vision_bearing_scatter_rad
        if scatter > 0.0 and origin is not None:
            observations = [self._scatter_bearing(obs, origin, scatter) for obs in observations]
        flip = sim.vision_color_flip_rate
        if flip > 0.0:
            observations = [
                replace(obs, color=_OPPOSITE_SIGN_COLOR[obs.color])
                if obs.color in _OPPOSITE_SIGN_COLOR and self._vision_rng.random() < flip
                else obs
                for obs in observations
            ]
        return observations

    def _sample_confidence(self, quantiles: Sequence[float], levels: Sequence[float]) -> float:
        """Draw a detection confidence from the MEASURED distribution.

        Piecewise-linear inverse-CDF interpolation at ``levels``, so
        five numbers in a TOML reproduce the shape of 3,315 real detections
        without fitting a parametric family the data does not obviously have
        (it is skewed and bounded near 0.96).

        The levels are NOT evenly spaced, and assuming they were is a silent
        error rather than a loud one: with [0, .25, .5, .75, 1] the sampler
        still returns the median exactly while reading p10 as 0.477 against a
        measured 0.515 and p90 as 0.942 against 0.917. It looks calibrated at
        the one point anybody checks.

        The emulator otherwise stamps a CONSTANT, which sits at the real p90 --
        every frame one of its best. That matters less for
        `sign_router.min_confidence`, which rejects nothing at either value,
        than for `_SignTrack`: it weights its colour vote by confidence, so a
        constant makes every vote equal where the robot makes a 0.45 detection
        count half of a 0.95 one.
        """
        if len(quantiles) != len(levels) or len(levels) < 2:
            # A length the levels do not describe cannot be interpolated
            # honestly; fall back to the constant rather than invent a shape.
            return float(self.tuning.simulation.detection_confidence)
        u = float(self._vision_rng.random())
        for i in range(1, len(levels)):
            if u <= levels[i]:
                span = levels[i] - levels[i - 1]
                frac = 0.0 if span <= 0.0 else (u - levels[i - 1]) / span
                return float(quantiles[i - 1] + frac * (quantiles[i] - quantiles[i - 1]))
        return float(quantiles[-1])

    def _scatter_bearing(
        self, obs: TrafficSignObservation, origin: Pose, sigma: float
    ) -> TrafficSignObservation:
        """Rotate one observation about ``origin`` by a Gaussian bearing error.

        RANGE is preserved exactly. A bearing error moves a detection along the
        arc at its own range and does not change how far away the camera thinks
        it is -- and on this robot the range does not come from the camera at
        all when the LIDAR fusion agrees, so corrupting it here would model the
        wrong sensor.
        """
        dx, dy = obs.world_x_m - origin.x, obs.world_y_m - origin.y
        rng_m = math.hypot(dx, dy)
        if rng_m <= 0.0:
            return obs
        theta = math.atan2(dy, dx) + float(self._vision_rng.normal(0.0, sigma))
        return replace(
            obs,
            world_x_m=origin.x + rng_m * math.cos(theta),
            world_y_m=origin.y + rng_m * math.sin(theta),
        )

    def _through_vision_pipeline(
        self, fresh: list[TrafficSignObservation]
    ) -> list[TrafficSignObservation]:
        """Delay a capture by the measured end-to-end perception latency.

        Returns the most recent capture that has finished the pipeline, and
        ``[]`` while none has -- which is the honest answer for the opening
        0.85 s of a run, during which a real robot has seen nothing yet.
        """
        latency = self.tuning.simulation.vision_latency_s
        if latency <= 0.0:
            return fresh
        self._vision_pipeline.append((self._elapsed_s, fresh))
        matured: list[TrafficSignObservation] = []
        while self._vision_pipeline and self._elapsed_s - self._vision_pipeline[0][0] >= latency:
            matured = self._vision_pipeline.popleft()[1]
        return matured

    def _capture_vision_detections(self) -> list[TrafficSignObservation]:
        """What the camera resolves THIS instant, before pipeline delay."""
        signs = self._detectable_signs()
        if not signs:
            return []
        believed = self.get_current_pose()
        if self.tuning.simulation.vision_through_pinhole:
            # Boxes decoded by the SHIPPED perception code, so the corpus
            # exercises the pinhole, the bearing formula and the discovery gates
            # instead of being handed the answer. See
            # SimulationParams.vision_through_pinhole.
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
            return self._corrupt_detections([obs for obs in observations if obs is not None], pose)
        true_pose = Pose(x=self._state.x, y=self._state.y, yaw=self._state.yaw)
        return self._corrupt_detections(
            emulate_sign_observations(
                signs,
                Waypoint(self._state.x, self._state.y),
                self._state.yaw,
                tuning=self.tuning,
                believed_pos=Waypoint(believed.x, believed.y) if believed is not None else None,
                believed_yaw=believed.yaw if believed is not None else None,
            ),
            # The frame the point was projected from: `emulate_sign_observations`
            # reprojects through the BELIEVED pose when it has one, so rotating
            # about anything else would not be a bearing error.
            believed if believed is not None else true_pose,
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

    def _init_lidar_occlusion(self) -> None:
        """Resolve which rays the chassis blocks, once per gateway.

        Symmetric about the nose and fixed for the life of the gateway, so the
        mask is built here rather than rebuilt every sweep. The values it reads
        are documented in `simulation.toml`; what they DO is in
        `_apply_lidar_sensor_model`.

        Read from `self.tuning`, NOT from `context.constants`. The context is
        built from the process-default tuning unless a caller threads one
        through, so a sweep overriding these would have silently measured its
        own control -- the inert-knob failure this repo keeps rediscovering.
        """
        occl = self.tuning.simulation
        bearing = np.abs(self._angles)
        self._lidar_occluded = (bearing >= math.radians(occl.lidar_occlusion_min_deg)) & (
            bearing <= math.radians(occl.lidar_occlusion_max_deg)
        )
        self._lidar_occlusion_dropout_rate = occl.lidar_occlusion_dropout_rate
        self._lidar_occlusion_self_return_m = occl.lidar_occlusion_self_return_m
        self._lidar_occlusion_self_return_std_m = occl.lidar_occlusion_self_return_std_m

    def _apply_lidar_sensor_model(self, ranges: np.ndarray) -> np.ndarray:
        """Turn raycast truth into what the C1 actually reports.

        Three effects, all MEASURED 2026-09-14 against 5,763,600 real rays from
        ``/scan`` over three of the 2026-09-13/14 rounds. Before this the model
        was Gaussian noise plus a uniform 1% dropout, far cleaner than the real
        sensor in exactly the 0.04-0.10 m band where ``contact_dist`` and the
        escape gates live -- which is why the corpus could not arbitrate them.

        1. CHASSIS OCCLUSION. Two bands off the front corners, |bearing| 25-60
           deg, where dropout and sub-floor returns sum to essentially 100% of
           real rays: the sensor returns nothing usable there. 68.9% are
           non-finite; the rest are reflections off the chassis itself, tightly
           clustered (p50 0.0207 m, p5-p95 0.0108-0.0280). GEOMETRY, not noise,
           so a uniform dropout rate cannot stand in for it.

        2. NO LOWER CLIP AT ``LIDAR_MIN_RANGE``. The old code clamped to 0.045
           while the sector filter keeps ``r > min_valid_range_m`` = 0.044.
           Since 0.045 > 0.044, every ray that should have read "too close to
           measure" survived as a valid 4.5 cm obstacle -- the OPPOSITE of the
           hardware, which discards them. Clipping at 0 lets that filter be
           exercised the way it is on the robot.

        3. DROPOUT OUTSIDE THE BANDS at the measured 9.5% rather than a 1%
           guess. The 25.4% whole-sweep figure is dominated by the bands and
           must NOT be applied uniformly.

        Occlusion is applied AFTER noise deliberately: the 30 mm sigma is a
        wall-ranging figure and does not describe a surface 2 cm from the lens.
        """
        if self._lidar_noise_std > 0.0:
            ranges = ranges + self._rng.normal(0.0, self._lidar_noise_std, ranges.shape)
            # Lower bound 0.0, not LIDAR_MIN_RANGE -- see (2) above.
            ranges = np.clip(ranges, 0.0, RobotSpecs.LIDAR_MAX_RANGE)
        if self._lidar_invalid_rate > 0.0:
            # Slamtec drivers emit no-return rays as NaN/inf, most often off
            # dark or shallow-incidence surfaces. ``_lidar_callback`` in the
            # ROS2 node substitutes max range for them -- a filter that has
            # never once seen a value it was written for, because this
            # simulator produced only finite ranges.
            #
            # Excludes the occluded bands, which get their own rate below; a ray
            # cannot be dropped twice and the band rate is not an increment.
            invalid = (self._rng.random(ranges.shape) < self._lidar_invalid_rate) & ~self._lidar_occluded
            ranges = np.where(invalid, np.inf, ranges)
        if self._lidar_occlusion_dropout_rate > 0.0:
            drop = self._rng.random(ranges.shape) < self._lidar_occlusion_dropout_rate
            self_return = self._rng.normal(
                self._lidar_occlusion_self_return_m,
                self._lidar_occlusion_self_return_std_m,
                ranges.shape,
            )
            # Bounded at +-3 sigma, which is 0.0051-0.0363 m at the measured
            # mean and spread, against a real envelope of 0.0047-0.0435. Derived
            # from the two configured values rather than restated as literals.
            #
            # NOT clipped to LIDAR_MIN_RANGE (0.045): that sits ABOVE the
            # min_valid_range_m floor of 0.044, so it would hand the filter a
            # valid 4.5 cm obstacle -- precisely the defect (2) removes. The
            # bound has to stay below the floor, and the measured distribution
            # already does.
            span = 3.0 * self._lidar_occlusion_self_return_std_m
            self_return = np.clip(
                self_return,
                max(0.0, self._lidar_occlusion_self_return_m - span),
                self._lidar_occlusion_self_return_m + span,
            )
            ranges = np.where(self._lidar_occluded, np.where(drop, np.inf, self_return), ranges)
        return ranges

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
        ranges = self._apply_lidar_sensor_model(ranges)
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
        est = self._localizer.estimate_position(
            Waypoint(prior.x, prior.y),
            prior.yaw,
            self._scan_ranges,
            self._angles_list,
            now_s=self._elapsed_s,
        )
        self._estimator.update_position(est.x, est.y)

    def get_localizer_inputs(self) -> LocalizerInputs | None:
        """(yaw, prior_x, prior_y) handed to the localizer on the last scan."""
        return self._localizer_inputs

    def get_localizer_health(self) -> LocalizerHealth | None:
        """How well the last fix explained its scan, or ``None`` when localizing is off.

        This gateway is constructed with ``localize=False`` in the scenarios
        that drive on ground truth, and then there is no fit to report.
        """
        if self._localizer is None:
            return None
        return LocalizerHealth(
            fit_cost=self._localizer.last_fit_cost,
            relocalization_count=self._localizer.relocalization_count,
        )

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
