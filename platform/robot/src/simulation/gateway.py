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
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from shared.config.constants import DictKeys, RobotSpecs
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.race_tracker import LapDetector
from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import TrackModel

CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ


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
    ) -> None:
        self._track = track
        self._state = initial_state
        self._kin = kinematics or AckermannKinematics()
        self._rng = rng or np.random.default_rng(0)
        self._lidar_noise_std = lidar_noise_std

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
        """Return an empty list — the Open Challenge has no traffic signs."""
        return []

    # Simulation stepping

    @property
    def state(self) -> AckermannState:
        """Current kinematic state of the simulated body."""
        return self._state

    def advance(self, dt: float = CONTROL_DT) -> None:
        """Integrate the last command over ``dt`` and regenerate the sensors."""
        self._state = self._kin.step(
            self._state,
            target_speed=self._command.speed_mps,
            target_steer_norm=self._command.steering_norm,
            dt=dt,
        )
        if not self.collided and self._track.footprint_collides(
            self._state.x, self._state.y, self._state.yaw,
        ):
            self.collided = True
            self.collision_xy = (self._state.x, self._state.y)
        self._refresh_sensors()

    def _refresh_sensors(self) -> None:
        ranges = self._track.raycast_scan(
            self._state.x, self._state.y, self._state.yaw, self._angles,
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

    @property
    def success(self) -> bool:
        """Completed all target laps without a wall contact."""
        return self.laps_completed >= self.target_laps and not self.collided


@dataclass(frozen=True, slots=True)
class _StartConditions:
    section: Section
    direction: Direction
    x: float
    y: float
    yaw: float


class ScenarioSimulator:
    """Builds and runs a closed-loop Open Challenge simulation from metadata."""

    def __init__(
        self,
        metadata: dict[str, Any],
        num_laps: int = 3,
        tuning: NavigationTuning | None = None,
        lidar_noise_std: float = RobotSpecs.LIDAR_NOISE_STDDEV,
        kinematics: AckermannKinematics | None = None,
        seed: int = 0,
    ) -> None:
        self._metadata = metadata
        self._num_laps = num_laps
        nav_tuning = tuning or NavigationTuning()

        widths = corridor_widths_from_metadata(metadata)
        self._track = TrackModel(widths)
        start = _start_conditions(metadata)

        # Mirror node.py: a single canonical lap, repeated num_laps times by the
        # navigator's waypoint-wrap + LapDetector lap counting.
        self._waypoints = calculate_waypoints(metadata, num_laps=1, arc_radius=nav_tuning.waypoints.ARC_RADIUS)

        self._gateway = SimulatedHardwareGateway(
            track=self._track,
            initial_state=AckermannState(x=start.x, y=start.y, yaw=start.yaw),
            kinematics=kinematics,
            lidar_noise_std=lidar_noise_std,
            rng=np.random.default_rng(seed),
        )

        lap_detector = LapDetector(
            start_pos=(start.x, start.y),
            start_section=start.section,
            direction=start.direction,
        )
        self._navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=self._waypoints,
            num_laps=num_laps,
            tuning=nav_tuning,
            lap_detector=lap_detector,
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
    ) -> SimResult:
        """Run the control loop until all laps finish, a wall is hit, or timeout.

        Args:
            max_steps: Safety budget on control ticks (4000 ≈ 200 s at 20 Hz).
            dt: Control interval (seconds).
            on_step: Optional callback invoked with ``(state, lidar_scan)`` after
                every tick — used by the live Gazebo/RViz visualizer to publish
                the ground-truth pose and LIDAR sweep. ``None`` in the headless
                test battery, so it costs nothing there beyond one attribute check.

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

        step = 0
        while step < max_steps:
            nav.step()
            gw.advance(dt)
            step += 1

            if on_step is not None:
                on_step(gw.state, gw.get_lidar_scan())

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
                break
            if nav.laps_completed >= self._num_laps:
                break

        timed_out = step >= max_steps and nav.laps_completed < self._num_laps
        return SimResult(
            target_laps=self._num_laps,
            laps_completed=nav.laps_completed,
            collided=gw.collided,
            timed_out=timed_out,
            steps=step,
            sim_time_s=step * dt,
            distance_m=distance,
            max_speed_mps=max_speed,
            avg_speed_mps=(speed_sum / step) if step else 0.0,
            min_lidar_range_m=(min_range if math.isfinite(min_range) else 0.0),
            collision_xy=gw.collision_xy,
            final_pose=(gw.state.x, gw.state.y, gw.state.yaw),
            lap_step_indices=lap_steps,
        )


def _start_conditions(metadata: dict[str, Any]) -> _StartConditions:
    sc = metadata[DictKeys.STARTING_CONDITIONS]
    pos = sc[DictKeys.POSITION]
    return _StartConditions(
        section=Section.from_string(sc[DictKeys.SECTION]),
        direction=Direction.from_string(sc[DictKeys.DIRECTION]),
        x=pos[DictKeys.X],
        y=pos[DictKeys.Y],
        yaw=sc[DictKeys.YAW],
    )
