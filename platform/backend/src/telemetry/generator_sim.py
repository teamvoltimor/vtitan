"""Generates deterministic telemetry snapshots for the command-center UI."""

from __future__ import annotations

import math
import random
import time
from collections import deque
from typing import TYPE_CHECKING

from src.telemetry.config import SimulationConstants
from src.telemetry.models import NodeHealth, Position3D, RobotSnapshot, TelemetryMetrics

if TYPE_CHECKING:
    from src.telemetry.recorder import TelemetryRecorder

_RNG_SEED = SimulationConstants.RNG_SEED
_ORBIT_PERIOD = SimulationConstants.ORBIT_PERIOD_FRAMES
_STAGE_DURATION = SimulationConstants.STAGE_DURATION_FRAMES
_LOG_COUNT = SimulationConstants.LOG_ENTRIES_PER_SNAPSHOT

_STAGES = SimulationConstants.STAGE_NAMES

_EVENT_TEMPLATES = SimulationConstants.LOG_TEMPLATES

_HEALTH_CYCLE = (NodeHealth.NOMINAL, NodeHealth.WATCHDOG, NodeHealth.REPLANNING)


class TelemetryGenerator:
    """Produces deterministic telemetry frames seeded through `random.Random`."""

    def __init__(
        self,
        history_length: int = 120,
        lidar_resolution: int = 360,
        recorder: TelemetryRecorder | None = None,
    ) -> None:
        self._history: deque[RobotSnapshot] = deque(maxlen=history_length)
        self._path: deque[Position3D] = deque(maxlen=history_length)
        self._frame = 0
        self._lidar_resolution = lidar_resolution
        self._rng = random.Random(_RNG_SEED)  # noqa: S311
        self._recorder = recorder
        self._advance_snapshot()

    def latest_snapshot(self) -> RobotSnapshot:
        """Advance the simulation and return the newest snapshot."""
        return self._advance_snapshot()

    def history(self, limit: int | None = None) -> list[RobotSnapshot]:
        """Return the most recent snapshots without advancing the stream."""
        snapshots = list(self._history)
        if limit is None or limit >= len(snapshots):
            return snapshots
        return snapshots[-limit:]

    def _advance_snapshot(self) -> RobotSnapshot:
        self._frame += 1
        timestamp = time.time()
        orientation = (self._frame / _ORBIT_PERIOD) * math.tau
        robot_position = self._calc_robot_position(orientation)
        self._path.append(robot_position)
        lidar_points = self._generate_lidar_points(robot_position)
        metrics = self._build_metrics(timestamp, orientation)
        logs = self._build_logs(metrics)
        snapshot = RobotSnapshot(
            timestamp=timestamp,
            mission_name="WRO Track 2026",
            robot_position=robot_position,
            robot_orientation=orientation,
            lidar_points=lidar_points,
            path_history=list(self._path),
            logs=logs,
            metrics=metrics,
        )
        if self._recorder:
            self._recorder.record(snapshot)
        self._history.append(snapshot)
        return snapshot

    def _calc_robot_position(self, orientation: float) -> Position3D:
        radius = 0.9 + math.sin(self._frame * 0.08) * 0.2
        x = 1.5 + radius * math.cos(orientation)
        y = 1.5 + radius * math.sin(orientation)
        return (x, y, 0.0)

    def _generate_lidar_points(self, robot_position: Position3D) -> list[Position3D]:
        """Simulate lidar sweep points with seeded turbulence from `random.Random`."""
        x0, y0, _ = robot_position
        points: list[Position3D] = []
        for index in range(self._lidar_resolution):
            angle = (index / self._lidar_resolution) * math.tau
            turbulence = math.sin(index * 0.08 + self._frame * 0.03) * 0.08 + self._rng.random() * 0.02
            radius = 1.2 + turbulence + 0.2 * math.cos(angle * 4)
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            z = 0.04 + math.sin(index * 0.1) * 0.01
            points.append((x, y, z))
        return points

    def _build_metrics(
        self,
        timestamp: float,
        orientation: float,
    ) -> TelemetryMetrics:
        """Construct metrics for the current frame using the provided orientation."""
        stage = _STAGES[(self._frame // _STAGE_DURATION) % len(_STAGES)]
        node_health = self._compute_node_health()
        speed = max(0.3, 0.6 + 0.2 * math.cos(self._frame * 0.04))
        return TelemetryMetrics(
            timestamp=timestamp,
            node_health=node_health,
            points_captured=self._lidar_resolution,
            range_min=0.12 + abs(math.sin(orientation * 0.5)) * 0.05,
            range_max=3.1 + abs(math.cos(orientation * 0.3)) * 0.4,
            range_mean=1.6 + math.sin(self._frame * 0.03) * 0.1,
            forward=0.35 + abs(math.cos(self._frame * 0.07)) * 0.3,
            left=0.4 + abs(math.sin(self._frame * 0.08)) * 0.3,
            right=0.38 + abs(math.sin(self._frame * 0.05)) * 0.28,
            back=0.3 + abs(math.cos(self._frame * 0.09)) * 0.25,
            speed=speed,
            stage=stage,
            lidar_available=True,
            odometry_available=True,
            imu_available=False,
            camera_available=False,
        )

    def _build_logs(self, metrics: TelemetryMetrics) -> list[str]:
        """Generate log messages that reflect the current metrics with jitter."""
        entries: list[str] = []
        for index in range(_LOG_COUNT):
            template = _EVENT_TEMPLATES[(self._frame + index) % len(_EVENT_TEMPLATES)]
            entries.append(
                template.format(
                    frequency=12 + math.sin(self._frame * 0.02) * 2,
                    stage=metrics.stage,
                    percent=50 + (self._frame % 50) + self._rng.uniform(-2, 2),
                    points=metrics.points_captured,
                    obstacle=0.2 + abs(math.sin(self._frame * 0.05)) * 0.2,
                    health=metrics.node_health.value,
                    speed=metrics.speed,
                    entries=len(self._history),
                ),
            )
        return entries

    def _compute_node_health(self) -> NodeHealth:
        return _HEALTH_CYCLE[(self._frame // _ORBIT_PERIOD) % len(_HEALTH_CYCLE)]
