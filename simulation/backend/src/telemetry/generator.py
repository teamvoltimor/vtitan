"""Generates deterministic telemetry snapshots for the command-center UI."""

from __future__ import annotations

import math
import random
import time
from collections import deque

from src.telemetry.models import NodeHealth, Position3D, SimulationSnapshot, TelemetryMetrics
from src.telemetry.recorder import TelemetryRecorder

_RNG_SEED = 0
_ORBIT_PERIOD = 60   # frames per full orbit
_STAGE_DURATION = 18  # frames per navigation stage
_LOG_COUNT = 3        # log entries emitted per snapshot

_STAGES = ("start", "acceleration", "cornering", "straightaway", "finish")

_EVENT_TEMPLATES = (
    "ROS bridge: synchronized · {frequency:.1f} Hz",
    "Navigator: {stage} segment · {percent:.0f}% complete",
    "LIDAR: captured {points} points · obstacle {obstacle:.2f} m ahead",
    "Telemetry node {health} · speed {speed:.2f} m/s",
    "Replay buffer: {entries} snapshots stored",
)

_HEALTH_CYCLE = (NodeHealth.NOMINAL, NodeHealth.WATCHDOG, NodeHealth.REPLANNING)


class TelemetryGenerator:
    """Produces a continuous sequence of telemetry frames and keeps a rolling history."""

    def __init__(
        self,
        history_length: int = 120,
        lidar_resolution: int = 360,
        recorder: TelemetryRecorder | None = None,
    ) -> None:
        self._history: deque[SimulationSnapshot] = deque(maxlen=history_length)
        self._path: deque[Position3D] = deque(maxlen=history_length)
        self._frame = 0
        self._lidar_resolution = lidar_resolution
        self._rng = random.Random(_RNG_SEED)
        self._recorder = recorder
        self._advance_snapshot()

    def latest_snapshot(self) -> SimulationSnapshot:
        """Advance the simulation and return the newest snapshot."""
        return self._advance_snapshot()

    def history(self, limit: int | None = None) -> list[SimulationSnapshot]:
        """Return the most recent snapshots without advancing the stream."""
        snapshots = list(self._history)
        if limit is None or limit >= len(snapshots):
            return snapshots
        return snapshots[-limit:]

    def _advance_snapshot(self) -> SimulationSnapshot:
        self._frame += 1
        timestamp = time.time()
        orientation = (self._frame / _ORBIT_PERIOD) * math.tau
        robot_position = self._calc_robot_position(orientation)
        self._path.append(robot_position)
        lidar_points = self._generate_lidar_points(robot_position)
        metrics = self._build_metrics(timestamp, robot_position, orientation)
        logs = self._build_logs(metrics)
        snapshot = SimulationSnapshot(
            timestamp=timestamp,
            scenario_name="WRO Track 2026",
            robot_position=robot_position,
            robot_orientation=orientation,
            lidar_points=lidar_points,
            path_history=tuple(self._path),
            logs=logs,
            metrics=metrics,
        )
        if self._recorder:
            self._recorder.record(snapshot)
        self._history.append(snapshot)
        return snapshot

    def _calc_robot_position(self, orientation: float) -> Position3D:
        radius = 1.5 + math.sin(self._frame * 0.08) * 0.4
        x = radius * math.cos(orientation)
        y = radius * math.sin(orientation)
        return (x, y, 0.0)

    def _generate_lidar_points(self, robot_position: Position3D) -> list[Position3D]:
        x0, y0, _ = robot_position
        points: list[Position3D] = []
        for index in range(self._lidar_resolution):
            angle = (index / self._lidar_resolution) * math.tau
            turbulence = math.sin(index * 0.08 + self._frame * 0.03) * 0.08
            radius = 1.2 + turbulence + 0.2 * math.cos(angle * 4)
            x = x0 + radius * math.cos(angle)
            y = y0 + radius * math.sin(angle)
            z = 0.04 + math.sin(index * 0.1) * 0.01
            points.append((x, y, z))
        return points

    def _build_metrics(
        self,
        timestamp: float,
        robot_position: Position3D,
        orientation: float,
    ) -> TelemetryMetrics:
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
        )

    def _build_logs(self, metrics: TelemetryMetrics) -> tuple[str, ...]:
        entries: list[str] = []
        for index in range(_LOG_COUNT):
            template = _EVENT_TEMPLATES[(self._frame + index) % len(_EVENT_TEMPLATES)]
            entries.append(
                template.format(
                    frequency=12 + math.sin(self._frame * 0.02) * 2,
                    stage=metrics.stage,
                    percent=50 + (self._frame % 50),
                    points=metrics.points_captured,
                    obstacle=0.2 + abs(math.sin(self._frame * 0.05)) * 0.2,
                    health=metrics.node_health.value,
                    speed=metrics.speed,
                    entries=len(self._history),
                )
            )
        return tuple(entries)

    def _compute_node_health(self) -> NodeHealth:
        return _HEALTH_CYCLE[(self._frame // _ORBIT_PERIOD) % len(_HEALTH_CYCLE)]
