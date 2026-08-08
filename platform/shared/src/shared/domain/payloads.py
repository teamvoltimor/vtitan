"""Pydantic boundaries for API and WebSocket communication.

These models ensure strict validation at the edges of the system.
"""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field
from shared.domain.enums import NodeHealth, RobotState
from shared.domain.models import Detection, Pose, Velocity


class SystemStatusPayload(BaseModel):
    """System health status snapshot payload."""

    battery_voltage: float = Field(default=0.0)
    battery_current: float = Field(default=0.0)
    cpu_temp: float = Field(default=0.0)
    motor_temps: list[float] = Field(default_factory=list)
    ros_nodes_active: int = Field(default=0)


class RaceMetricsPayload(BaseModel):
    """Racing metrics during autonomous run."""

    lap_number: int = Field(default=0)
    lap_time: float = Field(default=0.0)
    total_time: float = Field(default=0.0)
    waypoint_index: int = Field(default=0)
    collision_count: int = Field(default=0)
    forward_clearance: float = Field(default=0.0)


class RobotSnapshotPayload(BaseModel):
    """Complete snapshot of robot state and telemetry."""

    timestamp: float
    state: RobotState
    health: NodeHealth
    position: Pose
    velocity: Velocity
    lidar_ranges: list[float] = Field(default_factory=list)
    vision_detections: list[Detection] = Field(default_factory=list)
    system_status: SystemStatusPayload
    race_metrics: RaceMetricsPayload


class WebSocketMessage(BaseModel):
    """Generic envelope for WebSocket messages."""

    topic: str
    timestamp: float
    data: dict[str, Any]
