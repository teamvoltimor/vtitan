"""Telemetry domain models that serialize to JSON for the frontend."""

from __future__ import annotations

import enum

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel


class NodeHealth(enum.StrEnum):
    """Enumerates the telemetry node health states sent to the UI."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"


Position3D = tuple[float, float, float]


class TelemetryMetrics(BaseModel):
    """Aggregated statistics emitted from the ROS bridge."""

    timestamp: float = Field(..., description="Unix timestamp (seconds).")
    node_health: NodeHealth
    points_captured: int = Field(..., ge=0)
    range_min: float = Field(..., ge=0.0)
    range_max: float = Field(..., ge=0.0)
    range_mean: float = Field(..., ge=0.0)
    forward: float = Field(..., ge=0.0)
    left: float = Field(..., ge=0.0)
    right: float = Field(..., ge=0.0)
    back: float = Field(..., ge=0.0)
    speed: float = Field(..., ge=0.0)
    stage: str

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class RobotSnapshot(BaseModel):
    """Complete payload that the frontend consumes."""

    timestamp: float
    mission_name: str
    robot_position: Position3D
    robot_orientation: float
    lidar_points: list[Position3D]
    path_history: list[Position3D]
    logs: list[str]
    metrics: TelemetryMetrics

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )
