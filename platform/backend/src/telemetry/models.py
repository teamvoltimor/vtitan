"""Telemetry domain models that serialize to JSON for the frontend."""

from __future__ import annotations

import enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel


class NodeHealth(enum.StrEnum):
    """Enumerates the telemetry node health states sent to the UI."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"


Position3D = tuple[float, float, float]


class TopicUpdate(BaseModel):
    """Single raw topic update with message data."""

    topic_name: str
    message_type: str  # e.g., "sensor_msgs/LaserScan"
    timestamp: float
    update_rate_hz: float  # Calculated from message frequency
    data: dict  # Raw message fields as nested dict

    model_config = ConfigDict(frozen=True, alias_generator=to_camel)


class TopicsSnapshot(BaseModel):
    """Collection of all active topic updates."""

    timestamp: float
    topics: list[TopicUpdate]

    model_config = ConfigDict(frozen=True, alias_generator=to_camel)


class ImuData(BaseModel):
    """IMU sensor data from BNO085."""

    linear_acceleration: tuple[float, float, float]  # m/s² (x, y, z)
    angular_velocity: tuple[float, float, float]  # rad/s (roll, pitch, yaw)
    orientation_quaternion: tuple[float, float, float, float]  # (x, y, z, w)

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class Detection(BaseModel):
    """YOLO object detection (traffic sign)."""

    class_name: str  # "red_sign" | "green_sign"
    confidence: float  # 0.0 - 1.0
    bbox: tuple[float, float, float, float]  # (x, y, w, h) normalized

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class MotorState(BaseModel):
    """Motor telemetry from BuildHAT."""

    steering_angle: float  # radians
    drive_speed: float  # motor speed (0-100 scale)
    encoder_position: int  # encoder ticks

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class TelemetryMetrics(BaseModel):
    """Aggregated statistics emitted from the ROS bridge."""

    timestamp: float = Field(..., description="Unix timestamp (seconds).")
    node_health: NodeHealth

    # LiDAR metrics (all optional)
    points_captured: int = Field(default=0, ge=0)
    range_min: Optional[float] = Field(default=None, ge=0.0)
    range_max: Optional[float] = Field(default=None, ge=0.0)
    range_mean: Optional[float] = Field(default=None, ge=0.0)
    forward: Optional[float] = Field(default=None, ge=0.0)
    left: Optional[float] = Field(default=None, ge=0.0)
    right: Optional[float] = Field(default=None, ge=0.0)
    back: Optional[float] = Field(default=None, ge=0.0)

    # Navigation metrics (optional)
    speed: Optional[float] = Field(default=None, ge=0.0)
    stage: str = Field(default="unknown")

    # Sensor health flags
    lidar_available: bool = Field(default=False)
    imu_available: bool = Field(default=False)
    camera_available: bool = Field(default=False)
    odometry_available: bool = Field(default=False)

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class RobotSnapshot(BaseModel):
    """Complete payload that the frontend consumes."""

    timestamp: float
    mission_name: str

    # Optional position (allows missing odometry)
    robot_position: Optional[Position3D] = None
    robot_orientation: Optional[float] = None

    # Sensor data (defaults to empty arrays)
    lidar_points: list[Position3D] = Field(default_factory=list)
    path_history: list[Position3D] = Field(default_factory=list)
    logs: list[str] = Field(default_factory=list)

    metrics: TelemetryMetrics

    # Extended telemetry (all optional)
    imu_data: Optional[ImuData] = None
    vision_detections: Optional[list[Detection]] = None
    motor_state: Optional[MotorState] = None

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )
