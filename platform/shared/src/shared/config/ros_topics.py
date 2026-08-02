"""ROS2 topic name configuration.

Single source of truth for ROS topic names, loaded from ros_topics.toml.
Prevents drift between telemetry_bridge_node subscriptions and what actually publishes.

Example usage:
    from shared.config.ros_topics import RosTopicConfig

    topics = RosTopicConfig.load_default()
    print(topics.state_machine.state)  # "/robot_state"
"""

from __future__ import annotations

from pathlib import Path
from typing import ClassVar

from pydantic import BaseModel, ConfigDict

try:
    import tomllib
except ImportError:
    import tomli as tomllib  # type: ignore


class StateMachineTopics(BaseModel):
    """State machine related topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    state: str
    """Robot state published by state_machine_node."""


class NavigationTopics(BaseModel):
    """Navigation related topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    odometry: str | None = None
    """Odometry from track_navigator_node (if implemented)."""


class SensorTopics(BaseModel):
    """Sensor data topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    scan: str
    """LIDAR range data from the Slamtec C1."""

    imu: str
    """IMU data from the BNO085."""

    hailo_detections: str
    """Hailo vision detections."""


class CommandTopics(BaseModel):
    """Command topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    cmd_vel: str
    """Drive command velocity (Twist) sent to the drive/steering nodes."""


class ActuatorTopics(BaseModel):
    """Actuator feedback topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    joint_states: str
    """Joint state feedback (steering angle, drive speed, encoder position)."""


class RosTopicConfig(BaseModel):
    """ROS2 topic names configuration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    state_machine: StateMachineTopics
    navigation: NavigationTopics
    sensors: SensorTopics
    commands: CommandTopics
    actuators: ActuatorTopics

    _default_config_path: ClassVar[Path] = (
        Path(__file__).resolve().parents[3] / "config" / "ros_topics.toml"
    )
    """Path to the checked-in ros_topics.toml file."""

    @classmethod
    def load_default(cls) -> RosTopicConfig:
        """Load topic configuration from the checked-in ros_topics.toml.

        Returns:
            RosTopicConfig instance with topic names.

        Raises:
            FileNotFoundError: If ros_topics.toml does not exist.
            ValueError: If TOML is invalid.
        """
        path = cls._default_config_path
        if not path.exists():
            raise FileNotFoundError(f"ROS topics config not found: {path}")

        with open(path, "rb") as f:
            data = tomllib.load(f)

        return cls.model_validate(data)
