"""ROS2 topic name configuration.

Single source of truth for ROS topic names, loaded from ros_topics.toml.
Prevents drift between telemetry_bridge_node subscriptions and what actually publishes.

Example usage:
    from shared.config.ros_topics import RosTopicConfig

    topics = RosTopicConfig.load_default()
    print(topics.state_machine.state)  # "/robot_state"
"""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import ClassVar

from pydantic import BaseModel, ConfigDict


class StateMachineTopics(BaseModel):
    """State machine related topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    state: str
    """Robot state published by state_machine_node."""

    race_metrics: str
    """Race metrics (JSON), published by state_machine_node."""

    system_status: str
    """System diagnostics, published by both state_machine_node and telemetry_bridge_node."""


class NavigationTopics(BaseModel):
    """Navigation related topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    odometry: str | None = None
    """Odometry from track_navigator_node (if implemented)."""

    laps_completed: str
    """Lap count from track_navigator_node's CoreNavigator/LapDetector."""

    nav_debug: str
    """Full per-tick NavigatorDebugSnapshot (JSON) from track_navigator_node,
    covering both CoreNavigator.step() and the pre-direction-settle blind
    creep phase -- see NavigatorDebugSnapshot's own docstring."""


class SensorTopics(BaseModel):
    """Sensor data topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    scan: str
    """LIDAR range data from the Slamtec C1."""

    imu: str
    """IMU data from the BNO085."""

    hailo_fps: str
    """Hailo inference FPS, published by the vision/detector stack."""

    vision_detections: str
    """Vision detections (std_msgs/String, JSON), published by vision_node."""

    hailo_detections: str
    """Hailo vision detections.

    NOTE: as of 2026-08-08, nothing publishes vision_msgs/Detection2DArray on
    this topic -- vision_node only ever publishes JSON on `vision_detections`
    above. Kept for telemetry_bridge_node's still-dead Detection2DArray
    subscription, tracked separately for a rewrite.
    """


class CommandTopics(BaseModel):
    """Command topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    ackermann_cmd: str
    """Ackermann drive command sent to the drive/steering nodes."""


class ActuatorTopics(BaseModel):
    """Actuator feedback topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    joint_states: str
    """Joint state feedback (steering angle, drive speed, encoder position)."""

    steering_position: str
    """Current steering position in degrees, published by ackermann_motor_node."""

    drive_speed: str
    """Current drive speed in degrees/s, published by ackermann_motor_node."""

    status: str
    """Motor status diagnostics, published by ackermann_motor_node."""


class ChallengeModeTopics(BaseModel):
    """Challenge-mode jumper topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    jumper_inserted: str
    """Challenge-mode jumper state, published by the Pi Zero (which the wire is attached to)."""


class ButtonTopics(BaseModel):
    """Physical button topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    event: str
    """Button events from button_node on the Pi Zero."""

    hold: str
    """JSON hold-progress feedback from button_node, so the OLED can count down."""


class UiTopics(BaseModel):
    """Display/remote-viewing topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    telemetry_summary: str
    """Low-rate lidar/yaw/detection summary from telemetry_bridge_node for the OLED."""

    oled_mirror: str
    """Live mirror of the OLED panel, published by oled_display_node."""


class RosTopicConfig(BaseModel):
    """ROS2 topic names configuration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    state_machine: StateMachineTopics
    navigation: NavigationTopics
    sensors: SensorTopics
    commands: CommandTopics
    actuators: ActuatorTopics
    challenge_mode: ChallengeModeTopics
    button: ButtonTopics
    ui: UiTopics

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
            msg = f"ROS topics config not found: {path}"
            raise FileNotFoundError(msg)

        with path.open("rb") as f:
            data = tomllib.load(f)

        return cls.model_validate(data)
