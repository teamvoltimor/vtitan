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
from enum import StrEnum
from pathlib import Path
from typing import ClassVar

from pydantic import BaseModel, ConfigDict


class RosMessageType(StrEnum):
    """ROS2 message type strings, as returned by rclpy's own graph-introspection APIs.

    Covers ``get_publisher_names_and_types_by_node``, and
    ``get_subscriptions_info_by_topic``'s ``.topic_type``. Named here so tests
    pinning a node's message-type contract don't hand-retype the
    ``"pkg_name/msg/TypeName"`` string.
    """

    ACKERMANN_DRIVE_STAMPED = "ackermann_msgs/msg/AckermannDriveStamped"
    IMAGE = "sensor_msgs/msg/Image"
    JOINT_STATE = "sensor_msgs/msg/JointState"
    STRING = "std_msgs/msg/String"


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

    camera_image_raw: str
    """Raw camera image, published by the vision stack (for diagnostics/monitoring)."""

    joy: str
    """Joystick input (sensor_msgs/Joy), consumed by joy_teleop_node."""


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

    active: str
    """Resolved ScenarioType ("open"/"obstacles"), published by state_machine_node once
    the jumper reading is debounced/latched (or timed out to Open). Latched (TRANSIENT_LOCAL)
    so a late-subscribing track_navigator_node still gets the current value immediately.
    Republished on every BOOT_CHECK resolution, including the one after a SYSTEM_RESET, so
    the robot can switch challenges purely from the button without a process restart."""


class BagRecorderTopics(BaseModel):
    """mcap bag recorder topics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    run_path: str
    """The run directory bag_recorder_node just started writing to, published once
    per RACING entry (latched). Lets a separate process -- e.g. vision_node's
    per-run video recorder -- write into the exact same directory as the mcap
    without the two nodes sharing any other state."""


class SimulationTopics(BaseModel):
    """Simulation-only topics, published by live_visualizer."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    odom: str
    """Simulated robot pose (Odometry)."""

    track: str
    """Simulated track markers (MarkerArray)."""

    plan: str
    """Planned waypoint polyline the navigator is tracking (Path).

    A belief, not ground truth. Under the Obstacles Challenge it is also the
    avoidance manoeuvre itself -- ``apply_sign_lanes`` rewrites these waypoints
    onto a pass-side lane -- so it is the one topic that shows what the sign
    planner decided rather than what the chassis did about it.
    """

    sign_estimates: str
    """Sign positions the router is routing around (MarkerArray).

    Discovery estimates in a blind run, which is what the plan above was built
    from; the ground-truth signs stay on the ``track`` topic. Publishing both
    is the point -- the gap between them is a real failure mode.
    """

    robot_model: str
    """Chassis, sensor mounts and the four road wheels (MarkerArray, base_link).

    Its own topic rather than part of ``track`` because the wheels carry the
    live steering angle: they change every tick, whereas the track markers are
    cached and re-sent about once a second behind a leading DELETEALL that
    would erase them.
    """


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
    bag_recorder: BagRecorderTopics
    simulation: SimulationTopics

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
