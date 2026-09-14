"""Hand-written wrapper over the generated ``RosTopics`` DTO.

The DTO (``shared.config.generated.ros_topics_schema``) is generated from
``src/model/ros_topics.schema.json`` and holds the TOML's fields and their
descriptions. Everything with behavior lives here: the ``RosMessageType``
enum (not a TOML value) and the TOML loader.

Single source of truth for ROS topic names, loaded from ros_topics.toml.
Prevents drift between telemetry_bridge_node subscriptions and what actually publishes.

The nested topic groups keep their established ``*Topics`` names as aliases of
the generated DTO classes, so the split between generated DTO and hand-written
wrapper is invisible to callers.

Example usage:
    from shared.config.ros_topics import RosTopicConfig

    topics = RosTopicConfig.load_default()
    print(topics.state_machine.state)  # "/robot_state"
"""

from __future__ import annotations

from enum import StrEnum
from typing import TYPE_CHECKING, ClassVar

from shared.config.generated.ros_topics_schema import (
    Actuators,
    BagRecorder,
    Button,
    ChallengeMode,
    Commands,
    Navigation,
    RosTopics as _RosTopicsDTO,
    Sensors,
    Simulation,
    StateMachine,
    Ui,
)
from shared.config.paths import SHARED_CONFIG_ROOT, load_toml_model

if TYPE_CHECKING:
    from pathlib import Path

# Established public names, now backed by the generated DTO classes.
ActuatorTopics = Actuators
BagRecorderTopics = BagRecorder
ButtonTopics = Button
ChallengeModeTopics = ChallengeMode
CommandTopics = Commands
NavigationTopics = Navigation
SensorTopics = Sensors
SimulationTopics = Simulation
StateMachineTopics = StateMachine
UiTopics = Ui


class RosMessageType(StrEnum):
    """ROS2 message type strings, as returned by rclpy's own graph-introspection APIs.

    Covers ``get_publisher_names_and_types_by_node``, and
    ``get_subscriptions_info_by_topic``'s ``.topic_type``. Named here so tests
    pinning a node's message-type contract don't hand-retype the
    ``"pkg_name/msg/TypeName"`` string. Not a TOML value, so it has no generated
    counterpart.
    """

    ACKERMANN_DRIVE_STAMPED = "ackermann_msgs/msg/AckermannDriveStamped"
    IMAGE = "sensor_msgs/msg/Image"
    JOINT_STATE = "sensor_msgs/msg/JointState"
    STRING = "std_msgs/msg/String"


class RosTopicConfig(_RosTopicsDTO):
    """ROS2 topic names configuration, loaded from ros_topics.toml.

    Subclasses the generated DTO purely to attach the loader; every field
    declaration is inherited from the generated ``RosTopics``.
    """

    default_config_path: ClassVar[Path] = SHARED_CONFIG_ROOT / "ros_topics.toml"
    """Path to the checked-in ros_topics.toml file."""

    @classmethod
    def load_default(cls) -> RosTopicConfig:
        """Load and validate from ros_topics.toml."""
        return load_toml_model(cls, cls.default_config_path)
