"""Motor configuration, sourced from ``motors.toml`` (+ env overrides)."""

from __future__ import annotations

from typing import Any, ClassVar

from pydantic_settings import SettingsConfigDict
from shared.config.constants import RobotSpecs
from shared.config.defaults_model import DefaultsModel
from shared.config.generated.hardware.motors.motors_schema import (
    Drive as GeneratedDrive,
    HardwareMotorsMotors,
    Steering as GeneratedSteering,
)

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class MotorSteeringConfig(DefaultsModel, GeneratedSteering):
    """Steering configuration.

    Subclasses the generated ``Steering`` DTO for the motors.toml-backed group.
    ``port`` is env-only (device paths stay in ``.env``), while
    ``max_steering_angle`` and ``linkage_ratio`` are physical facts read from
    ``robot.toml`` through :class:`RobotSpecs` -- not motors.toml keys -- so all
    three are wrapper behaviour rather than schema fields.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "offset": 0.0,
        "centering_speed": 20,
        "turning_speed": 30,
        "reversed": False,
    }

    port: str
    """Port for the steering motor (identifier for its controller: serial port, I2C address, ...)."""

    @property
    def max_steering_angle(self) -> float:
        """Maximum SERVO angle (absolute value) in degrees, from ``robot.toml``.

        Commands beyond +/-this are clamped for safety. It used to default to
        45.0, which is not this robot: the servo reaches 90, and the
        navigator's own steering limit is derived from that number, so a stale
        default here described a different chassis from the one the planner
        assumed.
        """
        return RobotSpecs.SERVO_MAX_ANGLE_DEG

    @property
    def linkage_ratio(self) -> float:
        """Road-wheel degrees produced per servo degree, from ``robot.toml``.

        Everything upstream -- ``/ackermann_cmd``, the navigator, the simulator
        -- speaks in WHEEL angles, per the ROS convention; only the servo speaks
        servo angles. Without this conversion the node fed a wheel angle
        straight to the servo and the wheels under-turned by ~22%.
        """
        return RobotSpecs.LINKAGE_RATIO


class MotorDriveConfig(DefaultsModel, GeneratedDrive):
    """Drive configuration, from motors.toml's ``[drive]`` table (incl. ``test_duration``)."""

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "reversed": False,
        "encoder_reversed": False,
        "speed_scale": 1.0,
    }

    port: str
    """Port for the drive motor (identifier for its controller: serial port, I2C address, ...)."""


class Config(HardwareBaseSettings, HardwareMotorsMotors):
    """Motor configuration.

    Subclasses the generated ``HardwareMotorsMotors`` DTO for the TOML-backed
    ``steering``/``drive`` groups. The annotations below select the wrapper
    subclasses for each group (which inherit, not redeclare, the generated
    fields); without them the derived ``max_steering_angle``/``linkage_ratio``
    would be unreachable.
    """

    model_config = SettingsConfigDict(
        env_prefix="motor_",
        # "__" (not "_") so nested leaf names containing underscores parse
        # correctly, e.g. MOTOR_DRIVE__MIN_SPEED -> drive.min_speed. A
        # default_factory would construct the nested model with zero arguments
        # and bypass this resolution, so the groups stay required.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "motors" / "motors.toml",
    )

    steering: MotorSteeringConfig
    drive: MotorDriveConfig
