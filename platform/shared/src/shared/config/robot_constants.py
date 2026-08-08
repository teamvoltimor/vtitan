"""Physical constants for the WRO 2026 robot chassis (vTitan + Ackermann steering).

Loads ``platform/shared/config/robot.toml`` directly at runtime -- the single
source of truth also consumed by the Go ``simconfig`` package and the URDF
xacro fragment (regenerated via ``task gen:robot-constants``). Python used to
read a checked-in generated module (``robot_constants_gen.py``) instead, which
duplicated the TOML into a second, driftable Python file; this reads the TOML
itself, the same way :class:`~shared.config.navigation_tuning.NavigationTuning`
reads its own TOML tree.
"""

from __future__ import annotations

import math
import tomllib
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict

DEFAULT_CONFIG_PATH: Path = Path(__file__).resolve().parents[3] / "config" / "robot.toml"
"""platform/shared/config/robot.toml -- resolved relative to this module's own
location rather than the caller's, same rationale as NavigationTuning's
DEFAULT_CONFIG_DIR."""


class Chassis(BaseModel):
    """Robot body's box dimensions and mass."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    length: float
    width: float
    height: float
    mass: float


class Ackermann(BaseModel):
    """Steering geometry shared by the drivetrain and the Gazebo Ackermann plugin."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    wheelbase: float
    track_width: float


class Steering(BaseModel):
    """Servo travel and the linkage that converts it into road-wheel angle."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    servo_max_angle_deg: float
    linkage_ratio: float

    @property
    def max_steering_angle(self) -> float:
        """Road-wheel angle at full lock (radians).

        Not a TOML field: it is whatever the steering hardware produces
        through the linkage, not a free parameter -- see robot.toml's
        ``[ackermann]`` comment.
        """
        return math.radians(self.servo_max_angle_deg * self.linkage_ratio)


class Wheel(BaseModel):
    """One wheel's dimensions and mass."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    radius: float
    width: float
    mass: float


class Drivetrain(BaseModel):
    """Drive motor's measured limits. Physical ceilings, not tuning."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    max_speed_mps: float
    max_accel_mps2: float
    rear_steer_ratio: float


class Lidar(BaseModel):
    """Slamtec C1 mount offset and orientation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_x_offset: float
    mount_z_offset: float
    inverted: bool
    mount_yaw_offset_deg: float


class Imu(BaseModel):
    """BNO085 mount offset."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_z_offset: float


class Camera(BaseModel):
    """RPi Camera Module 3 Wide mount offset and tilt."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_x_offset: float
    mount_z_offset: float
    mount_pitch: float


class RobotConstants(BaseModel):
    """Physical constants for the robot chassis, loaded from robot.toml."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    chassis: Chassis
    ackermann: Ackermann
    steering: Steering
    wheel: Wheel
    drivetrain: Drivetrain
    lidar: Lidar
    imu: Imu
    camera: Camera

    @classmethod
    def load_default(cls) -> RobotConstants:
        """Load from the checked-in ``platform/shared/config/robot.toml``."""
        with DEFAULT_CONFIG_PATH.open("rb") as f:
            data: dict[str, Any] = tomllib.load(f)
        return cls.model_validate(data)
