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
    """Servo travel and the bench-measured road-wheel angle it produces."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    servo_max_angle_deg: float
    max_wheel_angle_deg: float
    """Road-wheel angle (deg) measured on the bench at full servo lock.

    See robot.toml's ``[steering]`` comment: this is what gets measured with a
    protractor and declared, not a ratio -- the ratio is derived from it.
    """

    @property
    def linkage_ratio(self) -> float:
        """Road-wheel degrees produced per servo degree.

        Derived from the bench-measured ``max_wheel_angle_deg``, not declared
        directly -- kept for consumers that convert an arbitrary wheel angle
        to a servo angle (e.g. ``ackermann_motor_node``).
        """
        return self.max_wheel_angle_deg / self.servo_max_angle_deg

    @property
    def max_steering_angle(self) -> float:
        """Road-wheel angle at full lock (radians).

        Not a free parameter: it is whatever the steering hardware produces
        through the linkage -- see robot.toml's ``[ackermann]`` comment.
        """
        return math.radians(self.max_wheel_angle_deg)


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
            data: dict[str, object] = tomllib.load(f)
        return cls.model_validate(data)
