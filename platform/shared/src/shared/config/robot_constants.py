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

from pydantic import BaseModel, ConfigDict, ValidationInfo, field_validator

from shared.config._merge import deep_merge
from shared.config.hardware_profile import profile_dirs

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
    """Servo travel, the road-wheel angle it produces, and how much of it we use.

    Two different numbers, deliberately separate since 2026-08-21:

    * ``max_wheel_angle_deg`` is PHYSICS -- what the linkage actually delivers at
      full servo lock, measured with a protractor. It is the only input to
      ``linkage_ratio``, so it must never be lowered to mean "steer more gently".
    * ``steering_limit_deg`` is POLICY -- how much of that travel the navigator
      is allowed to command. Lower it freely.

    They were one field until a 270 deg servo made the distinction load-bearing.
    With the 180 deg servo the two coincided at 55 deg, so nothing distinguished
    them; conflated, capping the command to 55 on an 85 deg linkage would have
    recomputed ``linkage_ratio`` as 55/135 = 0.407 instead of the true
    85/135 = 0.630, and ``ackermann_motor_node`` -- the one consumer that
    converts a wheel angle back to a servo angle -- would have driven the servo
    1.55x too far. The wheels would reach 85 deg when 55 was asked for.

    The simulator would NOT have caught it: it reads ``max_steering_angle``
    directly and never performs the servo conversion, so the error is invisible
    in every sweep and appears only on hardware.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    servo_max_angle_deg: float
    max_wheel_angle_deg: float
    """Road-wheel angle (deg) the linkage produces at full servo lock.

    See robot.toml's ``[steering]`` comment: this is what gets measured with a
    protractor and declared, not a ratio -- the ratio is derived from it. A
    hardware fact; change it only after re-measuring.
    """

    steering_limit_deg: float | None = None
    """Road-wheel angle (deg) the navigator may actually command.

    ``None`` means "use the full linkage travel", which is what every config
    predating the 270 deg servo intends -- so the field is optional and old
    TOMLs keep their exact behaviour.

    Set it to steer more gently than the hardware can. That is a real tuning
    axis rather than a safety limiter: a wider wheel angle lets the chassis cut
    corners tighter than the waypoint polyline (generated for a fixed arc shape)
    anticipates, which is the traced cause of the ``wideonly`` wall-collision
    regression -- ``select_target_point`` rejects the next waypoints as "behind"
    after a sharp cut and locks onto a distant one. Until path generation is
    turn-radius aware, the usable limit may be well below the physical maximum.
    """

    @field_validator("steering_limit_deg")
    @classmethod
    def _limit_within_linkage(cls, value: float | None, info: ValidationInfo) -> float | None:
        """Reject a commanded limit the linkage cannot reach.

        Silently clamping would hide a real config error: a limit above the
        physical maximum means somebody believes the car steers harder than it
        does, and every downstream angle would be quietly wrong.
        """
        physical = info.data.get("max_wheel_angle_deg")
        if value is None or physical is None:
            return value
        if value <= 0:
            msg = f"steering_limit_deg must be positive, got {value}"
            raise ValueError(msg)
        if value > physical:
            msg = (
                f"steering_limit_deg ({value}) exceeds the linkage's "
                f"max_wheel_angle_deg ({physical}) -- the hardware cannot reach it"
            )
            raise ValueError(msg)
        return value

    @property
    def linkage_ratio(self) -> float:
        """Road-wheel degrees produced per servo degree.

        Derived from the bench-measured ``max_wheel_angle_deg``, not declared
        directly -- kept for consumers that convert an arbitrary wheel angle
        to a servo angle (e.g. ``ackermann_motor_node``).

        Deliberately independent of ``steering_limit_deg``: the gearing does not
        change because we chose to use less of it.
        """
        return self.max_wheel_angle_deg / self.servo_max_angle_deg

    @property
    def max_steering_angle(self) -> float:
        """Largest road-wheel angle the navigator may command (radians).

        ``steering_limit_deg`` when set, otherwise the linkage's full travel.
        This is the value the planner and simulator gate on.
        """
        return math.radians(self.steering_limit_deg or self.max_wheel_angle_deg)


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
        """Load ``platform/shared/config/robot.toml``, with any active hardware profile overlaid.

        A hardware profile (``VTITAN_HARDWARE_PROFILE``, see
        :mod:`shared.config.hardware_profile`) only needs to declare the
        keys it changes -- e.g. a ``[steering]`` block for a different
        servo. Every other field still comes from this base file.
        """
        with DEFAULT_CONFIG_PATH.open("rb") as f:
            data: dict[str, object] = tomllib.load(f)
        for directory in profile_dirs():
            overlay_path = directory / "robot.toml"
            if overlay_path.exists():
                with overlay_path.open("rb") as f:
                    data = deep_merge(data, tomllib.load(f))
        return cls.model_validate(data)
