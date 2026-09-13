"""Hand-written wrapper over the generated ``RobotConfig`` DTO.

The DTO (``shared.config.generated.robot_schema``) is generated from
``src/config/schemas/robot.schema.json`` and holds only the TOML's fields and
their descriptions -- all optional, because a hardware-profile overlay declares
only the keys it changes. Everything with behavior lives here: the
``steering_limit_deg`` validator, the derived helpers (``linkage_ratio``,
``max_steering_angle``), the profile merge, and the check that a hardware
profile actually supplied the motor and servo facts.

Kept behind the same public names the codebase already imports, so the split
between generated DTO and hand-written wrapper is invisible to callers.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

from pydantic import ValidationInfo, field_validator

from shared.config.generated.robot_schema import (
    Ackermann,
    Camera,
    Chassis,
    Drivetrain,
    Imu,
    Lidar,
    RobotConfig as _RobotConfigDTO,
    Steering as _SteeringDTO,
    Wheel,
)
from shared.config.hardware_profile import PROFILES_ROOT, active_profiles
from shared.config.paths import SHARED_CONFIG_ROOT, load_toml_merged, profile_overlay_paths

if TYPE_CHECKING:
    from pathlib import Path

__all__ = [
    "Ackermann",
    "Camera",
    "Chassis",
    "Drivetrain",
    "Imu",
    "Lidar",
    "RobotConstants",
    "Steering",
    "Wheel",
]

DEFAULT_CONFIG_PATH: Path = SHARED_CONFIG_ROOT / "robot.toml"
"""src/config/robot.toml -- resolved via shared.config.paths rather
than a fragile ``parents[N]`` relative to this file."""


class Steering(_SteeringDTO):
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


_COMPONENT_FACTS: tuple[tuple[str, str, str], ...] = (
    ("drivetrain", "max_speed_mps", "a drive motor"),
    ("drivetrain", "max_accel_mps2", "a drive motor"),
    ("drivetrain", "speed_response_tau_s", "a drive motor"),
    ("steering", "servo_max_angle_deg", "a steering servo"),
    ("steering", "max_wheel_angle_deg", "a steering servo"),
)
"""Keys the base config refuses to guess, and the component that supplies each.

The base ``robot.toml`` describes the chassis, which does not change when a
motor or servo is swapped. These all do change, so requiring them from a
named profile is what stops a run from silently modelling whichever hardware
happened to be checked in -- the failure mode that put a night of motor tests
on the wrong ceiling, undetectable from behaviour alone."""


def _require_component_facts(data: dict[str, object]) -> None:
    """Fail loudly when no hardware profile supplied the motor or servo facts.

    Raised here rather than left to pydantic because pydantic's own message
    ("field required") names the field but not the cause or the cure, and the
    cure -- export ``VTITAN_HARDWARE_PROFILE`` -- is not guessable from it.

    Args:
        data: The merged base + profile mapping, before validation.

    Raises:
        ValueError: Naming every missing fact and the profiles that supply it.
    """
    missing = [
        (section, key, component)
        for section, key, component in _COMPONENT_FACTS
        if not isinstance(data.get(section), dict) or key not in data[section]  # type: ignore[index]
    ]
    if not missing:
        return

    available = sorted(p.name for p in PROFILES_ROOT.iterdir() if p.is_dir()) if PROFILES_ROOT.is_dir() else []
    wanted = ", ".join(f"[{section}] {key}" for section, key, _ in missing)
    components = sorted({component for _, _, component in missing})
    active = active_profiles()
    msg = (
        f"robot config is missing {wanted}, which {' and '.join(components)} profile(s) supply. "
        f"VTITAN_HARDWARE_PROFILE is currently {','.join(active) if active else 'unset'}. "
        f"Name one profile per component, comma-separated, e.g. "
        f"VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm. "
        f"Available profiles: {', '.join(available) if available else '(none found)'}."
    )
    raise ValueError(msg)


class RobotConstants(_RobotConfigDTO):
    """Physical constants for the robot chassis, loaded from robot.toml.

    Subclasses the generated DTO purely to attach the ``Steering`` behavior
    and the profile-merge loader; every other field declaration is inherited
    from the generated ``RobotConfig``.
    """

    steering: Steering

    default_config_path: ClassVar[Path] = DEFAULT_CONFIG_PATH

    @classmethod
    def _load_raw(cls) -> dict[str, object]:
        """Merge the base robot.toml with any active hardware-profile overlay.

        A hardware profile (``VTITAN_HARDWARE_PROFILE``, see
        :mod:`shared.config.hardware_profile`) declares only the keys it
        changes. Every other field still comes from this base file.

        The base file deliberately does NOT declare the drive motor's ceiling
        or the servo's geometry, so a profile supplying each is REQUIRED and
        :func:`_require_component_facts` raises naming what is missing when one
        is not.

        Raises:
            ValueError: If no profile supplied the motor or servo facts.
        """
        data: dict[str, object] = load_toml_merged(DEFAULT_CONFIG_PATH, overlays=profile_overlay_paths("robot.toml"))
        _require_component_facts(data)
        return data

    @classmethod
    def load_default(cls) -> RobotConstants:
        """Load and validate from robot.toml, merged with active profiles."""
        return cls.model_validate(cls._load_raw())
