"""Physical constants for the WRO 2026 robot chassis (vTitan + Ackermann steering).

Loads ``src/config/robot.toml`` directly at runtime -- the single
source of truth also consumed by the Go ``simconfig`` package and the URDF
xacro fragment (both now hand-maintained copies; the ``task gen:robot-constants``
regenerator was removed 2026-09-03). Python used to
read a checked-in generated module (``robot_constants_gen.py``) instead, which
duplicated the TOML into a second, driftable Python file; this reads the TOML
itself, the same way :class:`~shared.config.navigation_tuning.NavigationTuning`
reads its own TOML tree.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

from pydantic import BaseModel, ConfigDict, ValidationInfo, field_validator

if TYPE_CHECKING:
    from pathlib import Path

from shared.config.hardware_profile import PROFILES_ROOT, active_profiles
from shared.config.paths import SHARED_CONFIG_ROOT, TomlLoadableModel, load_toml_merged, profile_overlay_paths

DEFAULT_CONFIG_PATH: Path = SHARED_CONFIG_ROOT / "robot.toml"
"""src/config/robot.toml -- resolved via shared.config.paths rather
than a fragile ``parents[N]`` relative to this file."""


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

    min_turn_radius_intercept_m: float = 0.053
    """Turn-radius floor extrapolated to zero speed (m). See ``min_turn_radius_m``."""

    min_turn_radius_slope_s: float = 1.86
    """How fast the floor grows with speed (m per m/s). See ``min_turn_radius_m``."""

    min_turn_radius_cap_m: float = 0.35
    """Bound on the speed curve (m). NOT MEASURED -- see ``min_turn_radius_m``.

    At full lock the chassis is slow by definition, because it slows down to
    turn, so the saturation is not observable in the bags at all. This is the
    largest value the measured range (up to ~0.17 m/s) supports, carried so the
    linear term cannot run away. Above that speed the curve is extrapolation."""

    min_turn_radius_m: float = 0.29
    """Tightest turn radius the chassis can actually make (m). 0 disables the floor.

    A physical saturation, measured on hardware, not a simulator knob: every
    consumer of the bicycle model owes it the same floor, and the two that
    exist -- ``AckermannKinematics`` and ``BayExit``'s dead reckoning -- read it
    from here so they cannot drift apart again. They already had: while only the
    first honoured it, the second over-read the bay ratchet's outward travel by
    31x. See the field's note in robot.toml for the measurement.
    """

    speed_response_tau_s: float
    """First-order lag between a commanded speed and the achieved one (s).

    Separate from ``max_accel_mps2`` because they are different failures: a
    clamp bounds how fast speed may change, a lag says every change arrives
    late regardless of size. The 2026-08-29 bag shows this drivetrain obeying
    the second, so modelling it as the first (the simulator's behaviour until
    then) reaches commanded speed far too early.
    """

    yaw_gain: float
    """Fraction of the modelled yaw rate the chassis actually delivers.

    The kinematic model is zero-slip; real tyres are not. Measured, not
    assumed -- see the ``[drivetrain]`` comment in ``robot.toml``.
    """


class Lidar(BaseModel):
    """Slamtec C1 mount offset, orientation, and measurement floor."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_x_offset: float
    mount_z_offset: float
    inverted: bool
    mount_yaw_offset_deg: float

    min_range: float
    """Closest range the sensor can report (m).

    A property of the unit, so it belongs with the rest of the hardware
    description rather than in a hand-maintained Python constant -- which is
    where it lived until 2026-08-21, stated as 0.05 when the C1 measures to
    about 0.045. Anything nearer is not "no obstacle", it is unmeasurable, and
    the difference matters wherever the chassis works close to a surface:
    parking, wall contact, and the escape maneuver all operate inside a few
    centimetres.
    """

    max_range: float
    """Farthest range the sensor reports (m)."""
    samples: int
    """Horizontal sample count of one 360 deg sweep."""
    update_rate: float
    """Sweep refresh rate (Hz)."""
    noise_stddev: float
    """Per-ray range noise stddev (m)."""
    diameter: float
    """Puck diameter (m), matching the lidar_link mesh in wro_robot.urdf.xacro."""
    height: float
    """Puck height (m), matching the lidar_link mesh in wro_robot.urdf.xacro."""


class Imu(BaseModel):
    """BNO085 mount offset."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_z_offset: float
    update_rate: float
    """Measurement refresh rate (Hz)."""
    gyro_noise: float
    """Gyroscope angular-rate noise stddev (rad/s)."""
    accel_noise: float
    """Accelerometer linear-acceleration noise stddev (m/s^2)."""
    mass: float
    """Board mass (kg)."""
    size: tuple[float, float, float]
    """Board form factor (m): length x width x height."""


class Camera(BaseModel):
    """RPi Camera Module 3 Wide mount offset and tilt."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mount_x_offset: float
    mount_z_offset: float
    mount_pitch: float
    hfov: float
    """Horizontal field of view (rad)."""
    width: int
    """Sensor horizontal resolution (pixels)."""
    height: int
    """Sensor vertical resolution (pixels)."""
    update_rate: float
    """Frame capture rate (Hz)."""
    near_clip: float
    """Rendering near-clip plane (m)."""
    far_clip: float
    """Rendering far-clip plane (m)."""


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


class RobotConstants(TomlLoadableModel):
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
