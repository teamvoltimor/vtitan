"""Selects which hardware-profile overlay(s) apply on top of the base config.

A hardware profile (``src/shared/config/profiles/<name>/``, plus the
matching ``src/config/hardware/motors/profiles/<name>/`` for
driver-level settings such as the servo's PWM range) only declares the TOML
keys that differ from the checked-in base -- e.g. a different servo's
``[steering]`` geometry. Selected via ``VTITAN_HARDWARE_PROFILE``, an
ordered comma-separated list; later profiles win on any key they both set.
Empty/unset means base config only, i.e. the current hardware, unchanged.

See docs/internal/plans/2026-08-11-servo-hardware-profiles.md.
"""

from __future__ import annotations

from pathlib import Path

from pydantic_settings import BaseSettings, SettingsConfigDict

PROFILES_ROOT: Path = Path(__file__).resolve().parents[3] / "config" / "profiles"
"""src/shared/config/profiles -- resolved relative to this module's own
location, same rationale as NavigationTuning's DEFAULT_CONFIG_DIR."""


class HardwareProfileSettings(BaseSettings):
    """Reads ``VTITAN_HARDWARE_PROFILE`` (comma-separated, ordered profile names)."""

    model_config = SettingsConfigDict(env_prefix="vtitan_")

    hardware_profile: str = ""


def active_profiles() -> list[str]:
    """Ordered list of selected profile names; empty if none selected."""
    raw = HardwareProfileSettings().hardware_profile
    return [name.strip() for name in raw.split(",") if name.strip()]


def profile_dirs() -> list[Path]:
    """Resolve each active profile name to its directory under :data:`PROFILES_ROOT`.

    Raises ``ValueError`` naming the profile if ``VTITAN_HARDWARE_PROFILE``
    names one with no matching directory -- silently ignoring a typo'd
    profile would mean the robot runs on the wrong tuning with no signal
    that anything was wrong.
    """
    dirs = []
    for name in active_profiles():
        directory = PROFILES_ROOT / name
        if not directory.is_dir():
            msg = f"Unknown hardware profile {name!r}; expected a directory at {directory}"
            raise ValueError(msg)
        dirs.append(directory)
    return dirs
