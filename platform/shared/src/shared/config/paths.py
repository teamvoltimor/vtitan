"""Canonical paths to the shared config tree and a TOML-model loader.

Every ``shared/config/*`` module used to recompute the config root with a
fragile ``Path(__file__).resolve().parents[N]`` — and the ``N`` differed
(``parents[3]`` vs ``parents[4]``) purely by module depth, not by intent, so a
file moved one directory would silently resolve the wrong root. This module
anchors the one true root from :mod:`shared.config.hardware_profile` (whose depth
is fixed by its import path) and offers :func:`load_toml_model` to replace the
repeated ``open('rb')`` → ``tomllib.load`` → ``model_validate`` (and optional
profile-overlay ``deep_merge``) boilerplate scattered across the config models.
"""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Self, TypeVar

from pydantic import BaseModel

from shared.config._merge import deep_merge
from shared.config.hardware_profile import PROFILES_ROOT, profile_dirs

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

# Anchored from hardware_profile.py (shared/config/hardware_profile.py) rather
# than each caller, so module relocations can't change the resolved root.
SHARED_CONFIG_ROOT: Path = PROFILES_ROOT.parent
"""platform/shared/config -- the single shared TOML root, resolved once."""

# The robot-side hardware settings live under a separate tree (driver-level
# PWM ranges etc.); kept here so callers don't re-derive it either.
ROBOT_CONFIG_ROOT: Path = SHARED_CONFIG_ROOT.parents[1] / "robot" / "config"
"""platform/robot/config -- driver/hardware TOML root (HardwareBaseSettings)."""

_ModelT = TypeVar("_ModelT", bound=BaseModel)


def load_toml_model(
    cls: type[_ModelT],
    path: Path,
    *,
    overlays: Iterable[Path] = (),
    overlay_required: bool = False,
) -> _ModelT:
    """Load *cls* from a TOML file, optionally overlaying sibling TOML files.

    Replaces the repeated ``with path.open('rb') as f: data = tomllib.load(f)``
    then ``cls.model_validate(data)`` (plus any ``deep_merge`` of profile
    overlays) seen in ``robot_constants`` / ``track_constants`` /
    ``ros_topics`` / ``navigation_tuning``.

    Args:
        cls: The pydantic model to validate the merged mapping against.
        path: Base TOML file to load.
        overlays: Extra TOML files merged on top of *path* in order (later wins
            on any key); used for hardware-profile overlays. A non-existent
            overlay path is skipped unless *overlay_required*.
        overlay_required: If true, every path in *overlays* must exist.

    Returns:
        The validated model instance.
    """
    with path.open("rb") as f:
        data: dict[str, Any] = tomllib.load(f)
    for overlay in overlays:
        if not overlay.exists():
            if overlay_required:
                msg = f"required config overlay not found: {overlay}"
                raise FileNotFoundError(msg)
            continue
        with overlay.open("rb") as f:
            data = deep_merge(data, tomllib.load(f))
    return cls.model_validate(data)


def load_toml_merged(
    path: Path,
    *,
    overlays: Iterable[Path] = (),
    overlay_required: bool = False,
) -> dict[str, Any]:
    """Load a TOML file merged with *overlays*, returning the raw mapping.

    Like :func:`load_toml_model` but without pydantic validation -- for callers
    that must inspect or patch the merged dict (e.g. assert required profile
    facts exist) before building the model.
    """
    with path.open("rb") as f:
        data: dict[str, Any] = tomllib.load(f)
    for overlay in overlays:
        if not overlay.exists():
            if overlay_required:
                msg = f"required config overlay not found: {overlay}"
                raise FileNotFoundError(msg)
            continue
        with overlay.open("rb") as f:
            data = deep_merge(data, tomllib.load(f))
    return data


def profile_overlay_paths(filename: str) -> Sequence[Path]:
    """Return *filename* within each active hardware-profile directory, in order.

    Convenience for callers (e.g. ``robot_constants``) that overlay a same-named
    TOML from every active profile onto the base file.
    """
    return [directory / filename for directory in profile_dirs()]


class TomlLoadableModel(BaseModel):
    """Base for config models loaded from a single checked-in TOML file.

    Collapses the near-identical ``load_default`` classmethod repeated across
    ``robot_constants`` / ``track_constants`` / ``ros_topics`` (and any future
    model) onto one shared implementation: a subclass sets
    :attr:`default_config_path` and inherits ``load_default``.

    Subclasses that must pre-merge profile overlays or otherwise patch the
    mapping before validation override :meth:`_load_raw` -- the default just
    opens and parses :attr:`default_config_path` with
    :func:`load_toml_merged`.
    """

    default_config_path: ClassVar[Path]

    @classmethod
    def _load_raw(cls) -> dict[str, Any]:
        """Return the (merged) TOML mapping to validate against this model."""
        return load_toml_merged(cls.default_config_path)

    @classmethod
    def load_default(cls) -> Self:
        """Load and validate this model from :attr:`default_config_path`."""
        return cls.model_validate(cls._load_raw())
