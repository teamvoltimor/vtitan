"""Quaternion-conversion settings shared by the RVC IMU backends."""

from __future__ import annotations

from typing import Any, ClassVar

from pydantic import field_validator
from shared.config.defaults_model import DefaultsModel
from shared.config.generated._models import Quaternion

_VALID_EULER_SEQUENCES = frozenset({"xyz", "zyx", "xzy", "yzx", "zxy", "yxz"})


class QuaternionConfig(DefaultsModel, Quaternion):
    """Configuration for quaternion calculation from Euler angles.

    Subclasses the generated ``Quaternion`` DTO so the TOML-backed
    ``euler_sequence``/``negate_*`` fields (and their names) come from the
    schema. The old hand-written shipped defaults are re-applied as wrapper
    fallbacks rather than by redeclaring a DTO field, and the Euler-order check
    the generated model does not carry stays as a validator.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "euler_sequence": "xyz",
        "negate_yaw": True,
        "negate_pitch": False,
        "negate_roll": True,
    }

    @field_validator("euler_sequence")
    @classmethod
    def _validate_euler_sequence(cls, value: str) -> str:
        if value not in _VALID_EULER_SEQUENCES:
            msg = f"Invalid euler_sequence '{value}'. Valid options are: {_VALID_EULER_SEQUENCES}"
            raise ValueError(msg)
        return value
