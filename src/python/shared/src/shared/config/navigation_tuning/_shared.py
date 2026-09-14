"""Shared helpers used by every tuning-group module in this package."""

from __future__ import annotations

from typing import Any, ClassVar

from pydantic import model_validator


class TuningModel:
    """Mixin supplying a generated-DTO-backed tuning group's shipped fallbacks.

    The generated DTOs carry the field declarations (lowercase names, types
    and descriptions) but no defaults, because the shipped values live in the
    per-group TOML. A tuning group subclass therefore names each value once in
    ``_DEFAULTS`` and this before-validator supplies them for any key omitted
    by the caller, a partial hardware-profile overlay, or a challenge overlay.

    This is the Python counterpart of the Go profile types' ``profileDefaults``
    (``src/go/internal/config/profile``): the schema comes from the generated
    DTO, the fallbacks come from the tuning layer, and neither re-declares the
    other's fields.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {}

    @model_validator(mode="before")
    @classmethod
    def _apply_defaults(cls, data: Any) -> Any:
        if not isinstance(data, dict) or not cls._DEFAULTS:
            return data
        return {**cls._DEFAULTS, **data}
