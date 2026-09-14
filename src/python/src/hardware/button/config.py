"""Button configuration shared by the GPIO and MCP2221A backends."""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.defaults_model import DefaultsModel
from shared.config.generated.hardware.button.gpio_schema import Button as GeneratedButton


class Config(DefaultsModel, GeneratedButton):
    """Configuration for button driver.

    Subclasses the generated ``Button`` DTO used by both backends' schemas; the
    old hand-written defaults are re-applied as wrapper fallbacks rather than by
    redeclaring a DTO field.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "pull_up": True,
        "debounce_ms": 50,
        "long_press_threshold_sec": 3.0,
        "shutdown_press_threshold_sec": 10.0,
    }
