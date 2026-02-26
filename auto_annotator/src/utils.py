"""src.utils – Logging setup and colour helpers.

All palette and colour constants that were previously hardcoded inline are
now kept as module-level constants so this module stays free of magic numbers.
"""

from __future__ import annotations

import colorsys
import json
import logging
import sys
from datetime import UTC, datetime
from typing import Any

from src.constants import COLOR_RED_RGB

# Palette generation constants.
# These control the golden-ratio HSV colour palette used for auto-assigning
# new class colours when no explicit colour is provided by the user.

_PALETTE_SIZE: int = 24
"""Number of distinct colours in the auto-generated class palette."""

_PALETTE_SATURATION: float = 0.88
"""HSV saturation value for palette colours (0 = grey, 1 = fully saturated)."""

_PALETTE_VALUE: float = 0.96
"""HSV brightness value for palette colours (0 = black, 1 = full brightness)."""

_PALETTE_GOLDEN_RATIO: float = 0.6180339887498949
"""Golden ratio used as the hue step between successive palette colours.

Distributes colours evenly around the hue wheel without obvious clustering.
"""


class _JsonFormatter(logging.Formatter):
    """Emit each log record as a single JSON line.

    Adds a ``ts`` (ISO-8601 UTC timestamp), ``lvl``, ``logger``, and ``msg``
    field.  Any dict stored in ``record._extra`` is merged into the payload,
    and exception tracebacks are included under ``exc`` when present.
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format *record* as a JSON string.

        Args:
            record: Log record to format.

        Returns:
            Single-line JSON string representing the log entry.
        """
        payload: dict[str, Any] = {
            "ts": datetime.now(UTC).isoformat(),
            "lvl": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        extra = getattr(record, "_extra", None)
        if extra:
            payload.update(extra)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload)


def get_logger(name: str) -> logging.Logger:
    """Return a JSON-formatting logger attached to stdout.

    Safe to call multiple times with the same *name*; handlers are only
    added on the first call.

    Args:
        name: Logger name, typically the module's ``__name__``.

    Returns:
        Configured :class:`logging.Logger` instance.
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(_JsonFormatter())
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
    return logger


def _build_palette(n: int = _PALETTE_SIZE) -> list[str]:
    """Generate a golden-ratio HSV stepping palette of *n* hex colour strings.

    Each successive hue is offset by the golden ratio so that adjacent colours
    are perceptually distant from one another.

    Args:
        n: Number of colours to generate.

    Returns:
        List of CSS hex strings, e.g. ``["#f5a623", "#7ed321", ...]``.
    """
    h = 0.0
    out: list[str] = []
    for _ in range(n):
        r, g, b = colorsys.hsv_to_rgb(h, _PALETTE_SATURATION, _PALETTE_VALUE)
        out.append(f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}")
        h = (h + _PALETTE_GOLDEN_RATIO) % 1.0
    return out


PALETTE_HEX: list[str] = _build_palette(_PALETTE_SIZE)
"""Pre-built list of ``_PALETTE_SIZE`` visually distinct hex colour strings.

Used by the Settings tab to suggest colours when adding new annotation classes.
"""


def _hex_to_rgb(h: str) -> tuple[int, int, int]:
    """Parse a CSS hex or ``rgb()`` string to an ``(R, G, B)`` int tuple.

    Accepts both ``"#rrggbb"`` and ``"rgb(r, g, b)"`` formats.  Returns
    :data:`src.constants.COLOR_RED_RGB` as a fallback when parsing fails.

    Args:
        h: Colour string in hex (``"#ee2737"``) or CSS rgb (``"rgb(238,39,55)"``) format.

    Returns:
        Three-element tuple of integer channel values in RGB order.
    """
    if h.startswith("rgb"):
        parts = h.split("(")[1].split(")", maxsplit=1)[0].split(",")
        return int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))
    h = h.lstrip("#")
    try:
        return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    except ValueError:
        return COLOR_RED_RGB


def _hex_to_bgr(h: str) -> tuple[int, int, int]:
    """Parse a CSS hex or ``rgb()`` string to a ``(B, G, R)`` int tuple (OpenCV order).

    Convenience wrapper around :func:`_hex_to_rgb` that reverses the channel order
    for use with OpenCV functions that expect BGR rather than RGB.

    Args:
        h: Colour string in hex or CSS rgb format.

    Returns:
        Three-element tuple of integer channel values in BGR order.
    """
    r, g, b = _hex_to_rgb(h)
    return (b, g, r)
