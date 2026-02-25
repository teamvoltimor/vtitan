"""src.utils – Logging setup and colour helpers."""

import colorsys
import json
import logging
import sys
from datetime import datetime, timezone
from typing import Any


# ── Structured JSON logger ─────────────────────────────────────────────────────


class _JsonFormatter(logging.Formatter):
    """Emit each log record as a single JSON line."""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "ts": datetime.now(timezone.utc).isoformat(),
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
    """Return a JSON-formatting logger attached to stdout."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(_JsonFormatter())
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
    return logger


# ── Colour palette ─────────────────────────────────────────────────────────────


def _build_palette(n: int = 24) -> list[str]:
    """Golden-ratio HSV stepping palette.  Returns n hex colour strings."""
    golden = 0.6180339887498949
    h = 0.0
    out = []
    for _ in range(n):
        r, g, b = colorsys.hsv_to_rgb(h, 0.88, 0.96)
        out.append(f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}")
        h = (h + golden) % 1.0
    return out


PALETTE_HEX: list[str] = _build_palette(24)


def _hex_to_rgb(h: str) -> tuple[int, int, int]:
    """Parse a CSS hex or rgb() string to (R, G, B) ints."""
    if h.startswith("rgb"):
        parts = h.split("(")[1].split(")")[0].split(",")
        return int(float(parts[0])), int(float(parts[1])), int(float(parts[2]))
    h = h.lstrip("#")
    try:
        return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    except ValueError:
        return (255, 0, 0)


def _hex_to_bgr(h: str) -> tuple[int, int, int]:
    """Parse a CSS hex or rgb() string to (B, G, R) ints (OpenCV order)."""
    r, g, b = _hex_to_rgb(h)
    return (b, g, r)
