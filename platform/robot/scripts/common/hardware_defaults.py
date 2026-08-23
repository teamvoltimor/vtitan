"""Shared hardware-interaction timeouts for diagnostic and run scripts.

These are process/transport-level settle and reply bounds used across the
``scripts/`` helpers (discovery waits, estop settle, service reply timeouts).
Centralised here so every script reads the same values instead of redeclaring
private ``DISCOVERY_SEC`` / ``SETTLE_*`` / ``REPLY_TIMEOUT_SEC`` module constants.
"""

from __future__ import annotations

DISCOVERY_SEC: float = 3.0
"""How long to wait for ROS discovery / a node to come up before timing out."""

SETTLE_SEC: float = 2.0
"""How long to let hardware or a node settle after a triggering action."""

ESTOP_SETTLE_SEC: float = 2.0
"""How long to wait after an estop before treating motion as stopped."""

SETTLE_TIMEOUT_S: float = 2.0
"""Generic settle timeout alias (used by motor test scripts)."""

REPLY_TIMEOUT_SEC: float = 5.0
"""How long to wait for a service / topic reply before giving up."""
