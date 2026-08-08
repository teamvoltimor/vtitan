"""Reconnect-backoff constants shared by both gRPC channels.

command_channel.py (backend->robot) and telemetry_ingest_channel.py
(robot->backend) each run their own independent reconnect loop, but both used
to declare identical ``_BACKOFF_INITIAL``/``_BACKOFF_MAX`` constants rather
than sharing one definition -- fine while they agreed, but nothing enforced
that agreement, so a future retune of one loop could silently diverge from
the other.
"""

from __future__ import annotations

BACKOFF_INITIAL_S = 1.0
"""Delay before the first reconnect attempt, and after a successful connection resets."""

BACKOFF_MAX_S = 60.0
"""Ceiling for the exponential backoff (doubles each failed attempt, capped here)."""
