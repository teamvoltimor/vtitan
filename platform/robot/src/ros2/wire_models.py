"""Pydantic wire models for the String topics that carry structured JSON payloads.

These topics are produced by one node and consumed by another (often on a
different OS process or board), so the field names are a cross-process
contract: a rename on one side is a silent break on the other, because a JSON
lookup just misses. Modelling the payload lets both sides share one schema and
serialise/parse with ``model_dump_json`` / ``model_validate_json`` instead of
hand-rolled dicts.

Each class matches one topic's payload:

- ``RaceMetricsWire`` -- ``/race_metrics`` (state_machine_node -> oled_display_node)
- ``TelemetrySummaryWire`` -- ``/ui/telemetry_summary`` (telemetry_bridge_node -> oled_display_node)
- ``ButtonHoldWire`` -- ``/button/hold`` (button_node -> oled_display_node)

Every field is optional-with-default so a consumer never fails to decode a
frame it can still act on -- the OLED explicitly must not blank the display on
a malformed or partial message, and a producer that has not seen data yet has
nothing real to report. Callers that need to distinguish "absent" from a real
zero use ``None`` defaults and coalesce at the read site (see the OLED's use of
``target_laps`` and the telemetry fields).
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class RaceMetricsWire(BaseModel):
    """``/race_metrics`` payload."""

    laps_completed: int = 0
    # None until the state machine publishes; consumers fall back to their own
    # lap target so the "n/3" line stays honest before the first real figure.
    target_laps: int | None = None
    total_race_time: float = 0.0
    current_velocity: float = 0.0
    current_steering: float = 0.0
    gyro_yaw: float = 0.0
    current_corridor: int = 0


class TelemetrySummaryWire(BaseModel):
    """``/ui/telemetry_summary`` payload."""

    lidar_front_cm: float | None = None
    lidar_left_cm: float | None = None
    lidar_right_cm: float | None = None
    gyro_yaw_deg: float | None = None
    best_detection_class_id: int | str | None = None
    best_detection_confidence: float | None = None


class ButtonHoldThreshold(BaseModel):
    """One ``/button/hold`` hold-threshold entry."""

    at: float
    kind: str


class ButtonHoldWire(BaseModel):
    """``/button/hold`` payload."""

    held_sec: float = 0.0
    thresholds: list[ButtonHoldThreshold] = Field(default_factory=list)