"""Shared domain enumerations.

Canonical home for cross-context domain enums (track sections, robot
direction, risk classification, robot/runtime state). Using enums instead of
bare strings eliminates typo-prone comparisons and provides IDE autocomplete
throughout the codebase. ``shared.config.enums`` re-exports ``RiskLevel`` from
here so there is exactly one definition.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self


class _FromStringEnum(StrEnum):
    """Mixin providing a ``from_string`` classmethod to any ``StrEnum`` subclass.

    Eliminates the identical try/except boilerplate that every enum in this
    module was repeating.
    """

    @classmethod
    def from_string(cls, value: str) -> Self:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid {cls.__name__.lower()}: {value!r}. Expected one of {options}"
            ) from err


class Section(_FromStringEnum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    @property
    def capitalized(self) -> str:
        return self.value.capitalize()


class Direction(_FromStringEnum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class ScenarioType(_FromStringEnum):
    """WRO 2026 challenge type."""

    OPEN = "open"
    OBSTACLES = "obstacles"


class CorridorSide(_FromStringEnum):
    """Which of a corridor's two boundaries something is measured toward.

    Every corridor on this track is bounded by the mat's outer wall on one side
    and a face of the inner block on the other, whichever cardinal section it
    is. Naming the side rather than a compass direction keeps the meaning the
    same for all four.
    """

    INNER = "inner"
    OUTER = "outer"


class RiskLevel(StrEnum):
    """Collision risk classification for navigation logic."""

    SAFE = "safe"
    CRITICAL = "critical"
    OBSTACLE = "obstacle"


class LightingScenario(StrEnum):
    """Realistic lighting scenarios for simulation."""

    DIRECT_SUNLIGHT = "direct_sunlight"
    CLOUDY = "cloudy"
    INDOOR_BRIGHT = "indoor_bright"
    INDOOR_DIM = "indoor_dim"
    EVENING = "evening"
    MIXED = "mixed"


class RobotState(_FromStringEnum):
    """Robot state machine states."""

    BOOT_CHECK = "boot_check"
    READY = "ready"
    RACING = "racing"
    FINISHED = "finished"


class NodeHealth(_FromStringEnum):
    """Telemetry node health status."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"


class NavigatorPhase(StrEnum):
    """Which branch of ``CoreNavigator.step()`` produced a given
    ``NavigatorDebugSnapshot`` -- see that model's docstring for why a field
    being ``None`` means "not computed on this phase", not "unknown".
    """

    NOT_YET_STEPPED = "not_yet_stepped"
    NO_POSE = "no_pose"
    BLIND_CREEP = "blind_creep"
    ACTIVE_MANEUVER = "active_maneuver"
    STUCK_ESCAPE_HOLDING = "stuck_escape_holding"
    STUCK_ESCAPE_MANEUVER = "stuck_escape_maneuver"
    FINISHED_HOLD = "finished_hold"
    PARKING = "parking"
    WAYPOINT_WRAP_FALLBACK = "waypoint_wrap_fallback"
    WAYPOINT_REACHED = "waypoint_reached"
    NORMAL_DRIVE = "normal_drive"
    ESCAPE_TRIGGERED = "escape_triggered"


# The class ids the retrained GMR traffic-sign detector emits, in the order the
# checkpoint itself declares them. Confirmed by running the checkpoint over the
# per-class image folders: green_prism images predict green, red_prism predict
# red. This is the single source of truth for that order.
#
# Do NOT take it from auto-annotator's data.yaml, which says (red, green,
# magenta) and is stale -- its `path` points at an archived directory. Consuming
# the HEF with that order swaps red and green, inverting the WRO pass-side rule
# on every obstacle, and nothing about it fails loudly.
#
# Regenerate after retraining with:
#   python -c "import onnx; print(onnx.load('hailo/data/gmr.onnx').metadata_props)"
GMR_CLASS_NAMES: dict[int, str] = {
    0: "green",
    1: "magenta",
    2: "red",
}
