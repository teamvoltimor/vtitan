"""Shared domain enumerations.

Canonical home for cross-context domain enums (track sections, robot
direction, risk classification, robot/runtime state). Using enums instead of
bare strings eliminates typo-prone comparisons and provides IDE autocomplete
throughout the codebase. ``shared.config.enums`` re-exports ``RiskLevel`` from
here so there is exactly one definition.
"""

from __future__ import annotations

from enum import StrEnum


class Section(StrEnum):
    """Four navigable corridors of the WRO 2026 track."""

    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    @classmethod
    def from_string(cls, value: str) -> Section:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid section: {value!r}. Expected one of {options}"
            ) from err

    @property
    def capitalized(self) -> str:
        return self.value.capitalize()


class Direction(StrEnum):
    """Robot traversal direction around the WRO track."""

    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    @classmethod
    def from_string(cls, value: str) -> Direction:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(d.value for d in cls)
            raise ValueError(
                f"Invalid direction: {value!r}. Expected one of {options}"
            ) from err


class ScenarioType(StrEnum):
    """WRO 2026 challenge type."""

    OPEN = "open"
    OBSTACLES = "obstacles"

    @classmethod
    def from_string(cls, value: str) -> ScenarioType:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid scenario type: {value!r}. Expected one of {options}"
            ) from err


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


class RobotState(StrEnum):
    """Robot state machine states."""

    BOOT_CHECK = "boot_check"
    READY = "ready"
    RACING = "racing"
    FINISHED = "finished"

    @classmethod
    def from_string(cls, value: str) -> RobotState:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(s.value for s in cls)
            raise ValueError(
                f"Invalid robot state: {value!r}. Expected one of {options}"
            ) from err


class NodeHealth(StrEnum):
    """Telemetry node health status."""

    NOMINAL = "nominal"
    WATCHDOG = "watchdog"
    REPLANNING = "replanning"

    @classmethod
    def from_string(cls, value: str) -> NodeHealth:
        try:
            return cls(value.lower())
        except ValueError as err:
            options = tuple(n.value for n in cls)
            raise ValueError(
                f"Invalid node health: {value!r}. Expected one of {options}"
            ) from err


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
