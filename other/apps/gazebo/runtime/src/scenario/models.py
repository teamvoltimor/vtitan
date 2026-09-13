"""Frozen dataclasses mirroring the simgen JSON metadata schema.

These replace raw ``dict[str, Any]`` access throughout tests and the recording
pipeline.  Every field maps 1-to-1 to a key in the Go generator's Metadata
struct — the field names intentionally match the JSON keys.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from shared.domain.enums import Direction, ScenarioType, Section


@dataclass(frozen=True, slots=True)
class Position:
    """2-D world coordinate (meters, bottom-left origin)."""

    x: float
    y: float


@dataclass(frozen=True, slots=True)
class CorridorWidth:
    """Width specification for one corridor section."""

    type: str  # "narrow" | "wide" | "fixed"
    width_mm: int  # 600 or 1000

    @property
    def width_m(self) -> float:
        return self.width_mm / 1000.0


@dataclass(frozen=True, slots=True)
class StartingConditions:
    """Robot starting configuration for one scenario."""

    section: Section
    direction: Direction
    position: Position
    yaw: float


@dataclass(frozen=True, slots=True)
class ScenarioMetadata:
    """Complete metadata record produced by simgen for one world SDF."""

    scenario_id: int
    challenge_type: ScenarioType
    seed: int | None
    num_signs: int
    has_parking_lot: bool
    corridor_widths: dict[Section, CorridorWidth]
    starting_conditions: StartingConditions

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> ScenarioMetadata:
        """Parse a simgen metadata JSON dict into a typed ScenarioMetadata."""
        sc = raw["starting_conditions"]
        return cls(
            scenario_id=raw["scenario_id"],
            challenge_type=ScenarioType.from_string(raw["challenge_type"]),
            seed=raw.get("seed"),
            num_signs=raw["num_signs"],
            has_parking_lot=raw["has_parking_lot"],
            corridor_widths={
                Section.from_string(k): CorridorWidth(
                    type=v["type"],
                    width_mm=v["width_mm"],
                )
                for k, v in raw["corridor_widths"].items()
            },
            starting_conditions=StartingConditions(
                section=Section.from_string(sc["section"]),
                direction=Direction.from_string(sc["direction"]),
                position=Position(x=sc["position"]["x"], y=sc["position"]["y"]),
                yaw=sc["yaw"],
            ),
        )

    def corridor_width_m(self, section: Section) -> float:
        """Return the corridor width in metres for the given section."""
        return self.corridor_widths[section].width_m
