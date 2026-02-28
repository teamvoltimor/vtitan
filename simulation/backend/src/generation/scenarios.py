"""WRO 2026 official 36-scenario traffic sign definitions.

Each scenario defines pillar placement in the South corridor template.
Coordinates are transformed to other corridors via apply_scenario_to_section().
"""

from __future__ import annotations

from src.config.constants import TrackDimensions
from src.config.enums import Section

# WRO 2026 official 36 predefined traffic sign scenarios.
# Each entry is a list of (color, x, y) tuples for the South corridor.
# x = depth position (1.0=near, 1.5=mid, 2.0=far)
# y = width position (0.4=outer, 0.6=inner)
SCENARIOS: dict[int, list[tuple[str, float, float]]] = {
    # Single pillar scenarios (1–12)
    1:  [("green", 1.0, 0.6)],
    2:  [("red",   1.0, 0.6)],
    3:  [("green", 1.5, 0.6)],
    4:  [("red",   1.5, 0.6)],
    5:  [("green", 2.0, 0.6)],
    6:  [("red",   2.0, 0.6)],
    7:  [("green", 1.0, 0.4)],
    8:  [("red",   1.0, 0.4)],
    9:  [("green", 1.5, 0.4)],
    10: [("red",   1.5, 0.4)],
    11: [("green", 2.0, 0.4)],
    12: [("red",   2.0, 0.4)],
    # Double pillar scenarios (13–36)
    13: [("green", 1.0, 0.4), ("green", 2.0, 0.6)],
    14: [("green", 1.0, 0.4), ("red",   2.0, 0.6)],
    15: [("red",   1.0, 0.4), ("green", 2.0, 0.6)],
    16: [("green", 1.0, 0.4), ("red",   2.0, 0.6)],
    17: [("red",   1.0, 0.4), ("green", 2.0, 0.6)],
    18: [("red",   1.0, 0.4), ("red",   2.0, 0.6)],
    19: [("green", 1.0, 0.6), ("green", 2.0, 0.4)],
    20: [("green", 1.0, 0.6), ("red",   2.0, 0.4)],
    21: [("red",   1.0, 0.6), ("green", 2.0, 0.4)],
    22: [("green", 1.0, 0.6), ("red",   2.0, 0.4)],
    23: [("red",   1.0, 0.6), ("green", 2.0, 0.4)],
    24: [("red",   1.0, 0.6), ("red",   2.0, 0.4)],
    25: [("green", 1.0, 0.6), ("green", 2.0, 0.6)],
    26: [("green", 1.0, 0.6), ("red",   2.0, 0.6)],
    27: [("red",   1.0, 0.6), ("green", 2.0, 0.6)],
    28: [("green", 1.0, 0.6), ("red",   2.0, 0.6)],
    29: [("red",   1.0, 0.6), ("green", 2.0, 0.6)],
    30: [("red",   1.0, 0.6), ("red",   2.0, 0.6)],
    31: [("green", 1.0, 0.4), ("green", 2.0, 0.4)],
    32: [("green", 1.0, 0.4), ("red",   2.0, 0.4)],
    33: [("red",   1.0, 0.4), ("green", 2.0, 0.4)],
    34: [("green", 1.0, 0.4), ("red",   2.0, 0.4)],
    35: [("red",   1.0, 0.4), ("green", 2.0, 0.4)],
    36: [("red",   1.0, 0.4), ("red",   2.0, 0.4)],
}

VALID_SCENARIO_IDS: frozenset[int] = frozenset(range(1, 37))


def apply_scenario_to_section(
    scenario_id: int,
    section: Section,
) -> list[tuple[str, float, float]]:
    """Transform scenario pillar coordinates from the South template to any corridor.

    The SCENARIOS dict defines pillar positions in South-corridor frame.
    This function rotates/mirrors them to produce correct world coordinates
    for any of the four corridors.

    Args:
        scenario_id: WRO scenario number (1–36).
        section: Target corridor section.

    Returns:
        List of (color, world_x, world_y) tuples for the target corridor.

    Raises:
        ValueError: If scenario_id is outside 1–36.
    """
    if scenario_id not in VALID_SCENARIO_IDS:
        raise ValueError(f"scenario_id must be 1–36, got {scenario_id}")

    track_max = TrackDimensions.MAX_COORD
    transformed: list[tuple[str, float, float]] = []

    for color, x_south, y_south in SCENARIOS[scenario_id]:
        if section is Section.SOUTH:
            world_x, world_y = x_south, y_south
        elif section is Section.NORTH:
            world_x = x_south
            world_y = track_max - y_south
        elif section is Section.EAST:
            world_x = track_max - y_south
            world_y = x_south
        else:  # Section.WEST
            world_x = y_south
            world_y = x_south

        transformed.append((color, world_x, world_y))

    return transformed
