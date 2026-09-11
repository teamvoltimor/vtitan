"""Robot-corridor classification from a world position.

A single pure function mapping an (x, y) point to the cardinal corridor
section it currently occupies, used by navigation and sign-routing code that
needs to know which corridor the chassis is in.
"""

from __future__ import annotations

from shared.config.constants import TrackDimensions
from shared.domain.enums import Section


def corridor_for_position(x: float, y: float) -> Section:
    """Classify which corridor section the robot is currently in.

    Uses the fixed inner-square boundaries (1.0-2.0 in both axes) to assign a
    cardinal section. In corner zones (both x and y outside the inner square
    range simultaneously), the nearest boundary face determines the section.

    Args:
        x: Robot world X position (metres).
        y: Robot world Y position (metres).

    Returns:
        Section enum for the current corridor.
    """
    in_x = TrackDimensions.CORNER_MIN <= x <= TrackDimensions.CORNER_MAX
    in_y = TrackDimensions.CORNER_MIN <= y <= TrackDimensions.CORNER_MAX

    if y < TrackDimensions.CORNER_MIN and in_x:
        return Section.SOUTH
    if y > TrackDimensions.CORNER_MAX and in_x:
        return Section.NORTH
    if x > TrackDimensions.CORNER_MAX and in_y:
        return Section.EAST
    if x < TrackDimensions.CORNER_MIN and in_y:
        return Section.WEST

    # Corner: classify by nearest inner-boundary face. Dict insertion order
    # (S, N, E, W) preserves the original tie-break.
    face_distances = {
        Section.SOUTH: abs(y - TrackDimensions.CORNER_MIN),
        Section.NORTH: abs(y - TrackDimensions.CORNER_MAX),
        Section.EAST: abs(x - TrackDimensions.CORNER_MAX),
        Section.WEST: abs(x - TrackDimensions.CORNER_MIN),
    }
    return min(face_distances, key=lambda section: face_distances[section])
