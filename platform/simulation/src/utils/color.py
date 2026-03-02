from __future__ import annotations

from src.config.constants import TrafficSignSpecs
from src.config.enums import Direction, Section


def rgb_to_normalized(r: int, g: int, b: int) -> tuple[float, float, float]:
    """Convert RGB(0-255) to normalized RGB(0-1).

    Args:
        r: Red channel value in range 0-255.
        g: Green channel value in range 0-255.
        b: Blue channel value in range 0-255.

    Returns:
        Tuple of (r, g, b) in range 0-1.
    """
    return (r / 255.0, g / 255.0, b / 255.0)


def normalized_to_rgb(r: float, g: float, b: float) -> tuple[int, int, int]:
    """Convert normalized RGB(0-1) to RGB(0-255).

    Args:
        r: Red channel value in range 0-1.
        g: Green channel value in range 0-1.
        b: Blue channel value in range 0-1.

    Returns:
        Tuple of (r, g, b) in range 0-255.
    """
    return (int(r * 255), int(g * 255), int(b * 255))


def get_section_string(section: Section | str) -> str:
    """Return the lowercase string value of a Section enum or string.

    Args:
        section: Section enum member or raw string.

    Returns:
        Lowercase string representation.
    """
    if isinstance(section, Section):
        return section.value
    return str(section).lower()


def get_direction_string(direction: Direction | str) -> str:
    """Return the lowercase string value of a Direction enum or string.

    Args:
        direction: Direction enum member or raw string.

    Returns:
        Lowercase string representation.
    """
    if isinstance(direction, Direction):
        return direction.value
    return str(direction).lower()


def get_grid_positions() -> list[tuple[float, float]]:
    """Return all 6 grid intersection positions for a corridor.

    Returns:
        List of (depth, width) tuples representing grid intersection positions.
    """
    depths = [
        TrafficSignSpecs.GRID_DEPTH_NEAR,
        TrafficSignSpecs.GRID_DEPTH_MIDDLE,
        TrafficSignSpecs.GRID_DEPTH_FAR,
    ]
    widths = [
        TrafficSignSpecs.GRID_WIDTH_OUTER,
        TrafficSignSpecs.GRID_WIDTH_INNER,
    ]
    return [(d, w) for d in depths for w in widths]
