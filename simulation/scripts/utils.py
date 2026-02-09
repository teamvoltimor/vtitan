from constants import TrafficSignSpecs
from enums import Direction, Section

def rgb_to_normalized(r, g, b):
    """Convert RGB(0-255) to normalized RGB(0-1)

    Args:
        r, g, b: RGB values in range 0-255

    Returns:
        Tuple of (r, g, b) in range 0-1
    """
    return (r / 255.0, g / 255.0, b / 255.0)


def normalized_to_rgb(r, g, b):
    """Convert normalized RGB(0-1) to RGB(0-255)

    Args:
        r, g, b: RGB values in range 0-1

    Returns:
        Tuple of (r, g, b) in range 0-255
    """
    return (int(r * 255), int(g * 255), int(b * 255))


def get_section_string(section):
    """Get string value from Section enum or string

    Args:
        section: Section enum or string

    Returns:
        String value (lowercase)
    """
    if isinstance(section, Section):
        return section.value
    return str(section).lower()


def get_direction_string(direction):
    """Get string value from Direction enum or string

    Args:
        direction: Direction enum or string

    Returns:
        String value (lowercase)
    """
    if isinstance(direction, Direction):
        return direction.value
    return str(direction).lower()


def get_grid_positions():
    """Get all 6 grid intersection positions for a corridor

    Returns:
        List of (depth, width) tuples representing grid positions
    """
    depths = [
        TrafficSignSpecs.GRID_DEPTH_NEAR,
        TrafficSignSpecs.GRID_DEPTH_MIDDLE,
        TrafficSignSpecs.GRID_DEPTH_FAR
    ]
    widths = [
        TrafficSignSpecs.GRID_WIDTH_OUTER,
        TrafficSignSpecs.GRID_WIDTH_INNER
    ]
    return [(d, w) for d in depths for w in widths]