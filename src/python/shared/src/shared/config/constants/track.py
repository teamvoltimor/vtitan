"""Track/mat physical and visual constants, sourced from track.toml."""

from __future__ import annotations

import math
from typing import Final

from shared.config.constants._shared import _track
from shared.domain.enums import Section


class TrackDimensions:
    """Official WRO track dimensions (meters).

    Sourced from src/config/track.toml via TrackConstants — see
    shared.config.track_constants. Do not hand-edit these values here; edit
    the TOML directly.
    """

    # Mat and track sizes
    MAT_SIZE: Final[float] = _track.track.mat_size  # 3200mm mat size
    TRACK_SIZE: Final[float] = _track.track.size  # 3000mm inner track size

    # Coordinate system (bottom-left origin)
    MIN_COORD: Final[float] = _track.track.min_coord  # Minimum coordinate (bottom-left)
    MAX_COORD: Final[float] = _track.track.max_coord  # Maximum coordinate (top-right)
    CENTER_COORD: Final[float] = _track.track.center_coord  # Center of track

    # Corner section (obstacles challenge)
    CORNER_MIN: Final[float] = _track.track.corner_min  # Corner section start
    CORNER_MAX: Final[float] = _track.track.corner_max  # Corner section end
    CORNER_SIZE: Final[float] = _track.track.corner_size  # 1000mm × 1000mm corner


class WallSpecs:
    """Wall dimensions and positioning.

    Sourced from track.toml — see TrackDimensions.
    """

    HEIGHT: Final[float] = _track.wall.height  # 100mm wall height
    THICKNESS: Final[float] = _track.wall.thickness  # 100mm wall thickness (visual)
    # Collision extends 40mm per side past visual to prevent LIDAR pass-through.
    # LIDAR is 30mm ahead of chassis front, so 40mm buffer ensures the LIDAR
    # stays ≥10mm outside the visual wall face even at full contact.
    COLLISION_THICKNESS: Final[float] = _track.wall.collision_thickness

    # Wall positioning offsets
    EXTERIOR_OFFSET: Final[float] = _track.wall.exterior_offset  # Half thickness for exterior walls
    INTERIOR_OFFSET: Final[float] = _track.wall.interior_offset  # Half thickness for interior walls

    # Color (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = _track.wall.color  # Black


class CorridorDimensions:
    """Corridor width specifications for different challenges.

    Sourced from track.toml — see TrackDimensions.
    """

    # Open challenge: Variable corridor widths
    NARROW: Final[float] = _track.corridor.narrow  # 600mm narrow corridor
    WIDE: Final[float] = _track.corridor.wide  # 1000mm wide corridor

    # Validation bounds (wider range to allow test tolerance)
    MIN_WIDTH: Final[float] = _track.corridor.min_width  # Minimum valid corridor width (500mm)
    MAX_WIDTH: Final[float] = _track.corridor.max_width  # Maximum valid corridor width (1500mm)

    # Obstacles challenge: Fixed corridor width
    OBSTACLES_WIDTH: Final[float] = _track.corridor.obstacles  # 1000mm fixed width

    # Corridor division grid. DIVISION_WIDTH is derived from the two lines, not
    # a third independent number.
    DIVISION_OUTER: Final[float] = _track.corridor.division_lines[0]  # 400mm from outer wall
    DIVISION_INNER: Final[float] = _track.corridor.division_lines[-1]  # 600mm from outer wall
    DIVISION_WIDTH: Final[float] = _track.corridor.division_width  # 200mm middle section width


class TrafficSignSpecs:
    """Official WRO traffic sign dimensions and colors.

    Sourced from track.toml — see TrackDimensions.
    """

    WIDTH: Final[float] = _track.sign.width  # 50mm
    DEPTH: Final[float] = _track.sign.depth  # 50mm
    HEIGHT: Final[float] = _track.sign.height  # 100mm
    Z_POSITION: Final[float] = _track.sign.z_position  # Half height (50mm)

    # Official colors (RGB normalized 0-1)
    # WRO Spec 13.21-13.22
    RED_COLOR: Final[tuple[float, float, float]] = _track.sign.red_color  # RGB(238, 39, 55)
    GREEN_COLOR: Final[tuple[float, float, float]] = _track.sign.green_color  # RGB(68, 214, 44)

    # Color randomization (standard deviation for Gaussian noise)
    RED_STD: Final[tuple[float, float, float]] = _track.sign.red_std
    GREEN_STD: Final[tuple[float, float, float]] = _track.sign.green_std

    # Grid positions. The width lines are the corridor's own division lines —
    # CorridorDimensions.DIVISION_OUTER/INNER — not separate measurements.
    GRID_DEPTH_NEAR: Final[float] = _track.sign.grid_depth_near  # Entry position
    GRID_DEPTH_MIDDLE: Final[float] = _track.sign.grid_depth_middle  # Center position
    GRID_DEPTH_FAR: Final[float] = _track.sign.grid_depth_far  # Exit position
    GRID_WIDTH_OUTER: Final[float] = _track.corridor.division_lines[0]  # Outer division line
    GRID_WIDTH_INNER: Final[float] = _track.corridor.division_lines[-1]  # Inner division line

    # Number of signs
    PLACEMENT_CIRCLE_DIAMETER: Final[float] = _track.sign.placement_circle_diameter
    """Circle on the mat each pillar is placed within (85mm).

    Touching a pillar is NOT a failure. The pillar may be nudged, and the run
    stays valid as long as ANY corner of its square is still inside this circle
    — only pushing it fully out counts against the team. See
    ``MAX_LEGAL_DISPLACEMENT_M`` for the tolerance that follows.
    """

    MAX_LEGAL_DISPLACEMENT_M: Final[float] = math.sqrt(
        (PLACEMENT_CIRCLE_DIAMETER / 2) ** 2 - (WIDTH / 2) ** 2,
    ) + (WIDTH / 2)
    """How far a pillar may be pushed and still have a corner in its circle.

    Derived, not measured: the corner that survives longest is the one trailing
    the push, so the bound is the displacement at which even that corner leaves
    the circle. Worst case is a push along an axis (59.4mm at the official
    50mm/85mm geometry); a diagonal push tolerates more (77.9mm), so using the
    axis figure everywhere is the conservative choice.
    """

    MIN_SIGNS: Final[int] = _track.sign.min_count  # Minimum per round
    MAX_SIGNS: Final[int] = _track.sign.max_count  # Maximum per round (7 red + 7 green)


class ParkingLotSpecs:
    """Parking block dimensions and positioning (obstacles challenge only).

    Sourced from track.toml — see TrackDimensions.
    """

    # Block dimensions (meters)
    LENGTH: Final[float] = _track.parking.length  # 200mm
    WIDTH: Final[float] = _track.parking.width  # 20mm
    HEIGHT: Final[float] = _track.parking.height  # 100mm
    Z_POSITION: Final[float] = _track.parking.z_position  # Half height (50mm)

    # Color (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = _track.parking.color  # Magenta RGB(255, 0, 255)

    # Positioning
    WALL_OFFSET: Final[float] = _track.parking.wall_offset  # Half of LENGTH (100mm from wall edge)
    # Spacing = 1.5 × robot_length (bay length the robot must pull into)
    BLOCK_SPACING_FACTOR: Final[float] = _track.parking.spacing_factor


class StartingZoneSpecs:
    """Starting zone dimensions and visual appearance.

    Sourced from track.toml — see TrackDimensions. The starting-square
    layout (bands, spawn offsets, cell midpoints) lives in
    shared.config.starting_zone.STARTING_ZONE_LAYOUT, which validates it.
    """

    # Default dimensions (meters)
    DEFAULT_LENGTH: Final[float] = _track.starting_zone.default_length  # 500mm
    WIDTH: Final[float] = _track.corridor.division_width  # 200mm, the middle band
    THICKNESS: Final[float] = _track.starting_zone.thickness  # 1mm visual marker

    # Obstacles challenge adjustment
    OBSTACLES_SIZE_FACTOR: Final[float] = _track.starting_zone.obstacles_size_factor  # 90% of gap

    # Visual appearance (RGB normalized 0-1)
    COLOR: Final[tuple[float, float, float]] = _track.starting_zone.color  # Grey

    # Direction indicators
    CLOCKWISE_COLOR: Final[tuple[float, float, float]] = _track.starting_zone.clockwise_color  # Blue
    COUNTERCLOCKWISE_COLOR: Final[tuple[float, float, float]] = _track.starting_zone.counterclockwise_color  # Green
    INDICATOR_RADIUS: Final[float] = _track.starting_zone.indicator_radius  # 35mm radius


class TrackMarkings:
    """Corner lines and other track markings.

    Values are sourced from track.toml's ``[markings]`` table via TrackConstants
    (shared.config.track_constants) -- the single source of truth, also consumed
    by the Go simconfig generator.
    """

    # Corner line colors (RGB normalized 0-1)
    ORANGE_COLOR = _track.markings.orange_color  # RGB(255, 102, 0)
    BLUE_COLOR = _track.markings.blue_color  # RGB(0, 51, 255)

    # Corner line angle
    ANGLE = _track.markings.angle  # 30° from corner


class GridSections:
    """Track section definitions."""

    # All sections as a tuple (immutable — never append to this at runtime)
    SECTIONS = (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST)

    # Sections as strings (for JSON serialization)
    SECTION_NAMES = tuple(s.value for s in SECTIONS)

    # Along-corridor midpoints of the two starting cells in each band.
    # Sourced from track.toml: the track centre plus or minus half a cell.
    LENGTH_SECTION_LEFT = _track.cell_centers_along[0]  # Center of [1.0-1.5] section
    LENGTH_SECTION_RIGHT = _track.cell_centers_along[1]  # Center of [1.5-2.0] section
