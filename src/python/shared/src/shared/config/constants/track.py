"""Track/mat physical and visual constants, sourced from track.toml."""

from __future__ import annotations

import math
from typing import Final

from shared.config.constants._shared import _track
from shared.domain.enums import Section


class TrackDimensions:
    """Official WRO track dimensions (meters).

    Sourced from src/config/track.toml via TrackConstants - see
    shared.config.track_constants. Do not hand-edit these values here; edit
    the TOML directly.
    """

    MAT_SIZE: Final[float] = _track.track.mat_size
    TRACK_SIZE: Final[float] = _track.track.size
    MIN_COORD: Final[float] = _track.track.min_coord
    MAX_COORD: Final[float] = _track.track.max_coord
    CENTER_COORD: Final[float] = _track.track.center_coord
    CORNER_MIN: Final[float] = _track.track.corner_min
    CORNER_MAX: Final[float] = _track.track.corner_max
    CORNER_SIZE: Final[float] = _track.track.corner_size


class WallSpecs:
    """Wall dimensions and positioning.

    Sourced from track.toml - see TrackDimensions.
    """

    HEIGHT: Final[float] = _track.wall.height
    THICKNESS: Final[float] = _track.wall.thickness
    COLLISION_THICKNESS: Final[float] = _track.wall.collision_thickness
    EXTERIOR_OFFSET: Final[float] = _track.wall.exterior_offset
    INTERIOR_OFFSET: Final[float] = _track.wall.interior_offset
    COLOR: Final[tuple[float, float, float]] = _track.wall.color


class CorridorDimensions:
    """Corridor width specifications for different challenges.

    Sourced from track.toml - see TrackDimensions.
    """

    NARROW: Final[float] = _track.corridor.narrow
    WIDE: Final[float] = _track.corridor.wide
    MIN_WIDTH: Final[float] = _track.corridor.min_width
    MAX_WIDTH: Final[float] = _track.corridor.max_width
    OBSTACLES_WIDTH: Final[float] = _track.corridor.obstacles
    DIVISION_OUTER: Final[float] = _track.corridor.division_lines[0]
    DIVISION_INNER: Final[float] = _track.corridor.division_lines[-1]
    DIVISION_WIDTH: Final[float] = _track.corridor.division_width


class TrafficSignSpecs:
    """Official WRO traffic sign dimensions and colors.

    Sourced from track.toml - see TrackDimensions.
    """

    WIDTH: Final[float] = _track.sign.width
    DEPTH: Final[float] = _track.sign.depth
    HEIGHT: Final[float] = _track.sign.height
    Z_POSITION: Final[float] = _track.sign.z_position
    RED_COLOR: Final[tuple[float, float, float]] = _track.sign.red_color
    GREEN_COLOR: Final[tuple[float, float, float]] = _track.sign.green_color
    RED_STD: Final[tuple[float, float, float]] = _track.sign.red_std
    GREEN_STD: Final[tuple[float, float, float]] = _track.sign.green_std
    GRID_DEPTH_NEAR: Final[float] = _track.sign.grid_depth_near
    GRID_DEPTH_MIDDLE: Final[float] = _track.sign.grid_depth_middle
    GRID_DEPTH_FAR: Final[float] = _track.sign.grid_depth_far
    GRID_WIDTH_OUTER: Final[float] = _track.corridor.division_lines[0]
    GRID_WIDTH_INNER: Final[float] = _track.corridor.division_lines[-1]
    PLACEMENT_CIRCLE_DIAMETER: Final[float] = _track.sign.placement_circle_diameter
    MAX_LEGAL_DISPLACEMENT_M: Final[float] = math.sqrt(
        (PLACEMENT_CIRCLE_DIAMETER / 2) ** 2 - (WIDTH / 2) ** 2,
    ) + (WIDTH / 2)
    MIN_SIGNS: Final[int] = _track.sign.min_count
    MAX_SIGNS: Final[int] = _track.sign.max_count


class ParkingLotSpecs:
    """Parking block dimensions and positioning (obstacles challenge only).

    Sourced from track.toml - see TrackDimensions.
    """

    LENGTH: Final[float] = _track.parking.length
    WIDTH: Final[float] = _track.parking.width
    HEIGHT: Final[float] = _track.parking.height
    Z_POSITION: Final[float] = _track.parking.z_position
    COLOR: Final[tuple[float, float, float]] = _track.parking.color
    WALL_OFFSET: Final[float] = _track.parking.wall_offset
    BLOCK_SPACING_FACTOR: Final[float] = _track.parking.spacing_factor


class StartingZoneSpecs:
    """Starting zone dimensions and visual appearance.

    Sourced from track.toml - see TrackDimensions. The starting-square
    layout (bands, spawn offsets, cell midpoints) lives in
    shared.config.starting_zone.STARTING_ZONE_LAYOUT, which validates it.
    """

    DEFAULT_LENGTH: Final[float] = _track.starting_zone.default_length
    WIDTH: Final[float] = _track.corridor.division_width
    THICKNESS: Final[float] = _track.starting_zone.thickness
    OBSTACLES_SIZE_FACTOR: Final[float] = _track.starting_zone.obstacles_size_factor
    COLOR: Final[tuple[float, float, float]] = _track.starting_zone.color
    CLOCKWISE_COLOR: Final[tuple[float, float, float]] = _track.starting_zone.clockwise_color
    COUNTERCLOCKWISE_COLOR: Final[tuple[float, float, float]] = _track.starting_zone.counterclockwise_color
    INDICATOR_RADIUS: Final[float] = _track.starting_zone.indicator_radius


class TrackMarkings:
    """Corner lines and other track markings.

    Values are sourced from track.toml's ``[markings]`` table via TrackConstants
    (shared.config.track_constants) - the single source of truth, also consumed
    by the Go simconfig generator.
    """

    ORANGE_COLOR = _track.markings.orange_color
    BLUE_COLOR = _track.markings.blue_color
    ANGLE = _track.markings.angle


class GridSections:
    """Track section definitions."""

    SECTIONS = (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST)
    SECTION_NAMES = tuple(s.value for s in SECTIONS)
    LENGTH_SECTION_LEFT = _track.cell_centers_along[0]
    LENGTH_SECTION_RIGHT = _track.cell_centers_along[1]
