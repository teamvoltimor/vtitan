"""Coordinate transformation utilities for WRO track sections.

Eliminates duplicated section-specific coordinate logic across
randomizer and builder modules by providing a single reusable
coordinate transform helper.
"""

from __future__ import annotations

import math

from shared.config.constants import ParkingLotSpecs, StartingZoneSpecs, TrackDimensions
from shared.domain.enums import Section


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle from a unit quaternion.

    Uses the standard atan2 formula for the Z-axis rotation.

    Args:
        x: X component of the quaternion.
        y: Y component of the quaternion.
        z: Z component of the quaternion.
        w: W component of the quaternion.

    Returns:
        Yaw angle in radians, in the range [-π, π].
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CoordinateTransform:
    """Handles section-aware coordinate transformations.

    The WRO track is divided into 4 sections (N, S, E, W), each with
    different coordinate systems. This class centralizes the logic for
    transforming between local section coordinates and world coordinates.
    """

    @staticmethod
    def position_in_section(
        section: Section,
        x: float,
        y: float,
    ) -> tuple[float, float]:
        """Transform (x, y) local coordinates to world coordinates for given section.

        Args:
            section: Track section (N, S, E, W).
            x: Local X coordinate.
            y: Local Y coordinate.

        Returns:
            Tuple of (world_x, world_y) coordinates.
        """
        track_max = TrackDimensions.MAX_COORD

        if section is Section.SOUTH:
            return x, y
        if section is Section.NORTH:
            return x, track_max - y
        if section is Section.EAST:
            return track_max - y, x
        # Section.WEST
        return y, x

    @staticmethod
    def zone_size_for_section(
        section: Section,
        length: float,
        width: float,
    ) -> str:
        """Return zone size string formatted for section orientation.

        North/South sections are oriented horizontally (length-first),
        while East/West are oriented vertically (width-first).

        Args:
            section: Track section.
            length: Zone length.
            width: Zone width.

        Returns:
            SDF size string: "{length} {width} {thickness}".
        """
        is_ns = section in (Section.NORTH, Section.SOUTH)
        return f"{length} {width}" if is_ns else f"{width} {length}"

    @staticmethod
    def wall_positions_for_section(
        section: Section,
    ) -> tuple[float, float]:
        """Return (wall_position, wall_offset) for section's parking/zone.

        Args:
            section: Track section.

        Returns:
            Tuple of (depth_from_entry, wall_offset_perpendicular).
        """
        track_max = TrackDimensions.MAX_COORD
        wall_offset = ParkingLotSpecs.WALL_OFFSET

        if section is Section.SOUTH:
            return 1.5, wall_offset
        if section is Section.NORTH:
            return 1.5, track_max - wall_offset
        if section is Section.EAST:
            return track_max - wall_offset, 1.5
        # Section.WEST
        return wall_offset, 1.5

    @staticmethod
    def starting_zone_center(
        section: Section,
        corridor_width: float,
    ) -> tuple[float, float]:
        """Compute the center position of the starting zone for a section.

        Args:
            section: Track section.
            corridor_width: Width of corridor in this section.

        Returns:
            Tuple of (zone_center_x, zone_center_y) in world coordinates.
        """
        track_max = TrackDimensions.MAX_COORD
        track_center = TrackDimensions.CENTER_COORD
        zone_offset = StartingZoneSpecs.DEFAULT_LENGTH / 2

        if section is Section.SOUTH:
            return track_center, corridor_width + zone_offset
        if section is Section.NORTH:
            return track_center, track_max - corridor_width - zone_offset
        if section is Section.EAST:
            return track_max - corridor_width - zone_offset, track_center
        # Section.WEST
        return corridor_width + zone_offset, track_center
