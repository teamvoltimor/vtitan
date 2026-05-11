"""Builds static track elements (interior walls and zones)."""

from __future__ import annotations

from typing import Any
from xml.etree import ElementTree as ET

from shared.config.constants import (
    DictKeys,
    ModelNames,
    StartingZoneSpecs,
    TrackDimensions,
    WallSpecs,
    ZLayers,
)
from shared.config.enums import Direction, Section

from src.generation.builders.xml_helpers import build_wall_model


class TrackBuilder:
    """Handles static track elements: walls, zones, and boundaries.

    Responsibilities:
    - Interior corridor walls
    - Starting zones with direction indicators
    - Track boundary elements
    """

    @staticmethod
    def add_interior_walls(
        world: ET.Element,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> None:
        """Append the four interior walls computed from corridor widths.

        Args:
            world: The <world> ET element to append to.
            corridor_widths: Per-section width info from ScenarioRandomizer.
        """
        track_max = TrackDimensions.MAX_COORD

        north_y = track_max - corridor_widths[Section.NORTH][DictKeys.WIDTH]
        south_y = corridor_widths[Section.SOUTH][DictKeys.WIDTH]
        east_x = track_max - corridor_widths[Section.EAST][DictKeys.WIDTH]
        west_x = corridor_widths[Section.WEST][DictKeys.WIDTH]

        walls: list[tuple[str, float, float, float, float, bool]] = [
            # (name, cx, cy, length_x, length_y, is_horizontal)
            (
                ModelNames.INTERIOR_WALL_NORTH,
                (east_x + west_x) / 2,
                north_y - WallSpecs.INTERIOR_OFFSET,
                east_x - west_x,
                WallSpecs.THICKNESS,
                True,
            ),
            (
                ModelNames.INTERIOR_WALL_SOUTH,
                (east_x + west_x) / 2,
                south_y + WallSpecs.INTERIOR_OFFSET,
                east_x - west_x,
                WallSpecs.THICKNESS,
                True,
            ),
            (
                ModelNames.INTERIOR_WALL_EAST,
                east_x - WallSpecs.INTERIOR_OFFSET,
                (north_y + south_y) / 2,
                WallSpecs.THICKNESS,
                north_y - south_y,
                False,
            ),
            (
                ModelNames.INTERIOR_WALL_WEST,
                west_x + WallSpecs.INTERIOR_OFFSET,
                (north_y + south_y) / 2,
                WallSpecs.THICKNESS,
                north_y - south_y,
                False,
            ),
        ]

        for name, cx, cy, vis_x, vis_y, horizontal in walls:
            col_x = WallSpecs.COLLISION_THICKNESS if not horizontal else vis_x
            col_y = WallSpecs.COLLISION_THICKNESS if horizontal else vis_y
            world.append(
                build_wall_model(name, cx, cy, vis_x, vis_y, col_x, col_y),
            )

    @staticmethod
    def add_starting_zone(
        world: ET.Element,
        starting_conditions: dict[str, Any],
        corridor_widths: dict[Section, dict[str, Any]],
        parking_config: dict[str, Any] | None,
    ) -> None:
        """Add the starting zone rectangle and direction indicator.

        Removes any pre-existing starting zone from the base world, then
        adds the dynamically positioned zone with a colored direction circle.

        Args:
            world: The <world> ET element.
            starting_conditions: Dict from ScenarioRandomizer.randomize_starting_conditions().
            corridor_widths: Per-section width info.
            parking_config: Parking lot positions (obstacles challenge) or None.
        """
        starting_section: Section = starting_conditions[DictKeys.SECTION]

        sz = starting_conditions["starting_zone"]
        zone_length, zone_x, zone_y = sz["length"], sz["x"], sz["y"]

        # Remove existing static zone from base template
        for existing in world.findall(".//model[@name='starting_zone_south']"):
            world.remove(existing)

        direction: Direction = starting_conditions[DictKeys.DIRECTION]
        indicator_rgb = (
            StartingZoneSpecs.CLOCKWISE_COLOR
            if direction is Direction.CLOCKWISE
            else StartingZoneSpecs.COUNTERCLOCKWISE_COLOR
        )

        is_ns_corridor = starting_section in (Section.NORTH, Section.SOUTH)
        if is_ns_corridor:
            zone_size = f"{zone_length} {StartingZoneSpecs.WIDTH} {StartingZoneSpecs.THICKNESS}"
        else:
            zone_size = f"{StartingZoneSpecs.WIDTH} {zone_length} {StartingZoneSpecs.THICKNESS}"

        zone_name = f"{ModelNames.STARTING_ZONE_PREFIX}{starting_section}"
        zone_model = ET.Element("model", name=zone_name)
        ET.SubElement(zone_model, "static").text = "true"
        ET.SubElement(
            zone_model, "pose"
        ).text = f"{zone_x} {zone_y} {ZLayers.STARTING_ZONE_BASE} 0 0 0"

        link = ET.SubElement(zone_model, "link", name="link")

        # Base grey rectangle
        vis_base = ET.SubElement(link, "visual", name="visual_base")
        geom_b = ET.SubElement(ET.SubElement(vis_base, "geometry"), "box")
        ET.SubElement(geom_b, "size").text = zone_size
        mat_b = ET.SubElement(vis_base, "material")
        c = StartingZoneSpecs.COLOR
        ET.SubElement(mat_b, "ambient").text = f"{c[0]} {c[1]} {c[2]} 1"
        ET.SubElement(mat_b, "diffuse").text = f"{c[0]} {c[1]} {c[2]} 1"

        # Colored direction indicator circle
        vis_ind = ET.SubElement(link, "visual", name="visual_direction_indicator")
        ET.SubElement(vis_ind, "pose").text = f"0 0 {ZLayers.DIRECTION_INDICATOR} 0 0 0"
        cyl = ET.SubElement(ET.SubElement(vis_ind, "geometry"), "cylinder")
        ET.SubElement(cyl, "radius").text = str(StartingZoneSpecs.INDICATOR_RADIUS)
        ET.SubElement(cyl, "length").text = str(StartingZoneSpecs.THICKNESS)
        mat_ind = ET.SubElement(vis_ind, "material")
        ic = indicator_rgb
        color_str = f"{ic[0]} {ic[1]} {ic[2]} 1"
        ET.SubElement(mat_ind, "ambient").text = color_str
        ET.SubElement(mat_ind, "diffuse").text = color_str
        ET.SubElement(mat_ind, "emissive").text = color_str

        world.append(zone_model)

        # Sync starting position with actual spawn point
        starting_conditions[DictKeys.POSITION] = (zone_x, zone_y)
