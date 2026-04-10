"""Common XML element construction utilities for SDF builders."""

from __future__ import annotations

from xml.etree import ElementTree as ET


def add_box_visual(
    link: ET.Element,
    width: float,
    depth: float,
    height: float,
    color: list[float] | tuple[float, ...],
    visual_name: str = "visual",
) -> None:
    """Add a box visual element to a link.

    Args:
        link: The <link> ET element to add the visual to.
        width: Box width (X dimension).
        depth: Box depth (Y dimension).
        height: Box height (Z dimension).
        color: RGB color list/tuple normalized to [0, 1].
        visual_name: Name attribute for the visual element.
    """
    vis = ET.SubElement(link, "visual", name=visual_name)
    geom = ET.SubElement(ET.SubElement(vis, "geometry"), "box")
    ET.SubElement(geom, "size").text = f"{width} {depth} {height}"
    mat = ET.SubElement(vis, "material")
    color_str = f"{color[0]} {color[1]} {color[2]} 1"
    ET.SubElement(mat, "ambient").text = color_str
    ET.SubElement(mat, "diffuse").text = color_str


def add_box_collision(
    link: ET.Element,
    width: float,
    depth: float,
    height: float,
    collision_name: str = "collision",
) -> None:
    """Add a box collision element to a link.

    Args:
        link: The <link> ET element to add the collision to.
        width: Box width (X dimension).
        depth: Box depth (Y dimension).
        height: Box height (Z dimension).
        collision_name: Name attribute for the collision element.
    """
    col = ET.SubElement(link, "collision", name=collision_name)
    geom = ET.SubElement(ET.SubElement(col, "geometry"), "box")
    ET.SubElement(geom, "size").text = f"{width} {depth} {height}"


def build_wall_model(
    name: str,
    center_x: float,
    center_y: float,
    visual_x: float,
    visual_y: float,
    collision_x: float,
    collision_y: float,
    z_position: float = 0.0,
) -> ET.Element:
    """Construct a static wall model element.

    Args:
        name: Model name.
        center_x: X position of wall center.
        center_y: Y position of wall center.
        visual_x: Visual geometry X dimension.
        visual_y: Visual geometry Y dimension.
        collision_x: Collision geometry X dimension.
        collision_y: Collision geometry Y dimension.
        z_position: Z position of wall (default: 0.0).

    Returns:
        A constructed <model> ET element.
    """
    from shared.config.constants import WallSpecs

    model = ET.Element("model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{center_x} {center_y} {z_position} 0 0 0"

    link = ET.SubElement(model, "link", name="link")

    # Visual (thin appearance)
    add_box_visual(
        link,
        visual_x,
        visual_y,
        WallSpecs.HEIGHT,
        WallSpecs.COLOR,
        visual_name="visual",
    )

    # Collision (thicker for safety)
    add_box_collision(
        link,
        collision_x,
        collision_y,
        WallSpecs.HEIGHT,
        collision_name="collision",
    )

    return model
