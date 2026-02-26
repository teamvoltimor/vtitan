"""WRO 2026 base track SDF generator.

Programmatically generates ``worlds/wro_track_2026.sdf`` from WRO spec values.
All dimensions follow the official WRO 2026 Future Engineers specifications.

Usage:
    python -m src.generation.track_generator
    python -m src.generation.track_generator --output worlds/wro_track_2026.sdf
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET

# ── WRO 2026 track constants ───────────────────────────────────────────────────

# Mat and track dimensions (metres)
_MAT_SIZE = 3.2      # full mat (including border)
_TRACK_SIZE = 3.0    # inner track (wall-to-wall)
_TRACK_CENTER = 1.5  # world-frame track centre

# Wall specs
_WALL_HEIGHT = 0.10         # 100 mm (WRO Spec 13.3)
_WALL_THICKNESS = 0.10      # visual thickness
_WALL_COLLISION_PAD = 0.08  # extra thickness for collision box only
_WALL_CENTER_Z = 0.05       # half-height above ground

# Corner line specs
_CORNER_INNER = 1.0   # inner corner X/Y
_CORNER_OUTER = 3.0   # outer wall X/Y
_LINE_LENGTH = 1.156  # diagonal line length (sqrt((3-2)²+(2.58-2)²) ≈ 1.156m)
_LINE_THICKNESS = 0.02
_LINE_Z = 0.0001      # flat on ground

# Corridor grid line Z-offset
_GRID_Z = 0.0001
_GRID_Z_UPPER = 0.0002  # slightly above grid lines (for marking overlays)

# Colours
_BLACK = "0.0 0.0 0.0 1"
_WHITE = "1.0 1.0 1.0 1"
_GREY_GRID = "0.6 0.6 0.6 0.5"
_GREY_SUBDIV = "0.5 0.5 0.5 0.4"
_GREY_LOGO = "0.9 0.9 0.9 0.5"
_BLUE = "0.0 0.2 1.0 1"
_ORANGE = "1.0 0.4 0.0 1"
_GREY_ZONE = "0.7 0.7 0.7 1"


def generate_track_sdf(output_path: str | Path = "worlds/wro_track_2026.sdf") -> Path:
    """Generate the base WRO 2026 track SDF file and write it to disk.

    Args:
        output_path: Destination file path.

    Returns:
        Resolved path of the written file.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    root = ET.Element("sdf", version="1.7")
    world = ET.SubElement(root, "world", name="wro_track_2026")

    _add_physics(world)
    _add_sun(world)
    _add_ambient_light(world)
    _add_ground_plane(world)
    _add_exterior_walls(world)
    _add_corner_lines(world)
    _add_example_starting_zone(world)
    _add_central_logo(world)
    _add_grid_lines(world)
    _add_corridor_subdivision_lines(world)

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=True)
    return output_path.resolve()


# ── Section builders ───────────────────────────────────────────────────────────

def _add_physics(world: ET.Element) -> None:
    physics = ET.SubElement(world, "physics", name="default_physics",
                            default="true", type="ode")
    _text(ET.SubElement(physics, "max_step_size"), "0.001")
    _text(ET.SubElement(physics, "real_time_factor"), "1.0")
    _text(ET.SubElement(physics, "real_time_update_rate"), "1000")


def _add_sun(world: ET.Element) -> None:
    light = ET.SubElement(world, "light", name="sun", type="directional")
    _text(ET.SubElement(light, "pose"), "0 0 10 0 0 0")
    _text(ET.SubElement(light, "diffuse"), "0.8 0.8 0.8 1")
    _text(ET.SubElement(light, "specular"), "0.2 0.2 0.2 1")
    _text(ET.SubElement(light, "direction"), "-0.5 -0.5 -1.0")
    _text(ET.SubElement(light, "cast_shadows"), "true")


def _add_ambient_light(world: ET.Element) -> None:
    light = ET.SubElement(world, "light", name="ambient_light", type="point")
    _text(ET.SubElement(light, "pose"), "0 0 3 0 0 0")
    _text(ET.SubElement(light, "diffuse"), "0.5 0.5 0.5 1")
    _text(ET.SubElement(light, "specular"), "0.1 0.1 0.1 1")
    atten = ET.SubElement(light, "attenuation")
    _text(ET.SubElement(atten, "range"), "20")
    _text(ET.SubElement(atten, "constant"), "0.5")
    _text(ET.SubElement(atten, "linear"), "0.01")
    _text(ET.SubElement(atten, "quadratic"), "0.001")


def _add_ground_plane(world: ET.Element) -> None:
    """White 3200×3200 mm mat (WRO Spec 13.1–13.2)."""
    model = _static_model(world, "ground",
                          pose=f"{_TRACK_CENTER} {_TRACK_CENTER} 0 0 0 0")
    link = ET.SubElement(model, "link", name="link")

    vis = ET.SubElement(link, "visual", name="visual")
    _plane_geometry(vis, _MAT_SIZE, _MAT_SIZE)
    _material(vis, _WHITE)

    col = ET.SubElement(link, "collision", name="collision")
    _plane_geometry(col, _MAT_SIZE, _MAT_SIZE)
    _friction_surface(col, mu=0.8)


def _add_exterior_walls(world: ET.Element) -> None:
    """Four BLACK 100 mm-high walls enclosing the 3000×3000 mm track (WRO Spec 13.3–13.4)."""
    half = _TRACK_SIZE / 2  # 1.5 m from centre
    offset = _WALL_THICKNESS / 2  # wall extends outward

    walls = [
        ("north", f"{_TRACK_CENTER} {_TRACK_SIZE + offset} {_WALL_CENTER_Z} 0 0 0",
         (_MAT_SIZE + _WALL_THICKNESS, _WALL_THICKNESS, _WALL_HEIGHT)),
        ("south", f"{_TRACK_CENTER} {-offset} {_WALL_CENTER_Z} 0 0 0",
         (_MAT_SIZE + _WALL_THICKNESS, _WALL_THICKNESS, _WALL_HEIGHT)),
        ("east",  f"{_TRACK_SIZE + offset} {_TRACK_CENTER} {_WALL_CENTER_Z} 0 0 0",
         (_WALL_THICKNESS, _MAT_SIZE + _WALL_THICKNESS, _WALL_HEIGHT)),
        ("west",  f"{-offset} {_TRACK_CENTER} {_WALL_CENTER_Z} 0 0 0",
         (_WALL_THICKNESS, _MAT_SIZE + _WALL_THICKNESS, _WALL_HEIGHT)),
    ]
    for name, pose, (sx, sy, sz) in walls:
        model = _static_model(world, f"exterior_wall_{name}", pose=pose)
        link = ET.SubElement(model, "link", name="link")

        vis = ET.SubElement(link, "visual", name="visual")
        _box_geometry(vis, sx, sy, sz)
        _material(vis, _BLACK)

        col = ET.SubElement(link, "collision", name="collision")
        _box_geometry(col, sx + _WALL_COLLISION_PAD, sy + _WALL_COLLISION_PAD, sz)
        _contact_surface(col)


def _add_corner_lines(world: ET.Element) -> None:
    """Eight diagonal corner lines (orange/blue, 30°/60°) per WRO Spec 13.9.

    Each inner corner has two lines radiating at 30° and 60°, each 1156 mm long,
    terminating at the exterior wall. The lines lie flat on the track surface.
    """
    # (model_name, cx, cy, angle_deg, colour)
    corners = [
        # NE corner (inner corner at 2.0, 2.0)
        ("corner_ne_blue",   2.50, 2.29,  30.0, _BLUE),
        ("corner_ne_orange", 2.29, 2.50,  60.0, _ORANGE),
        # SE corner (inner corner at 2.0, 1.0)
        ("corner_se_orange", 2.50, 0.71, -30.0, _ORANGE),
        ("corner_se_blue",   2.29, 0.50, -60.0, _BLUE),
        # SW corner (inner corner at 1.0, 1.0)
        ("corner_sw_blue",   0.50, 0.71, -150.0, _BLUE),
        ("corner_sw_orange", 0.71, 0.50, -120.0, _ORANGE),
        # NW corner (inner corner at 1.0, 2.0)
        ("corner_nw_orange", 0.50, 2.29,  150.0, _ORANGE),
        ("corner_nw_blue",   0.71, 2.50,  120.0, _BLUE),
    ]
    for name, cx, cy, angle_deg, colour in corners:
        angle_rad = math.radians(angle_deg)
        pose = f"{cx} {cy} {_LINE_Z} 0 0 {angle_rad:.3f}"
        model = _static_model(world, name, pose=pose)
        link = ET.SubElement(model, "link", name="link")
        vis = ET.SubElement(link, "visual", name="visual")
        _box_geometry(vis, _LINE_LENGTH, _LINE_THICKNESS, 0.001)
        _material(vis, colour)


def _add_example_starting_zone(world: ET.Element) -> None:
    """Example grey starting zone (200×500 mm) in the south section (WRO Spec 13.10–13.11).

    The generator replaces this with a dynamically positioned zone per scenario.
    This serves as a visual reference in the base template.
    """
    model = _static_model(world, "starting_zone_south",
                          pose=f"1.5 0.5 {_GRID_Z_UPPER} 0 0 0")
    link = ET.SubElement(model, "link", name="link")
    vis = ET.SubElement(link, "visual", name="visual")
    _box_geometry(vis, 0.50, 0.20, 0.001)
    _material(vis, _GREY_ZONE)


def _add_central_logo(world: ET.Element) -> None:
    """800×800 mm light-grey central logo reference area."""
    model = _static_model(world, "central_logo",
                          pose=f"1.5 1.5 {_GRID_Z} 0 0 0")
    link = ET.SubElement(model, "link", name="link")
    vis = ET.SubElement(link, "visual", name="visual")
    _box_geometry(vis, 0.8, 0.8, 0.001)
    _material(vis, _GREY_LOGO)


def _add_grid_lines(world: ET.Element) -> None:
    """Four grey reference lines dividing the track into a 3×3 section grid."""
    # Vertical lines (constant X, span full Y)
    for name, x in [("grid_line_v1", 1.0), ("grid_line_v2", 2.0)]:
        model = _static_model(world, name, pose=f"{x} {_TRACK_CENTER} {_GRID_Z} 0 0 0")
        link = ET.SubElement(model, "link", name="link")
        vis = ET.SubElement(link, "visual", name="visual")
        _box_geometry(vis, 0.001, _TRACK_SIZE, 0.001)
        _material(vis, _GREY_GRID)

    # Horizontal lines (constant Y, span full X)
    for name, y in [("grid_line_h1", 1.0), ("grid_line_h2", 2.0)]:
        model = _static_model(world, name, pose=f"{_TRACK_CENTER} {y} {_GRID_Z} 0 0 0")
        link = ET.SubElement(model, "link", name="link")
        vis = ET.SubElement(link, "visual", name="visual")
        _box_geometry(vis, _TRACK_SIZE, 0.001, 0.001)
        _material(vis, _GREY_GRID)


def _add_corridor_subdivision_lines(world: ET.Element) -> None:
    """Grey subdivision lines inside each corridor (400–200–400 mm divisions).

    Each corridor has:
    - A centerline bisecting its length.
    - Two width-division lines at 400 mm and 600 mm from the inner wall.

    Lines extend only within the corridor section (1.0–2.0 m range),
    not into the 1000×1000 mm corner areas.
    """
    corridor_span = 1.0  # corridor section is 1 m wide at centre grid

    # North corridor (Y 2.0→3.0): centerline is vertical, width lines are horizontal
    _add_subdivision_north(world, corridor_span)
    # South corridor (Y 0.0→1.0): centerline is vertical, width lines are horizontal
    _add_subdivision_south(world, corridor_span)
    # East corridor (X 2.0→3.0): centerline is horizontal, width lines are vertical
    _add_subdivision_east(world, corridor_span)
    # West corridor (X 0.0→1.0): centerline is horizontal, width lines are vertical
    _add_subdivision_west(world, corridor_span)


def _add_subdivision_north(world: ET.Element, span: float) -> None:
    _corridor_line(world, "corridor_north_center",
                   cx=1.5, cy=2.5, width=0.001, height=span, is_vertical_box=False)
    _corridor_line(world, "corridor_north_width1",
                   cx=1.5, cy=2.4, width=span, height=0.001, is_vertical_box=False)
    _corridor_line(world, "corridor_north_width2",
                   cx=1.5, cy=2.6, width=span, height=0.001, is_vertical_box=False)


def _add_subdivision_south(world: ET.Element, span: float) -> None:
    _corridor_line(world, "corridor_south_center",
                   cx=1.5, cy=0.5, width=0.001, height=span, is_vertical_box=False)
    _corridor_line(world, "corridor_south_width1",
                   cx=1.5, cy=0.4, width=span, height=0.001, is_vertical_box=False)
    _corridor_line(world, "corridor_south_width2",
                   cx=1.5, cy=0.6, width=span, height=0.001, is_vertical_box=False)


def _add_subdivision_east(world: ET.Element, span: float) -> None:
    _corridor_line(world, "corridor_east_center",
                   cx=2.5, cy=1.5, width=span, height=0.001, is_vertical_box=False)
    _corridor_line(world, "corridor_east_width1",
                   cx=2.4, cy=1.5, width=0.001, height=span, is_vertical_box=False)
    _corridor_line(world, "corridor_east_width2",
                   cx=2.6, cy=1.5, width=0.001, height=span, is_vertical_box=False)


def _add_subdivision_west(world: ET.Element, span: float) -> None:
    _corridor_line(world, "corridor_west_center",
                   cx=0.5, cy=1.5, width=span, height=0.001, is_vertical_box=False)
    _corridor_line(world, "corridor_west_width1",
                   cx=0.4, cy=1.5, width=0.001, height=span, is_vertical_box=False)
    _corridor_line(world, "corridor_west_width2",
                   cx=0.6, cy=1.5, width=0.001, height=span, is_vertical_box=False)


def _corridor_line(
    world: ET.Element,
    name: str,
    cx: float,
    cy: float,
    width: float,
    height: float,
    is_vertical_box: bool,
) -> None:
    model = _static_model(world, name, pose=f"{cx} {cy} {_GRID_Z} 0 0 0")
    link = ET.SubElement(model, "link", name="link")
    vis = ET.SubElement(link, "visual", name="visual")
    _box_geometry(vis, width, height, 0.001)
    _material(vis, _GREY_SUBDIV)


# ── XML construction helpers ───────────────────────────────────────────────────

def _static_model(parent: ET.Element, name: str, pose: str) -> ET.Element:
    model = ET.SubElement(parent, "model", name=name)
    _text(ET.SubElement(model, "static"), "true")
    _text(ET.SubElement(model, "pose"), pose)
    return model


def _text(element: ET.Element, value: str) -> ET.Element:
    element.text = value
    return element


def _box_geometry(parent: ET.Element, sx: float, sy: float, sz: float) -> None:
    geom = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geom, "box")
    _text(ET.SubElement(box, "size"), f"{sx} {sy} {sz}")


def _plane_geometry(parent: ET.Element, sx: float, sy: float) -> None:
    geom = ET.SubElement(parent, "geometry")
    plane = ET.SubElement(geom, "plane")
    _text(ET.SubElement(plane, "normal"), "0 0 1")
    _text(ET.SubElement(plane, "size"), f"{sx} {sy}")


def _material(parent: ET.Element, rgba: str) -> None:
    mat = ET.SubElement(parent, "material")
    _text(ET.SubElement(mat, "ambient"), rgba)
    _text(ET.SubElement(mat, "diffuse"), rgba)


def _friction_surface(parent: ET.Element, mu: float = 0.8) -> None:
    surface = ET.SubElement(parent, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    _text(ET.SubElement(ode, "mu"), str(mu))
    _text(ET.SubElement(ode, "mu2"), str(mu))


def _contact_surface(parent: ET.Element) -> None:
    surface = ET.SubElement(parent, "surface")
    contact = ET.SubElement(surface, "contact")
    ode = ET.SubElement(contact, "ode")
    _text(ET.SubElement(ode, "kp"), "1e8")
    _text(ET.SubElement(ode, "kd"), "1000")
    _text(ET.SubElement(ode, "max_vel"), "0.0")
    _text(ET.SubElement(ode, "min_depth"), "0.0")


# ── CLI entry point ────────────────────────────────────────────────────────────

def main() -> None:
    """Generate the base WRO 2026 track SDF file."""
    parser = argparse.ArgumentParser(
        description="Generate the base WRO 2026 Gazebo track SDF."
    )
    parser.add_argument(
        "--output", default="worlds/wro_track_2026.sdf",
        help="Output SDF file path (default: worlds/wro_track_2026.sdf).",
    )
    args = parser.parse_args()

    output_path = generate_track_sdf(args.output)
    print(f"Track SDF written → {output_path}")


if __name__ == "__main__":
    main()
