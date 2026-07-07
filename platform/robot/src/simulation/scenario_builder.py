"""Build valid Open Challenge scenario metadata dicts (same schema as simgen).

Shared by the headless test battery (``tests/unit/test_open_challenge_sim.py``)
and the live Gazebo/RViz visualizer (``visualize_scenario.py``) so both drive
the navigator from the exact same start-pose math — no risk of the visualizer
silently drifting from what the tests actually validate.
"""

from __future__ import annotations

import math
from typing import Any

from shared.config.constants import CorridorDimensions, ParkingLotSpecs, RobotSpecs, TrackDimensions
from shared.config.enums import Direction, Section

from src.navigation.planning.waypoints import _OUTER_WALL_BIAS
from src.navigation.race_tracker import _TRAVEL_DIRS

_TRACK_MAX = TrackDimensions.MAX_COORD
_TRACK_CENTER = _TRACK_MAX / 2
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_PARKING_BLOCK_SPACING = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.WIDTH


def start_pose(
    section: Section, direction: Direction, widths_m: dict[str, float],
) -> tuple[float, float, float]:
    """Spawn pose on the biased corridor centerline, aligned with travel."""
    south_cy = widths_m["south"] / 2 - _OUTER_WALL_BIAS
    north_cy = _TRACK_MAX - widths_m["north"] / 2 + _OUTER_WALL_BIAS
    east_cx = _TRACK_MAX - widths_m["east"] / 2 + _OUTER_WALL_BIAS
    west_cx = widths_m["west"] / 2 - _OUTER_WALL_BIAS
    center = {
        Section.SOUTH: (_TRACK_CENTER, south_cy),
        Section.NORTH: (_TRACK_CENTER, north_cy),
        Section.EAST: (east_cx, _TRACK_CENTER),
        Section.WEST: (west_cx, _TRACK_CENTER),
    }[section]
    nx, ny = _TRAVEL_DIRS[(section, direction)]
    return center[0], center[1], math.atan2(ny, nx)


def build_open_metadata(
    widths_mm: dict[str, int],
    section: Section,
    direction: Direction,
    scenario_id: int = 0,
) -> dict[str, Any]:
    """Construct a valid Open Challenge metadata dict (same schema as simgen)."""
    widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
    sx, sy, yaw = start_pose(section, direction, widths_m)
    return {
        "scenario_id": scenario_id,
        "challenge_type": "open",
        "seed": None,
        "num_signs": 0,
        "has_parking_lot": False,
        "parking_lot": None,
        "sign_positions": [],
        "corridor_widths": {
            side: {
                "type": "narrow" if widths_mm[side] == _NARROW_MM else "wide",
                "width_mm": widths_mm[side],
            }
            for side in ("north", "south", "east", "west")
        },
        "starting_conditions": {
            "direction": str(direction),
            "section": section.capitalized,
            "position": {"x": sx, "y": sy},
            "yaw": yaw,
        },
    }


def uniform_widths(mm: int) -> dict[str, int]:
    """All four corridor sides at the same width (millimetres)."""
    return dict.fromkeys(("north", "south", "east", "west"), mm)


def sign_world_pos(
    section: Section, depth_frac: float, lane_frac: float, widths_m: dict[str, float],
) -> tuple[float, float]:
    """World (x, y) for a sign at ``depth_frac`` along, ``lane_frac`` across a corridor.

    ``depth_frac`` runs 0->1 along the direction of travel around the track.
    ``lane_frac`` runs 0->1 across the corridor band, 0 at the inner-block edge
    and 1 at the outer-wall edge — matching the same corridor geometry
    ``TrackModel``/``TrackWalls`` build from ``widths_m``.
    """
    w = widths_m[section.value]
    if section is Section.SOUTH:
        return depth_frac * _TRACK_MAX, lane_frac * w
    if section is Section.NORTH:
        return depth_frac * _TRACK_MAX, _TRACK_MAX - lane_frac * w
    if section is Section.EAST:
        return _TRACK_MAX - lane_frac * w, depth_frac * _TRACK_MAX
    return lane_frac * w, depth_frac * _TRACK_MAX  # WEST


def parking_lot_dict(section: Section, depth_center: float = _TRACK_CENTER) -> dict[str, Any]:
    """Two parking blocks spaced for the chassis to fit between, centred on ``depth_center``.

    Mirrors the known-good fixture layout in ``tests/test_constants.py``
    (``PARKING_SOUTH_BLOCK1``/``2`` etc.): blocks sit ``ParkingLotSpecs.WALL_OFFSET``
    in from the outer wall, spaced ``BLOCK_SPACING_FACTOR * RobotSpecs.WIDTH`` apart.
    """
    half_gap = _PARKING_BLOCK_SPACING / 2
    offset = ParkingLotSpecs.WALL_OFFSET
    if section is Section.SOUTH:
        b1, b2 = (depth_center - half_gap, offset), (depth_center + half_gap, offset)
    elif section is Section.NORTH:
        b1, b2 = (depth_center - half_gap, _TRACK_MAX - offset), (depth_center + half_gap, _TRACK_MAX - offset)
    elif section is Section.EAST:
        b1, b2 = (_TRACK_MAX - offset, depth_center - half_gap), (_TRACK_MAX - offset, depth_center + half_gap)
    else:  # WEST
        b1, b2 = (offset, depth_center - half_gap), (offset, depth_center + half_gap)
    return {
        "block1_position": {"x": b1[0], "y": b1[1]},
        "block2_position": {"x": b2[0], "y": b2[1]},
    }


def build_obstacles_metadata(
    widths_mm: dict[str, int],
    section: Section,
    direction: Direction,
    sign_positions: list[dict[str, Any]],
    parking_lot: dict[str, Any] | None = None,
    scenario_id: int = 0,
) -> dict[str, Any]:
    """Construct a valid Obstacles Challenge metadata dict (same schema as simgen).

    ``sign_positions`` entries are ``{"x": float, "y": float, "color": "red"|"green"}``
    — resolve them with :func:`sign_world_pos`. ``parking_lot`` is the dict
    :func:`parking_lot_dict` returns, or ``None`` for no parking maneuver.
    """
    widths_m = {k: v / 1000.0 for k, v in widths_mm.items()}
    sx, sy, yaw = start_pose(section, direction, widths_m)
    return {
        "scenario_id": scenario_id,
        "challenge_type": "obstacles",
        "seed": None,
        "num_signs": len(sign_positions),
        "has_parking_lot": parking_lot is not None,
        "parking_lot": parking_lot,
        "sign_positions": sign_positions,
        "corridor_widths": {
            side: {
                "type": "narrow" if widths_mm[side] == _NARROW_MM else "wide",
                "width_mm": widths_mm[side],
            }
            for side in ("north", "south", "east", "west")
        },
        "starting_conditions": {
            "direction": str(direction),
            "section": section.capitalized,
            "position": {"x": sx, "y": sy},
            "yaw": yaw,
        },
    }
