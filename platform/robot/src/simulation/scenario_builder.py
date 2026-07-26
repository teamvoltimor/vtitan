"""Build valid Open Challenge scenario metadata dicts (same schema as simgen).

Shared by the headless test battery (``tests/unit/test_open_challenge_sim.py``)
and by ``find_recovery_envelope.py``/``test_deviation_recovery.py`` so both
drive the navigator from the exact same start-pose math. The RViz visualizer
and Obstacles Challenge scenarios instead load Go-generated fixtures — see
``scenario_catalog.py``.
"""

from __future__ import annotations

import math
from typing import Any

from shared.config.constants import CorridorDimensions, TrackDimensions
from shared.config.enums import Direction, Section

from src.navigation.planning.waypoints import _OUTER_WALL_BIAS
from src.navigation.race_tracker import TRAVEL_DIRS

_TRACK_MAX = TrackDimensions.MAX_COORD
_TRACK_CENTER = _TRACK_MAX / 2
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)


def start_pose(
    section: Section,
    direction: Direction,
    widths_m: dict[str, float],
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
    nx, ny = TRAVEL_DIRS[(section, direction)]
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
