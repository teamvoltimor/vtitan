"""Parallel-park controller for the WRO 2026 obstacles challenge.

Public API for driving the robot into the bay between the two parking blocks.
The implementation is split into submodules -- context (tuning constants),
zone (bay geometry), footprint (containment/collision predicates), controller
(the two-phase state machine) -- and this package re-exports the names that
callers and tests import from ``...maneuvers.parking``.
"""

from src.navigation.maneuvers.parking.context import (
    DEFAULT_PARKING_CONTEXT,
    ParkingConstants,
    ParkingContext,
)
from src.navigation.maneuvers.parking.controller import (
    ParkCommand,
    ParkController,
    bearing_error,
    inside_zone,
    park_controller_from_metadata,
    parking_lot_from_in_bay_start,
    pure_pursuit_steer,
)
from src.navigation.maneuvers.parking.footprint import (
    chassis_corners,
    footprint_breaches_markers,
    footprint_breaches_wall,
    footprint_inside,
    is_beyond_lot_centre,
)
from src.navigation.maneuvers.parking.geometry import normalise_angle
from src.navigation.maneuvers.parking.zone import (
    ParkZone,
    build_zone,
    staging_pos,
)

__all__ = [
    "DEFAULT_PARKING_CONTEXT",
    "ParkCommand",
    "ParkController",
    "ParkZone",
    "ParkingConstants",
    "ParkingContext",
    "bearing_error",
    "build_zone",
    "chassis_corners",
    "footprint_breaches_markers",
    "footprint_breaches_wall",
    "footprint_inside",
    "inside_zone",
    "is_beyond_lot_centre",
    "normalise_angle",
    "park_controller_from_metadata",
    "parking_lot_from_in_bay_start",
    "pure_pursuit_steer",
    "staging_pos",
]
