"""Parallel-park controller for the WRO 2026 obstacles challenge.

Public API for driving the robot into the bay between the two parking blocks.
The implementation is split into submodules -- context (tuning constants),
zone (bay geometry), footprint (containment/collision predicates), controller
(the two-phase state machine) -- and this package re-exports the names that
callers and tests import from ``...maneuvers.parking``.
"""

from src.navigation.maneuvers.parking.context import (
    _DEFAULT_PARKING_CONTEXT,
    ParkingContext,
    _ParkingConstants,
)
from src.navigation.maneuvers.parking.controller import (
    ParkCommand,
    ParkController,
    _bearing_error,
    _inside_zone,
    _pure_pursuit_steer,
    park_controller_from_metadata,
)
from src.navigation.maneuvers.parking.footprint import (
    _chassis_corners,
    _footprint_breaches_markers,
    _footprint_breaches_wall,
    _footprint_inside,
    _is_beyond_lot_centre,
)
from src.navigation.maneuvers.parking.geometry import _normalise_angle
from src.navigation.maneuvers.parking.zone import (
    ParkZone,
    _build_zone,
    _staging_pos,
)

__all__ = [
    "_DEFAULT_PARKING_CONTEXT",
    "ParkCommand",
    "ParkController",
    "ParkZone",
    "ParkingContext",
    "_ParkingConstants",
    "_bearing_error",
    "_build_zone",
    "_chassis_corners",
    "_footprint_breaches_markers",
    "_footprint_breaches_wall",
    "_footprint_inside",
    "_inside_zone",
    "_is_beyond_lot_centre",
    "_normalise_angle",
    "_pure_pursuit_steer",
    "_staging_pos",
    "park_controller_from_metadata",
]
