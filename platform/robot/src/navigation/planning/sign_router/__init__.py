"""WRO 2026 traffic-sign routing for the obstacles challenge.

Computes lateral waypoint deformations so the robot avoids a red obstacle on
its OUTWARD side (toward the outer wall) and a green obstacle on its INWARD
side (toward the inner square) -- an absolute rule tied to the track geometry,
not the travel direction: it holds identically whether the round is run
clockwise or counterclockwise.

Pure Python -- no ROS2 dependencies. Designed to be unit-tested independently.

Pass-side rule:
    - Red obstacle   -> robot passes on the OUTWARD side (away from centre).
    - Green obstacle -> robot passes on the INWARD side (toward centre).

The implementation is split into submodules (config/constants, routing table
+ pure helpers, deformation math, the stateful router); this package
re-exports the names callers import from ``...planning.sign_router`` -- the
``_``-prefixed helpers included, since tests and the router reach for them
directly (matching the ``waypoints`` package convention).
"""

from shared.domain.enums import Axis

from src.navigation.planning.sign_discovery import SignSpec
from src.navigation.planning.sign_router.config import (
    _CHASSIS_HALF_DIAGONAL,
    SignRouterConfig,
    SignRouterContext,
    _SignRouterConstants,
)
from src.navigation.planning.sign_router.deformation import (
    _apply_deformation,
    _match_detection_to_sign,
    _pin_depth,
)
from src.navigation.planning.sign_router.router import (
    _BEHIND_TOLERANCE,
    SignRouter,
)
from src.navigation.planning.sign_router.routing import (
    _ROUTING_TABLE,
    _is_squarely_in_corridor,
    clamp_lateral,
    outward_lateral_axis,
    signs_from_metadata,
)

__all__ = [
    "_BEHIND_TOLERANCE",
    "_CHASSIS_HALF_DIAGONAL",
    "_ROUTING_TABLE",
    "Axis",
    "SignRouter",
    "SignRouterConfig",
    "SignRouterContext",
    "SignSpec",
    "_SignRouterConstants",
    "_apply_deformation",
    "_is_squarely_in_corridor",
    "_match_detection_to_sign",
    "_pin_depth",
    "clamp_lateral",
    "outward_lateral_axis",
    "signs_from_metadata",
]
