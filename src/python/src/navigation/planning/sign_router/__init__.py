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
+ pure helpers, deformation math, the stateful router). This package
re-exports the public names callers import from ``...planning.sign_router``.
"""

from shared.domain.enums import Axis

from src.navigation.planning.sign_discovery import SignSpec
from src.navigation.planning.sign_router.config import (
    CHASSIS_HALF_DIAGONAL,
    SignRouterConfig,
    SignRouterConstants,
    SignRouterContext,
)
from src.navigation.planning.sign_router.deformation import (
    apply_deformation,
    match_detection_to_sign,
    pin_depth,
)
from src.navigation.planning.sign_router.router import (
    BEHIND_TOLERANCE,
    SignRouter,
)
from src.navigation.planning.sign_router.routing import (
    ROUTING_TABLE,
    candidate_corridors,
    clamp_lateral,
    depth_consistent_corridor,
    is_squarely_in_corridor,
    pass_side_lateral_axis,
    satisfiable_corridor,
    signs_from_metadata,
)

__all__ = [
    "BEHIND_TOLERANCE",
    "CHASSIS_HALF_DIAGONAL",
    "ROUTING_TABLE",
    "Axis",
    "SignRouter",
    "SignRouterConfig",
    "SignRouterConstants",
    "SignRouterContext",
    "SignSpec",
    "apply_deformation",
    "candidate_corridors",
    "clamp_lateral",
    "depth_consistent_corridor",
    "is_squarely_in_corridor",
    "match_detection_to_sign",
    "pass_side_lateral_axis",
    "pin_depth",
    "satisfiable_corridor",
    "signs_from_metadata",
]
