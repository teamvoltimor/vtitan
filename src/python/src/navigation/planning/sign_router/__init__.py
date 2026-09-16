"""WRO 2026 traffic-sign routing for the obstacles challenge.

Computes lateral waypoint deformations so the robot passes a red obstacle on
its own RIGHT and a green one on its own LEFT, for the direction actually
driven. "Right" is a vehicle-relative rule that names opposite world axes
depending on which way the round is driven, so the per-(corridor, direction)
polarity lives in ``ROUTING_TABLE`` and is looked up with the committed
direction, never assumed direction-agnostic -- an absolute outward/inward
rule was the bug corrected earlier:
``adr:0059-pass-side-travel-relative-and-scorer-independence``.

Pure Python -- no ROS2 dependencies. Designed to be unit-tested independently.

Pass-side rule (in the vehicle's own frame):
    - Red obstacle   -> robot passes on its RIGHT.
    - Green obstacle -> robot passes on its LEFT.

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
    pass_lateral,
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
    "pass_lateral",
    "pass_side_lateral_axis",
    "pin_depth",
    "satisfiable_corridor",
    "signs_from_metadata",
]
