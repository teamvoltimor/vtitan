"""Sign-router tuning-derived constants and configuration.

Holds ``SignRouterConfig`` (the tuning parameters, validated at construction),
the tuning-derived ``SignRouterConstants`` and the ``SignRouterContext`` that
carries them to helper functions. Pure-Python, no ROS2, unit-testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import TrafficSignSpecs

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.geometry import chassis_half_diagonal_m

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning, SignRouterParams

# How far a deformed waypoint must stay clear of the restricted inner square and
# the outer wall (WP-1). An unclamped deformation can otherwise place the
# waypoint inside the inner square or against a wall for a sign positioned near a
# corridor edge. See navigation.geometry.chassis_half_diagonal_m for why it's the
# diagonal, not the half-width.
CHASSIS_HALF_DIAGONAL = chassis_half_diagonal_m()


@dataclass(frozen=True, slots=True)
class SignRouterConstants:
    """Tuning-derived sign-router constants, computed once per SignRouter instance."""

    wall_clearance_margin_m: float
    deform_depth_buffer_m: float
    pin_corner_guard: bool
    pin_heading_guard: bool
    pin_heading_guard_rad: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> SignRouterConstants:
        """Derive the sign-router constants from ``NavigationTuning``."""
        sr = tuning.sign_router
        return cls(
            wall_clearance_margin_m=sr.WALL_CLEARANCE_MARGIN_M,
            deform_depth_buffer_m=sr.DEFORM_DEPTH_BUFFER_M,
            pin_corner_guard=sr.PIN_CORNER_GUARD,
            pin_heading_guard=sr.PIN_HEADING_GUARD,
            pin_heading_guard_rad=math.radians(sr.PIN_HEADING_GUARD_DEG),
        )


class SignRouterContext(TuningContext[SignRouterConstants]):
    """Context holding tuning-derived sign-router constants, passed to helper functions."""

    _constants_cls = SignRouterConstants


_DEFAULT_SIGN_ROUTER_CONTEXT = SignRouterContext()


@dataclass(frozen=True)
class SignRouterConfig:
    """Tuning parameters for the sign router."""

    lateral_offset: float | None = None
    """Metres of lateral deformation perpendicular to the corridor."""

    activation_dist: float = 1.40
    """Deformation activates when robot is within this distance of a sign (m)."""

    passed_dist: float = 1.60
    """Sign is marked as passed once robot moves further than this from it (m)."""

    depth_pin: bool = True
    """Hold the commanded point abeam the sign instead of letting it recede.

    ``False`` restores the plain lookahead depth, which is the arm every figure
    recorded before the pin was measured against. Both arms belong in ONE
    harness invocation: this lives in the router, so measuring it by editing
    between two runs mixes old and new code across a warm process pool -- which
    is exactly how the pin's own effect was first mistaken for a classifier fix.
    """

    detection_match_dist: float = 0.30
    """Max world-frame distance to associate a camera detection with an expected sign (m)."""

    min_confidence: float = 0.25
    """Minimum detection confidence to accept a camera-based color update."""

    commit_hysteresis: bool = False
    """Hold the engaged sign across ticks instead of re-racing every tick.

    Matches signs/sign_router.toml's default -- was True here, contradicting the
    TOML's False, so a bare ``SignRouterConfig()`` (only reachable now if a
    caller constructs one directly rather than via ``from_tuning()``) silently
    re-enabled hysteresis the config file disables.
    """

    corridor_flip_ticks: int = 1
    """Consecutive ticks a refined sign estimate must agree on a NEW corridor
    before its label moves there. A sign's corridor picks which world axis its
    deformation treats as lateral, so on a corner boundary -- where two-thirds of
    legal WRO grid positions sit -- millimetres of estimate jitter otherwise
    swing the commanded waypoint between two orthogonal axes every tick.

    Matches signs/sign_router.toml's default of 1, which leaves the mechanism
    inert: the oscillation is real and traced, but damping it measured flat over
    the corpus and cost a few new wall strikes. See that file for the numbers."""

    settle_ticks: int = 150
    """Ticks since this lap started (~7.5s at the standard 20Hz control loop)
    before a sign may be engaged/passed at all. Right after spawn (or a lap
    boundary), the robot can briefly swing toward a corridor it hasn't actually
    reached yet while settling onto its planned route -- if that swing grazes a
    not-yet-really-encountered sign's activation radius, it gets engaged and then
    marked passed as the robot continues on its real route away from it, retiring
    the sign before its genuine pass ever happens. Deferring bookkeeping (not
    candidate selection -- a sign already in the robot's actual corridor still
    deforms normally) for this settle window prevents that incidental graze from
    ever registering."""

    def __post_init__(self) -> None:
        """Compute derived tuning values and validate the configuration.

        ``_active_sign_candidates`` engages a sign once it is nearer than
        ``activation_dist`` and retires it once it is further than
        ``passed_dist``, in that order, on the same tick. So with
        ``activation_dist >= passed_dist`` every sign entering the activation
        radius is engaged and marked passed in the same breath, from a metre
        away, and stays retired for the rest of the run -- deformation never
        fires at the real pass and the robot drives straight into it.

        Measured: at ``activation_dist`` 1.30 against the shipped ``passed_dist``
        1.20, the corpus goes from 209 collisions to 256/256 with not one lap
        completed. Nothing failed, nothing logged; avoidance simply stopped
        existing. That is the worst shape a config error can take in a safety
        path, so it is an error rather than a clamp: silently repairing it would
        hide that the tuning being run is not the tuning that was asked for.

        This is the same defect ``settle_ticks`` guards from the other direction
        -- there an incidental spawn-time graze retires a sign early; here the
        thresholds themselves do it, on every sign.
        """
        if self.lateral_offset is None:
            tuning = get_tuning(None)
            default_offset = (
                CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2 + tuning.sign_router.SIGN_CLEARANCE_MARGIN_M
            )
            object.__setattr__(self, "lateral_offset", default_offset)
        if self.activation_dist >= self.passed_dist:
            msg = (
                f"activation_dist ({self.activation_dist}) must be < passed_dist "
                f"({self.passed_dist}): a sign would be engaged and marked passed on the "
                f"same tick, permanently retiring it before its real pass and disabling "
                f"sign avoidance for the whole run."
            )
            raise ValueError(msg)

    @classmethod
    def from_tuning(cls, params: SignRouterParams) -> SignRouterConfig:
        """Build from NavigationTuning's SignRouterParams group.

        ``lateral_offset`` isn't a raw tunable in ``SignRouterParams`` -- only its
        safety-margin component is (``SIGN_CLEARANCE_MARGIN_M``), so this
        recomputes the same derivation ``_SIGN_LATERAL_OFFSET`` uses at module
        scope: chassis half-DIAGONAL + sign half-width + margin. Keep the two in
        step; see that constant for why it is the diagonal.
        """
        return cls(
            lateral_offset=CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2 + params.SIGN_CLEARANCE_MARGIN_M,
            activation_dist=params.ACTIVATION_DIST_M,
            passed_dist=params.PASSED_DIST_M,
            depth_pin=params.DEPTH_PIN,
            detection_match_dist=params.DETECTION_MATCH_DIST_M,
            min_confidence=params.MIN_CONFIDENCE,
            settle_ticks=params.SETTLE_TICKS,
            commit_hysteresis=params.COMMIT_HYSTERESIS,
            corridor_flip_ticks=params.CORRIDOR_FLIP_TICKS,
        )
