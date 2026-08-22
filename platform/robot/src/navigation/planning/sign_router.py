"""WRO 2026 traffic-sign routing for obstacles challenge.

Computes lateral waypoint deformations so the robot avoids a red obstacle on
its OUTWARD side (toward the outer wall) and a green obstacle on its INWARD
side (toward the inner square) — an absolute rule tied to the track geometry,
not the travel direction: it holds identically whether the round is run
clockwise or counterclockwise.

Pure Python — no ROS2 dependencies. Designed to be unit-tested independently.

Pass-side rule:
    - Red obstacle   → robot passes on the OUTWARD side (away from centre).
    - Green obstacle → robot passes on the INWARD side (toward centre).

Pinned by ``TestPassSideRule`` in ``tests/unit/test_sign_router.py``: for
every (section, direction) the deformed waypoint moves outward for red and
inward for green.

The deformation is still keyed by (Section, Direction) because the AXIS and
WORLD-FRAME SIGN of "outward" both depend on which corridor is being driven,
but — unlike an earlier version of this table — the CLOCKWISE and
COUNTERCLOCKWISE rows for a given section are now IDENTICAL, not negations of
each other: outward/inward is a fixed property of the corridor, independent
of which way the robot is circling it.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING, Any

from shared.config.constants import DictKeys, TrackDimensions, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning, SignDiscoveryParams, SignRouterParams
from shared.domain.enums import Direction, Section
from shared.domain.models import ScenarioMetadata, SignColor, Waypoint

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.geometry import behind_tolerance_m, chassis_half_diagonal_m
from src.navigation.planning.sign_discovery import (
    ObservedSignMap,
    SignSpec,
)
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.utils import _dist2d, wrap_angle

if TYPE_CHECKING:
    from shared.domain.models import TrafficSignObservation

logger = logging.getLogger(__name__)

# Re-exported from sign_discovery, which owns the projection primitives this
# module routes on top of. Kept importable from here because that is where
# every caller and test already reaches for them.
__all__ = [
    "Axis",
    "SignRouter",
    "SignRouterConfig",
    "SignSpec",
    "clamp_lateral",
    "outward_lateral_axis",
    "signs_from_metadata",
]

# How far a deformed waypoint must stay clear of the restricted inner square
# and the outer wall (WP-1). An unclamped deformation can otherwise place the
# waypoint inside the inner square or against a wall for a sign positioned near
# a corridor edge. See navigation.geometry.chassis_half_diagonal_m for why it's
# the diagonal, not the half-width.
_CHASSIS_HALF_DIAGONAL = chassis_half_diagonal_m()

# How far behind the robot's own origin a sign may still sit and remain an
# avoidance candidate. See navigation.geometry.behind_tolerance_m.
_BEHIND_TOLERANCE = behind_tolerance_m()


class Axis(StrEnum):
    """Which world coordinate a routing-table entry deforms."""

    X = "x"
    Y = "y"


# Per-(corridor, direction) routing table: (axis, red_mult, green_mult).
# axis: Axis.Y means deform the y-coordinate; Axis.X deforms x.
# red_mult / green_mult: +1 or -1 multiplier applied to the LATERAL offset,
# chosen so red always moves the deformed waypoint OUTWARD (away from the
# inner square) and green always moves it INWARD — identically for CW and
# CCW, since outward/inward is a fixed property of the corridor, not the
# travel direction. (An earlier version of this table made the CW rows the
# world-frame negation of the CCW rows, which instead pinned "red on the
# robot's right" — a travel-RELATIVE rule that flips outward/inward between
# CW and CCW. That was wrong: the official rule is the absolute one above.)
_ROUTING_TABLE: dict[tuple[Section, Direction], tuple[Axis, int, int]] = {
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): (Axis.Y, -1, +1),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): (Axis.Y, +1, -1),
    (Section.EAST, Direction.COUNTERCLOCKWISE): (Axis.X, +1, -1),
    (Section.WEST, Direction.COUNTERCLOCKWISE): (Axis.X, -1, +1),
    (Section.SOUTH, Direction.CLOCKWISE): (Axis.Y, -1, +1),
    (Section.NORTH, Direction.CLOCKWISE): (Axis.Y, +1, -1),
    (Section.EAST, Direction.CLOCKWISE): (Axis.X, +1, -1),
    (Section.WEST, Direction.CLOCKWISE): (Axis.X, -1, +1),
}


def outward_lateral_axis(corridor: Section, color: SignColor) -> tuple[Axis, int] | None:
    """World-frame axis and sign of the pass-side rule for ``corridor``, direction-agnostic.

    The CLOCKWISE and COUNTERCLOCKWISE rows of ``_ROUTING_TABLE`` are
    identical for every section (see module docstring), so looking the rule
    up under a fixed direction is exactly as correct as knowing the real one.
    This lets a caller that hasn't inferred the travel direction yet -- e.g.
    ``corridor_follower`` during BLIND_CREEP -- still apply "red outward,
    green inward" instead of falling back to generic obstacle avoidance.

    Returns:
        ``(axis, multiplier)`` where a positive multiplier along ``axis``
        points OUTWARD (away from the inner square) for red, INWARD for
        green. ``None`` if ``corridor`` has no routing entry.
    """
    entry = _ROUTING_TABLE.get((corridor, Direction.CLOCKWISE))
    if entry is None:
        return None
    axis, red_mult, green_mult = entry
    return axis, red_mult if color == SignColor.RED else green_mult


@dataclass(frozen=True, slots=True)
class _SignRouterConstants:
    """Tuning-derived sign-router constants, computed once per SignRouter instance."""

    wall_clearance_margin_m: float
    deform_depth_buffer_m: float
    pin_corner_guard: bool
    pin_heading_guard: bool
    pin_heading_guard_rad: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> _SignRouterConstants:
        sr = tuning.sign_router
        return cls(
            wall_clearance_margin_m=sr.WALL_CLEARANCE_MARGIN_M,
            deform_depth_buffer_m=sr.DEFORM_DEPTH_BUFFER_M,
            pin_corner_guard=sr.PIN_CORNER_GUARD,
            pin_heading_guard=sr.PIN_HEADING_GUARD,
            pin_heading_guard_rad=math.radians(sr.PIN_HEADING_GUARD_DEG),
        )


class SignRouterContext(TuningContext[_SignRouterConstants]):
    """Context holding tuning-derived sign-router constants, passed to helper functions."""

    _constants_cls = _SignRouterConstants


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

    Matches signs/sign_router.toml's default -- was True here, contradicting
    the TOML's False, so a bare ``SignRouterConfig()`` (only reachable now if
    a caller constructs one directly rather than via ``from_tuning()``)
    silently re-enabled hysteresis the config file disables.
    """

    corridor_flip_ticks: int = 1
    """Consecutive ticks a refined sign estimate must agree on a NEW corridor
    before its label moves there. A sign's corridor picks which world axis its
    deformation treats as lateral, so on a corner boundary — where two-thirds
    of legal WRO grid positions sit — millimetres of estimate jitter otherwise
    swing the commanded waypoint between two orthogonal axes every tick.

    Matches signs/sign_router.toml's default of 1, which leaves the mechanism
    inert: the oscillation is real and traced, but damping it measured flat over
    the corpus and cost a few new wall strikes. See that file for the numbers."""

    settle_ticks: int = 150
    """Ticks since this lap started (~7.5s at the standard 20Hz control loop)
    before a sign may be engaged/passed at all. Right after spawn (or a lap
    boundary), the robot can briefly swing toward a corridor it hasn't
    actually reached yet while settling onto its planned route — if that
    swing grazes a not-yet-really-encountered sign's activation radius, it
    gets engaged and then marked passed as the robot continues on its real
    route away from it, retiring the sign before its genuine pass ever
    happens. Deferring bookkeeping (not candidate selection — a sign already
    in the robot's actual corridor still deforms normally) for this settle
    window prevents that incidental graze from ever registering."""

    def __post_init__(self) -> None:
        """Compute derived tuning values and validate the configuration.

        ``_active_sign_candidates`` engages a sign once it is nearer than
        ``activation_dist`` and retires it once it is further than
        ``passed_dist``, in that order, on the same tick. So with
        ``activation_dist >= passed_dist`` every sign entering the activation
        radius is engaged and marked passed in the same breath, from a metre
        away, and stays retired for the rest of the run — deformation never
        fires at the real pass and the robot drives straight into it.

        Measured: at ``activation_dist`` 1.30 against the shipped
        ``passed_dist`` 1.20, the corpus goes from 209 collisions to 256/256
        with not one lap completed. Nothing failed, nothing logged; avoidance
        simply stopped existing. That is the worst shape a config error can
        take in a safety path, so it is an error rather than a clamp: silently
        repairing it would hide that the tuning being run is not the tuning
        that was asked for.

        This is the same defect ``settle_ticks`` guards from the other
        direction — there an incidental spawn-time graze retires a sign early;
        here the thresholds themselves do it, on every sign.
        """
        if self.lateral_offset is None:
            tuning = get_tuning(None)
            default_offset = _CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2 + tuning.sign_router.SIGN_CLEARANCE_MARGIN_M
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

        ``lateral_offset`` isn't a raw tunable in ``SignRouterParams`` -- only
        its safety-margin component is (``SIGN_CLEARANCE_MARGIN_M``), so this
        recomputes the same derivation ``_SIGN_LATERAL_OFFSET`` uses at module
        scope: chassis half-DIAGONAL + sign half-width + margin. Keep the two
        in step; see that constant for why it is the diagonal.
        """
        return cls(
            lateral_offset=_CHASSIS_HALF_DIAGONAL + TrafficSignSpecs.WIDTH / 2 + params.SIGN_CLEARANCE_MARGIN_M,
            activation_dist=params.ACTIVATION_DIST_M,
            passed_dist=params.PASSED_DIST_M,
            depth_pin=params.DEPTH_PIN,
            detection_match_dist=params.DETECTION_MATCH_DIST_M,
            min_confidence=params.MIN_CONFIDENCE,
            settle_ticks=params.SETTLE_TICKS,
            commit_hysteresis=params.COMMIT_HYSTERESIS,
            corridor_flip_ticks=params.CORRIDOR_FLIP_TICKS,
        )


class SignRouter:
    """Routes the robot past traffic signs using lateral waypoint deformations.

    Usage in navigation tick::

        deformed_wp = router.deform_waypoint(
            waypoint=target_wp,
            robot_pos=(robot_x, robot_y),
            robot_yaw=robot_yaw,
            corridor=current_corridor,
            detections=latest_detections,
        )

    Args:
        signs: Expected sign list from scenario metadata (position + color).
        config: Tuning parameters.
        discovery_config: Tuning parameters for blind sign discovery
            (ObservedSignMap), only used when ``discover=True``.
    """

    def __init__(
        self,
        signs: list[SignSpec],
        config: SignRouterConfig | None = None,
        direction: Direction = Direction.COUNTERCLOCKWISE,
        discover: bool = False,
        discovery_config: SignDiscoveryParams | None = None,
        tuning: NavigationTuning | None = None,
    ) -> None:
        self._signs = list(signs)
        # SignRouterConfig.from_tuning(), not a bare SignRouterConfig(): the
        # dataclass's own field defaults are a second, independent copy of
        # the TOML defaults and can drift from them (commit_hysteresis=True
        # here vs. False in signs/sign_router.toml, until this fix) --
        # `tuning` is already accepted by this constructor, so there's no
        # reason the fallback shouldn't use it too.
        self._config = config or SignRouterConfig.from_tuning(get_tuning(tuning).sign_router)
        self._context = SignRouterContext(tuning)
        self._direction = direction
        self._passed: set[int] = set()
        self._engaged: set[int] = set()
        self._lap_tick = 0
        # Sign indices passed on the WRONG side of the corridor. The official
        # Obstacles rule is absolute: a RED obstacle must be cleared on its
        # OUTWARD side, a GREEN on its INWARD side. ``_active_sign_candidates``
        # records here, at the instant a sign is retired as passed, whether the
        # robot was on the permitted side — see ``_record_pass_side``. The
        # simulator reads ``wrong_side_violations`` and stops the run, the same
        # way it stops on a forbidden wall contact.
        self._wrong_side: set[int] = set()
        # The sign currently being routed around, kept across ticks so the
        # commanded line does not jump between two legal ones mid-pass. See
        # _prefer_committed.
        self._committed: int | None = None
        # Robot yaw at the tick each sign was first committed to, keyed by sign
        # index. Lets the depth pin (see _pin_depth) release on heading drift
        # even when the position-only PIN_CORNER_GUARD still reads squarely in
        # the corridor -- see PIN_HEADING_GUARD.
        self._commit_yaw: dict[int, float] = {}
        # Each sign's own corridor, kept in step with _signs — deform_waypoint()
        # must never apply a sign's (x, y) through a different corridor's axis
        # convention (see _nearest_active_sign). Recomputed per sign rather than
        # once up front, since discovery can both append signs and move an
        # existing one across a corridor boundary as its estimate improves.
        self._sign_corridors = [corridor_for_position(s.x, s.y) for s in self._signs]
        # Per-sign "how many ticks running has the estimate wanted to move to a
        # different corridor", keyed by sign index. See _settled_corridor.
        self._corridor_flip_streak: dict[int, tuple[Section, int]] = {}
        # Discovery mode: the sign layout is randomised every round and no
        # scenario file exists on the mat, so a blind robot has to find the
        # signs with its camera rather than be handed them. See sign_discovery.
        self._sign_map: ObservedSignMap | None
        if discover:
            discovery_config = discovery_config or SignDiscoveryParams()
            self._sign_map = ObservedSignMap(
                self._config.min_confidence,
                max_ingest_range_m=discovery_config.MAX_INGEST_RANGE_M,
                association_dist_m=discovery_config.ASSOCIATION_DIST_M,
                min_hits=discovery_config.MIN_HITS,
                robot_corridor_flip_ticks=discovery_config.ROBOT_CORRIDOR_FLIP_TICKS,
            )
        else:
            self._sign_map = None

    @property
    def signs(self) -> list[SignSpec]:
        """Signs currently being routed around — discovered ones included."""
        return list(self._signs)

    def _ingest_observations(
        self,
        observations: list[TrafficSignObservation] | None,
        robot_pos: Waypoint,
    ) -> None:
        """Fold a frame of observations into the discovered sign list.

        Appending only ever grows ``_signs``, which is what keeps the
        index-keyed ``_passed``/``_engaged`` bookkeeping valid. A published
        sign's position is refined in place for the same reason.
        """
        if self._sign_map is None:
            return

        self._sign_map.observe(observations, robot_pos)

        for track in self._sign_map.newly_confirmed():
            track.published_index = len(self._signs)
            spec = track.as_spec()
            self._signs.append(spec)
            self._sign_corridors.append(corridor_for_position(spec.x, spec.y))
            logger.info(
                "Discovered %s sign %d at (%.2f, %.2f) from %d detections",
                spec.color,
                track.published_index,
                spec.x,
                spec.y,
                track.hits,
            )

        for track in self._sign_map.published():
            index = track.published_index
            if index is None:
                continue
            spec = track.as_spec()
            if spec != self._signs[index]:
                self._signs[index] = spec
                self._sign_corridors[index] = self._settled_corridor(index, spec)

    def _settled_corridor(self, index: int, spec: SignSpec) -> Section:
        """Corridor for a refined sign estimate, held steady against jitter.

        A sign's corridor is what selects the world axis its deformation treats
        as lateral, and ``corridor_for_position`` is a hard partition with no
        dead zone. On a corner boundary — where two-thirds of legal WRO grid
        positions sit — a discovery estimate wobbling by millimetres therefore
        alternates between two corridors whose lateral axes are ORTHOGONAL, and
        the commanded waypoint jumps between two unrelated targets on every
        tick. The chassis converges on neither.

        A genuine corridor change (the estimate really was in the wrong place
        early on) still lands, just ``corridor_flip_ticks`` later — 0.25 s at
        20 Hz, against a 1.40 m activation distance.

        Temporal rather than a geometric dead-band deliberately: the corner
        tie-break picks the nearest inner face, so moving a point further into
        the corridor you want to keep can make that corridor *less* likely, and
        there is no usable distance-to-decision-surface to threshold on.
        """
        fresh = corridor_for_position(spec.x, spec.y)
        current = self._sign_corridors[index]
        if fresh == current:
            self._corridor_flip_streak.pop(index, None)
            return current

        candidate, streak = self._corridor_flip_streak.get(index, (fresh, 0))
        streak = streak + 1 if candidate == fresh else 1
        if streak < self._config.corridor_flip_ticks:
            self._corridor_flip_streak[index] = (fresh, streak)
            return current

        self._corridor_flip_streak.pop(index, None)
        logger.info(
            "Sign %d moved %s -> %s after %d consistent ticks at (%.2f, %.2f)",
            index,
            current.name,
            fresh.name,
            streak,
            spec.x,
            spec.y,
        )
        return fresh

    @property
    def is_discovering(self) -> bool:
        """True when the sign layout is being found by camera rather than handed over.

        The distinguishing fact about a discovering run is that the robot
        cannot know a corridor's signs until it has entered that corridor --
        signs sit inside corridors and the next one is outside a 102 deg FOV
        until the corner is turned, so observations top out around 2.3 m
        regardless of discovery tuning. Callers use this to treat the first
        lap as reconnaissance; see ``EXPLORE_LAP_SPEED_FRAC``.
        """
        return self._sign_map is not None

    @property
    def lane_specs(self) -> list[tuple[SignSpec, Section]]:
        """Every routed sign paired with the corridor label the router uses for it.

        Pairs rather than positions alone because a sign's corridor is what
        selects the world axis its avoidance treats as lateral, and the
        router's own label is the settled one (see ``_settled_corridor``) --
        recomputing it in the consumer would reintroduce the corner jitter that
        damping exists to suppress.

        Passed signs are INCLUDED, unlike ``routed_sign_positions``: the lane
        is planned geometry, so retiring a sign mid-lap would rewrite the path
        under a chassis that is still on it. The signs come back every lap
        anyway (``reset_for_new_lap``), so the lane is a property of the layout,
        not of this lap's bookkeeping.
        """
        return list(zip(self._signs, self._sign_corridors, strict=True))

    @property
    def lane_fingerprint(self) -> tuple[tuple[float, float, str], ...]:
        """Identity of the current sign layout, for cheap change detection.

        Blind discovery both appends signs and refines existing positions every
        tick, and rebuilding the planned path on a tick where nothing moved
        would re-seek the waypoint index for no reason. Rounded to the
        centimetre so sub-millimetre estimate jitter -- which cannot move a
        waypoint visibly -- does not count as a change.
        """
        return tuple((round(s.x, 2), round(s.y, 2), str(s.color)) for s in self._signs)

    @property
    def active_sign_count(self) -> int:
        """Number of signs not yet marked as passed (this lap)."""
        return len(self._signs) - len(self._passed)

    @property
    def routed_sign_positions(self) -> list[tuple[float, float]]:
        """World positions of the signs this router still intends to route around.

        The reactive collision layer uses this to tell a mapped obstacle it has
        a plan for from an unmapped one it does not (see
        ``collision_avoidance_controller.mask_mapped_obstacles``). Signs already
        marked passed are excluded: the router has stopped steering around them,
        so nothing owns them any more and they get the full reactive guard back.

        Discovered signs are included on the same footing as metadata ones —
        both live in ``_signs`` — but only once ``ObservedSignMap`` has actually
        published them, so an unconfirmed track never suppresses the guard.
        """
        return [(s.x, s.y) for i, s in enumerate(self._signs) if i not in self._passed]

    @property
    def routed_sign_positions_by_corridor(self) -> list[tuple[float, float, Section]]:
        """``routed_sign_positions``, each paired with the sign's own corridor.

        For ``mask_mapped_obstacles``'s escape-mask attribution: proximity
        alone is not enough to trust a LIDAR ray's endpoint as "this routed
        sign" under a wrong-but-consistent rigid rotation of the believed
        pose, which can reproject a genuinely unmapped obstacle's ray onto a
        routed sign's coordinates by coincidence (the same rotational-lock
        failure ``_SignTrack.corridor`` exists to guard discovery against).
        Requiring the ray's own corridor to match this sign's closes that
        gap without needing to know the rotation is even present.
        """
        return [(s.x, s.y, c) for i, (s, c) in enumerate(self.lane_specs) if i not in self._passed]

    def reset_for_new_lap(self) -> None:
        """Re-arm every sign so it's routed again on the next lap.

        Without this, a sign marked ``_passed`` on lap 1 (once the robot moves
        beyond ``passed_dist``) stays passed for the rest of the run — the
        Obstacles Challenge requires clearing every sign on all 3 laps, not
        just the first time each one is encountered.
        """
        self._passed.clear()
        self._engaged.clear()
        self._lap_tick = 0
        self._committed = None
        self._commit_yaw.clear()

    @property
    def wrong_side_violations(self) -> set[int]:
        """Sign indices retired as passed on the WRONG side of the corridor.

        Emptied by ``reset_for_new_lap`` so each lap is judged independently
        (a sign avoided correctly on lap 2 after a lap-1 violation is a fresh
        pass, not a reversal of the earlier miss). The simulator stops the run
        the moment this is non-empty.
        """
        return set(self._wrong_side)

    def _record_pass_side(self, index: int, robot_pos: Waypoint) -> None:
        """Decide whether ``index`` was cleared on its permitted side.

        The permitted side is absolute, fixed by the corridor geometry and the
        sign colour — red outward, green inward — and is exactly the lateral
        direction ``_ROUTING_TABLE`` deforms toward for that colour. The robot's
        lateral coordinate relative to the sign's is compared against it: same
        sign ⇒ correct side, opposite sign ⇒ wrong-side pass, recorded in
        ``_wrong_side``.

        The comparison uses the robot's position at the instant the sign is
        retired (distance > ``passed_dist``). By then the chassis is ~1.6 m
        down the corridor axis from the sign, but it is travelling *along* that
        axis, so its lateral coordinate is the same one it held abeam the sign
        — which is precisely the choice of side that the pass represents.
        """
        sign = self._signs[index]
        entry = _ROUTING_TABLE.get((self._sign_corridors[index], Direction.CLOCKWISE))
        if entry is None:
            return
        axis, red_mult, green_mult = entry
        permitted = red_mult if sign.color == SignColor.RED else green_mult
        robot_lat = robot_pos.x if axis == Axis.X else robot_pos.y
        sign_lat = sign.x if axis == Axis.X else sign.y
        side = 0 if robot_lat == sign_lat else (1 if robot_lat > sign_lat else -1)
        if side != 0 and side != permitted:
            self._wrong_side.add(index)

    def deform_waypoint(
        self,
        waypoint: tuple[float, float],
        robot_pos: tuple[float, float],
        robot_yaw: float,
        corridor: Section,
        observations: list[TrafficSignObservation] | None = None,
    ) -> tuple[float, float]:
        """Return a (possibly laterally deformed) version of the target waypoint.

        Checks all uncleared signs. The NEAREST active sign within activation
        distance drives the deformation. Camera observations are used to confirm
        the sign color if available and within match distance.

        Args:
            waypoint: Current target waypoint (x, y).
            robot_pos: Current robot position (x, y).
            robot_yaw: Robot heading (radians, 0 = east).
            corridor: Current track section.
            observations: Latest world-coordinate sign observations.

        Returns:
            Deformed waypoint (x, y). Unchanged if no active sign nearby.
        """
        # Discovery first: in blind mode this frame may be what reveals the
        # sign about to be routed around, so it has to land before candidate
        # selection rather than after it.
        robot_wp = Waypoint(*robot_pos)
        self._ingest_observations(observations, robot_wp)

        candidates = self._prefer_committed(self._active_sign_candidates(robot_wp, robot_yaw, corridor))

        # Walk candidates nearest-first and use the first whose deformation is
        # actually applicable, rather than giving up entirely if the closest one
        # is not. The nearest sign is often one the robot is still alongside but
        # has effectively cleared, sitting in the corridor just left behind; its
        # own deformation no longer applies to a target point that has already
        # moved into the next corridor. Returning the waypoint untouched in that
        # case blanks out avoidance for exactly the stretch approaching the NEXT
        # sign — which, if that sign sits near the corner exit, is the entire
        # runway available to steer around it.
        for nearest_idx, nearest_dist in candidates:
            # Sorted nearest-first, so once one is out of range every later one
            # is too — nothing further can apply.
            if nearest_dist > self._config.activation_dist:
                return waypoint

            # Key the corner check and the deformation math off the CANDIDATE
            # SIGN's own corridor, not the robot's current corridor label. The
            # two can legitimately disagree right at a corner — see
            # _active_sign_candidates' same-corridor-OR-within-activation_dist
            # comment — and the sign's own corridor is what actually determines
            # which world axis is "lateral" for it; using the robot's (possibly
            # stale, pre-corner) label here would deform the wrong axis.
            sign_corridor = self._sign_corridors[nearest_idx]

            # The deformation model assumes a straight corridor segment (hold
            # the depth axis, override the lateral axis with a value derived
            # from the sign's fixed position). Once the *target* waypoint itself
            # has curved into a corner, that override is stale and increasingly
            # wrong — skip it rather than fight the path's own curve.
            # Deliberately stricter than corridor_for_position()'s corner
            # tie-break (which exists to always assign the ROBOT some corridor,
            # even ambiguously): a corner waypoint like (2.42, 2.42) ties NORTH
            # vs EAST there and gets assigned NORTH by insertion order, but it's
            # still on the turning arc, not the straight segment this
            # deformation model assumes.
            if _is_squarely_in_corridor(waypoint[0], waypoint[1], sign_corridor, self._context):
                break
        else:
            # No candidate produced an applicable deformation.
            self._committed = None
            return waypoint

        # Stay with this sign until it is genuinely cleared, rather than
        # re-running the nearest-wins race from scratch next tick.
        if self._committed != nearest_idx:
            self._commit_yaw[nearest_idx] = robot_yaw
        self._committed = nearest_idx
        yaw_drift = abs(wrap_angle(robot_yaw - self._commit_yaw[nearest_idx]))
        sign = self._signs[nearest_idx]
        color = sign.color

        # Optionally override color with camera observation.
        if observations:
            camera_color = _match_detection_to_sign(
                observations,
                (sign.x, sign.y),
                self._config,
            )
            if camera_color is not None:
                color = camera_color.value

        # Taper the offset so it fades in and out over `passed_dist` instead of
        # snapping between full magnitude and zero in a single waypoint step at
        # a corridor boundary — a kink arriving at exactly the same place the
        # car is also turning through.
        #
        # Taper on whichever of the ROBOT or the TARGET POINT is nearer the
        # sign, not the target point alone. The lookahead target runs 0.2-0.4m
        # ahead of the robot, so keying on it alone means that at the instant
        # the robot draws level with the sign — the one moment full offset is
        # actually needed — the target is already that far PAST the sign and
        # the taper has quietly cut the offset by a third or more. The robot
        # then chases a half-hearted target and grazes the sign it was supposed
        # to clear. Taking the minimum holds full strength across the whole real
        # pass (either the robot or its target is near the sign throughout) and
        # decays only once both are clear, which preserves the smoothing this
        # taper exists for. Waypoint-at-sign callers still see taper == 1.0, so
        # single-point behaviour is unchanged.
        influence_dist = min(
            _dist2d(Waypoint(*waypoint), Waypoint(sign.x, sign.y)),
            _dist2d(robot_wp, Waypoint(sign.x, sign.y)),
        )
        # This shape peaks the commanded offset AT the sign: at activation_dist
        # 1.40 against passed_dist 1.60 the taper opens at 0.125, so avoidance
        # asks for 3.5 cm where a mid-turn pass needs 20.4 cm. That looks like
        # the reason the offset arrives late, and it is not. Holding full offset
        # from activation and fading only on the way out was measured over the
        # 256-scenario corpus at ramps of 0.20/0.40/0.70 m: byte-identical
        # without the depth pin (182 collisions at every value), and slightly
        # WORSE with it (137 -> 135 in-time). The lateral clamp saturates before
        # the taper ever binds, so the ramp has nothing to give. Do not re-try
        # it without new information; see docs/sign-avoidance-investigation.md.
        taper = max(0.0, 1.0 - influence_dist / self._config.passed_dist)
        effective_offset = self._config.lateral_offset * taper

        deformed = _apply_deformation(
            waypoint,
            sign,
            color,
            sign_corridor,
            self._direction,
            effective_offset,
            robot_pos if self._config.depth_pin else None,
            self._context,
            yaw_drift,
        )

        if deformed != waypoint:
            logger.debug(
                "Sign %d (%s) deformation: wp (%.3f,%.3f) → (%.3f,%.3f) [dist=%.2f m]",
                nearest_idx,
                color,
                waypoint[0],
                waypoint[1],
                deformed[0],
                deformed[1],
                nearest_dist,
            )

        return deformed

    def _prefer_committed(self, candidates: list[tuple[int, float]]) -> list[tuple[int, float]]:
        """Keep routing around the sign already being routed around.

        ``_active_sign_candidates`` re-runs a pure nearest-wins race every tick
        with no memory of the previous one. Where two signs are both in play —
        common, since the WRO grid puts them 0.50 m apart along a corridor and
        the corridor is only 1.0 m wide — the winner can flip while the chassis
        is already committed, and the commanded lateral line jumps from one
        sign's required value to the other's in a single tick. Both lines are
        legal; the damage is switching between them with no runway left to
        track the new one. Measured over the 256-scenario corpus, 28 of 229
        collisions had the winner change during the fatal approach.

        So a sign that is still an applicable candidate holds its claim. This
        is deliberately hysteresis on SELECTION only — the deformation math and
        the pass-side rule are untouched, which is what the two clearance-bound
        attempts got wrong (see the investigation doc).

        The claim is dropped as soon as it stops being reachable: when the sign
        retires (``_passed``), falls behind the chassis, leaves
        ``activation_dist``, or yields no applicable deformation. Without the
        distance test a receding sign could hold the claim from beyond its own
        activation range and mask the one coming up — the same masking bug
        already fixed once in the nearest-wins ordering.
        """
        if not self._config.commit_hysteresis or self._committed is None:
            return candidates
        for entry in candidates:
            if entry[0] == self._committed:
                if entry[1] > self._config.activation_dist:
                    break
                return [entry, *(c for c in candidates if c[0] != self._committed)]
        self._committed = None
        return candidates

    def _active_sign_candidates(
        self,
        robot_pos: Waypoint,
        robot_yaw: float,
        corridor: Section,
    ) -> list[tuple[int, float]]:
        """Not-yet-passed signs near ``corridor``, as ``(index, distance)`` nearest-first.

        Also maintains engagement/passed bookkeeping: a sign is engaged once the
        robot comes within activation distance, and retired only after it has
        been engaged and then left beyond ``passed_dist`` — never discarded from
        afar (which would silently disable routing at spawn). Bookkeeping runs
        for every sign regardless of corridor.

        Candidates are additionally restricted to signs not already behind the
        robot (measured along its heading, with ``_BEHIND_TOLERANCE`` slack so a
        sign still alongside the chassis keeps holding the line out). A receding
        sign stays geometrically nearer than the next one for a while, so
        without this the nearest-wins rule below masks the upcoming sign until
        it is far too close to steer around.

        The full ranked list is returned, not just the nearest, so the caller can
        fall through to the next one when the closest sign's deformation is not
        applicable to the current target point.

        Candidates are restricted to signs that either belong to
        ``corridor`` or are within ``activation_dist`` of the robot — not
        strict same-corridor equality. A sign one corridor over can sit right
        at a corner (e.g. at that corridor's own "near" grid depth, exactly on
        CORNER_MIN/MAX); the robot's corridor label only flips once its
        cornering arc has already carried it past that point, which is too
        late for any deformation to matter. Requiring same-corridor OR
        within-activation_dist lets a genuinely close cross-corridor sign start
        bending the path before the label flips, while still keeping distant
        cross-corridor signs from being engaged prematurely. The caller
        (``deform_waypoint``) uses the CANDIDATE's own corridor — not this
        method's ``corridor`` argument — for the actual axis/clamp math, so a
        cross-corridor candidate is never run through the wrong convention.
        Engage/pass bookkeeping itself is suppressed for the first
        ``settle_ticks`` of a lap (see ``SignRouterConfig.settle_ticks``);
        candidate selection isn't, so a sign genuinely in the robot's current
        corridor still deforms normally even during that window.

        Returns:
            ``[(index, distance), ...]`` sorted nearest-first; empty when no
            active sign remains.
        """
        self._lap_tick += 1
        settled = self._lap_tick > self._config.settle_ticks
        candidates: list[tuple[int, float]] = []

        for i, sign in enumerate(self._signs):
            if i in self._passed:
                continue
            d = _dist2d(robot_pos, Waypoint(sign.x, sign.y))
            if settled and d < self._config.activation_dist:
                self._engaged.add(i)
            if d > self._config.passed_dist:
                if settled and i in self._engaged:
                    self._passed.add(i)
                    self._record_pass_side(i, robot_pos)
                    logger.debug("Sign %d marked as passed (dist=%.2f m)", i, d)
                continue
            # A sign the robot has already driven past needs no avoidance, and
            # letting one stay a candidate actively HARMS the next sign: the
            # nearest-first ordering would keep ranking the receding sign
            # (still geometrically closer for a while) above the one actually
            # coming up, masking it until it's too close to steer around.
            # Measure along the robot's own heading and keep signs still
            # alongside the chassis (they must go on holding the line out until
            # fully cleared).
            dx, dy = sign.x - robot_pos.x, sign.y - robot_pos.y
            along_track = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
            if along_track < -_BEHIND_TOLERANCE:
                continue
            same_corridor = self._sign_corridors[i] == corridor
            # A sign in a DIFFERENT corridor than the robot's current label only
            # qualifies once the robot is within activation_dist of it — i.e.
            # close enough that the sign's own geometry is what actually
            # matters, not the robot's corridor bookkeeping. Without this, a
            # sign sitting right at a corner (e.g. at the corridor's own "near"
            # depth, exactly on CORNER_MIN/MAX) never becomes a deformation
            # candidate until the robot's corridor label flips — which happens
            # only once the robot's cornering arc has already carried it
            # straight past the sign, too late for any deformation to matter.
            # Requiring same-corridor OR within-activation_dist keeps distant
            # cross-corridor signs from being engaged prematurely while still
            # letting a genuinely close one start bending the path early.
            if not same_corridor and d > self._config.activation_dist:
                continue
            candidates.append((i, d))

        candidates.sort(key=lambda entry: entry[1])
        return candidates


def _apply_deformation(
    waypoint: tuple[float, float],
    sign: SignSpec,
    color: str,
    corridor: Section,
    direction: Direction,
    lateral_offset: float,
    robot_pos: tuple[float, float] | None = None,
    context: SignRouterContext | None = None,
    yaw_drift: float | None = None,
) -> tuple[float, float]:
    """Compute the laterally deformed waypoint for a given sign and corridor.

    The result is clamped so it can't land inside the restricted inner square
    or beyond the outer wall (WP-1) — a sign positioned near a corridor edge
    would otherwise deform the waypoint straight into a hazard.

    Only the LATERAL coordinate carries the avoidance; the depth coordinate is
    whatever the lookahead search picked, which sits 0.2-0.4 m further along
    the corridor every tick. Passing that through unchanged is what makes the
    offset arrive late: the commanded point holds a constant lateral value but
    keeps receding, so the slope the chassis must follow to reach it flattens
    tick by tick and the lateral error is only ever asymptotically closed --
    traced on go_obstacles_0000, the chassis needed 0.324 m of lateral travel
    over the 0.42 m of runway left and achieved 0.163 m of it, arriving level
    with the pillar still half a chassis width inside the line it was given.
    Ramping to full offset sooner does NOT fix that lag -- measured and
    rejected, see the taper comment in ``deform_waypoint`` -- because the target
    the offset is attached to is the thing running away.

    So while the sign lies between the robot and the lookahead point, pin the
    depth coordinate to the SIGN's own depth. The commanded point stops
    receding and becomes a fixed gate abeam the pillar, which the chassis has
    to be on by the time it gets there. ``robot_pos`` is optional so callers
    testing the pure pass-side mapping can keep asking for it alone.

    Args:
        waypoint: Original target waypoint (x, y).
        sign: Traffic sign spec (position + color).
        color: Effective sign color (may be camera-confirmed).
        corridor: Current track section.
        direction: Travel direction (CW/CCW) — selects the pass-side mapping.
        lateral_offset: Lateral deformation magnitude (m).
        robot_pos: Current robot position (x, y); enables the depth pin.
        context: Tuning-derived constants for the wall-clearance clamp.
            Defaults to the checked-in tuning.
        yaw_drift: Absolute heading change (rad) since the pin engaged on this
            sign; releases the pin past ``PIN_HEADING_GUARD_DEG`` when
            ``PIN_HEADING_GUARD`` is set. See ``_pin_depth``.

    Returns:
        Deformed waypoint (x, y).
    """
    if (corridor, direction) not in _ROUTING_TABLE:
        return waypoint

    axis, red_mult, green_mult = _ROUTING_TABLE[(corridor, direction)]
    mult = red_mult if color == SignColor.RED else green_mult

    wx, wy = waypoint
    if axis == Axis.Y:
        return (
            _pin_depth(wx, sign.x, robot_pos[0] if robot_pos else None, robot_pos, corridor, context, yaw_drift),
            clamp_lateral(sign.y + mult * lateral_offset, corridor, context),
        )
    return (
        clamp_lateral(sign.x + mult * lateral_offset, corridor, context),
        _pin_depth(wy, sign.y, robot_pos[1] if robot_pos else None, robot_pos, corridor, context, yaw_drift),
    )


def _pin_depth(
    waypoint_depth: float,
    sign_depth: float,
    robot_depth: float | None,
    robot_pos: tuple[float, float] | None,
    corridor: Section,
    context: SignRouterContext | None = None,
    yaw_drift: float | None = None,
) -> float:
    """Hold the commanded point abeam the sign instead of letting it recede.

    Applies only while the sign is genuinely between the chassis and the
    lookahead point, in whichever direction the robot is travelling along the
    corridor. Once the robot is level with the sign the condition lapses on its
    own and the ordinary lookahead resumes -- there is no separate "release"
    to get wrong, and a sign already behind never pulls the target backwards.

    ``deform_waypoint`` only checks ``_is_squarely_in_corridor`` once, upstream,
    against the raw (pre-deformation) WAYPOINT -- not against where the robot
    itself actually is. The lookahead target runs 0.2-0.4 m ahead of the robot,
    so the robot can already have curved out of the straight-corridor
    assumption this pin depends on (e.g. mid corner-arc) while the waypoint
    still reads as squarely in the corridor. Pinning to the sign's depth in
    that state drove the commanded point into a wall -- measured as 11 wall
    collisions with the pin on against 0 with it off, all corner-adjacent.
    Re-checking squareness here, against the robot's own real (x, y), closes
    that gap: the pin only fires when both ends of its own logic actually hold.

    That re-check rode in on an unrelated commit ten days after the 11 was
    measured and was never attributed on its own, so it carries its own toggle
    (``PIN_CORNER_GUARD``) -- both arms belong in one harness invocation.

    ``PIN_CORNER_GUARD`` re-checks the robot's POSITION but not its HEADING.
    Traced on go_obstacles_0049 (subset64, sighted): the robot entered a
    corner turn -- yaw rotating 67 deg to 127 deg over 46 ticks -- while its
    raw waypoint position still read squarely in the corridor the whole time,
    so the position guard never released the pin. The commanded point stayed
    frozen abeam a sign for 2.3 s while the chassis was actually mid-turn,
    steering saturated chasing it, and the chassis crashed into a wall.
    ``PIN_HEADING_GUARD`` releases the pin once the robot's heading has
    drifted more than ``PIN_HEADING_GUARD_DEG`` from where it stood when the
    pin first engaged on this sign, which is what the position check misses.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    if robot_depth is None or robot_pos is None:
        return waypoint_depth
    if context.constants.pin_corner_guard and not _is_squarely_in_corridor(
        robot_pos[0], robot_pos[1], corridor, context
    ):
        return waypoint_depth
    if (
        context.constants.pin_heading_guard
        and yaw_drift is not None
        and yaw_drift > context.constants.pin_heading_guard_rad
    ):
        return waypoint_depth
    if min(robot_depth, waypoint_depth) < sign_depth < max(robot_depth, waypoint_depth):
        return sign_depth
    return waypoint_depth


def clamp_lateral(value: float, corridor: Section, context: SignRouterContext | None = None) -> float:
    """Clamp a deformed lateral coordinate clear of the inner square and outer wall.

    SOUTH/WEST corridors border the inner square on their high side (the
    coordinate must stay below ``CORNER_MIN``); NORTH/EAST border it on their
    low side (must stay above ``CORNER_MAX``). Every corridor is also bounded
    on its outer side by the track wall.

    This clamp is asymmetric by nature and that is deliberate but NOT free:
    it keeps the full boundary clearance and hands whatever squeeze remains
    entirely to the sign. On 646 of the corpus's 1282 signs it binds, and the
    resulting lane clears its sign only within +/-28.2 deg of the corridor
    axis (the simulator collides via exact SAT on the oriented chassis, so
    clearance is yaw-dependent -- do not model it as a flat half-diagonal
    threshold).

    Rebalancing it has been measured and REFUTED. A ``pass_lateral`` that
    interpolated a squeezed plateau toward the midpoint of its free gap --
    the maximin placement, clear of both sides at every yaw -- moved the sign
    column exactly as predicted (199 collisions to 168) and the wall column
    far more (3 to 61), for 229/256 against a 202/256 baseline; swept at
    0.25/0.40/0.55/0.70 it was worse at every value. See ``sign_lane``'s
    module docstring for why (3.1 cm of adjustable range against a 6.3-6.6 cm
    crosstrack shortfall) and ``66fa5f2e`` for the reverted implementation.
    Do not re-try a placement change here without first reducing that
    tracking error.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    wall_clearance = _CHASSIS_HALF_DIAGONAL + context.constants.wall_clearance_margin_m
    low_side = corridor in (Section.SOUTH, Section.WEST)
    if low_side:
        value = min(value, TrackDimensions.CORNER_MIN - wall_clearance)
        value = max(value, TrackDimensions.MIN_COORD + wall_clearance)
    else:
        value = max(value, TrackDimensions.CORNER_MAX + wall_clearance)
        value = min(value, TrackDimensions.MAX_COORD - wall_clearance)
    return value


def _match_detection_to_sign(
    observations: list[TrafficSignObservation],
    expected_world_pos: tuple[float, float],
    config: SignRouterConfig,
) -> SignColor | None:
    """Try to confirm sign color using world-coordinate observations.

    Matches each observation against the expected sign world position.
    TrafficSignObservation already carries world coordinates, so no
    pixel-to-world projection is needed.

    Args:
        observations: Current frame observations.
        expected_world_pos: Expected (x, y) world position of the sign.
        config: Router config (confidence threshold, match distance).

    Returns:
        Confirmed SignColor, or None if no confident match.
    """
    best_match_dist = float("inf")
    best_color: SignColor | None = None

    for obs in observations:
        if obs.confidence < config.min_confidence:
            continue
        if obs.color not in (SignColor.RED, SignColor.GREEN):
            continue

        world = (obs.world_x_m, obs.world_y_m)
        d = _dist2d(Waypoint(*world), Waypoint(*expected_world_pos))
        if d < config.detection_match_dist and d < best_match_dist:
            best_match_dist = d
            best_color = obs.color

    return best_color


def signs_from_metadata(metadata: ScenarioMetadata | dict[str, Any]) -> list[SignSpec]:
    """Extract sign specs from scenario metadata.

    Args:
        metadata: Scenario metadata (Pydantic model or coercible dict).

    Returns:
        List of SignSpec for all signs in the scenario.
    """
    if isinstance(metadata, ScenarioMetadata):
        return [SignSpec(x=s.x, y=s.y, color=s.color) for s in metadata.sign_positions]
    sign_positions = metadata.get(DictKeys.SIGN_POSITIONS, [])
    return [
        SignSpec(x=entry[DictKeys.X], y=entry[DictKeys.Y], color=entry[DictKeys.COLOR])
        for entry in sign_positions
    ]


def _is_squarely_in_corridor(
    x: float, y: float, corridor: Section, context: SignRouterContext | None = None
) -> bool:
    """True if this waypoint is still a reasonable candidate for straight-corridor deformation.

    The deformation model holds the depth axis (whatever value the raw path
    already gives it) and overrides only the lateral axis with a value derived
    from the sign's position, then clamps that result into the corridor's own
    free-space band. Two independent checks:

    * Lateral axis (the one being overridden) must still read as this
      corridor, not already the opposite wall.
    * Depth axis (held, never touched) must stay within
      ``DEFORM_DEPTH_BUFFER_M`` of the inner square's own span — not the exact
      ``[CORNER_MIN, CORNER_MAX]`` window ``corridor_for_position()`` uses for
      its own robot-position classification, which is far too strict here: the
      lookahead target runs 0.2-0.4m ahead of the robot, so it's often already
      past that window well before the robot itself is anywhere near a corner,
      and requiring it anyway silently killed deformation through most of a
      sign's real engagement. But with no depth check at all, deformation can
      keep firing long after the robot has geometrically left this corridor
      for the next one, building up an offset that snaps back hard once the
      sign finally disengages by corridor mismatch — this buffer catches that
      case without reintroducing the original over-strict cutoff.
    """
    context = context or _DEFAULT_SIGN_ROUTER_CONTEXT
    deform_depth_buffer = context.constants.deform_depth_buffer_m
    depth_min = TrackDimensions.CORNER_MIN - deform_depth_buffer
    depth_max = TrackDimensions.CORNER_MAX + deform_depth_buffer
    if corridor is Section.SOUTH:
        return y < TrackDimensions.CORNER_MIN and depth_min <= x <= depth_max
    if corridor is Section.NORTH:
        return y > TrackDimensions.CORNER_MAX and depth_min <= x <= depth_max
    if corridor is Section.EAST:
        return x > TrackDimensions.CORNER_MAX and depth_min <= y <= depth_max
    if corridor is Section.WEST:
        return x < TrackDimensions.CORNER_MIN and depth_min <= y <= depth_max
    return False
