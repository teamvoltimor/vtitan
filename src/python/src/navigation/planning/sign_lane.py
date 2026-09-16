"""Obstacles-only lane planning: shift the PATH past a sign, not just the carrot.

``SignRouter.deform_waypoint`` nudges only the pure-pursuit carrot, so
``cross_track_error`` (the controller's own error signal and the input
``select_lookahead`` gates on) stays near zero and the offset closes only
asymptotically. Seven levers on that chase measured flat or worse. This module
changes the maneuver instead of its tuning: it rewrites the corridor's
straight-segment waypoints onto a pass-side LANE, so crosstrack, the lookahead
gate and the target search all agree the lane is the path, and the lateral
travel is spread over the whole straight instead of the last 1.4 m.

Measured rationale, tables and refutations live in
``adr:0051-sign-lane-planner`` and ``adr:0064-corridor-by-depth-and-clearance-budget``;
the WRO layout invariants this planner leans on (depth values, signs per
section, two signs 1.00 m apart, no sign in a corner, the clamp binding on most
signs) are in ``adr:0064-corridor-by-depth-and-clearance-budget``.

Obstacles-only by construction: the transform is driven by the routed sign list,
and the Open Challenge has no ``SignRouter``, so with no signs the returned path
is the input path, byte for byte.

The transform is 1:1 (same waypoint count, same order, only lateral coordinates
change), so every index-keyed invariant in ``CoreNavigator``
(``_waypoint_index`` advance, the lap-seam wrap, ``replace_path``'s re-seek)
survives it unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from shared.config.constants import TrackDimensions
from shared.domain.enums import Direction, Section
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_router import (
    Axis,
    SignSpec,
    clamp_lateral,
    pass_lateral,
    pass_side_lateral_axis,
)

__all__ = ["SignLaneParams", "apply_sign_lanes"]


@dataclass(frozen=True, slots=True)
class SignLaneParams:
    """Geometry of the lane transition. See ``SignRouterParams`` for the tunables."""

    lateral_offset: float
    """Lateral distance from the sign's own centre to the lane (m).

    The same magnitude ``SignRouter`` deforms by (chassis half-diagonal + sign
    half-width + margin), so the lane and the carrot ask for the same line --
    they differ in WHEN the robot is asked to be on it, not WHERE it is.
    """

    ramp_m: float
    """Along-corridor distance to transition on and off the lane (m)."""

    hold_m: float
    """Along-corridor half-width of the full-offset hold around a sign (m)."""

    split_overlap: bool | None = None
    """Split overlapping plateaux at their midpoint instead of letting one dip through another.

    Legal WRO geometry never overlaps -- a section holds at most two signs, 1.00 m
    apart, against a 0.50 m plateau -- so an overlap is always a DISCOVERY
    artifact. See ``SIGN_LANE_SPLIT_OVERLAP``.
    """

    skip_unsatisfiable: bool | None = None
    """Drop a sign whose clamped target is on the FORBIDDEN side of it.

    Such a sign cannot be satisfied by any lane geometry -- the instruction
    itself violates the rule -- so the choice is between planning a violating
    line and planning none. See ``SIGN_LANE_SKIP_UNSATISFIABLE``.
    """

    corner_entry_m: float | None = None
    """How far past the corridor's straight the lane may extend, into the
    corner arcs either side (m). A bare construction resolves it from the
    shipped ``SIGN_LANE_CORNER_ENTRY_M``; an explicit value overrides.

    Non-zero because most signs sit at a section BOUNDARY, where the straight
    offers no near-side runway: the lane would reach full offset on its first
    waypoint, against a corner arc still on the centreline, and the step lands
    at the corner exit beside the inner square. Safe to spend corner arc as
    runway because no sign ever occupies a corner, so ``clamp_lateral`` still
    bounds every point it moves. See ``adr:0051-sign-lane-planner`` and
    ``adr:0064-corridor-by-depth-and-clearance-budget``.
    """

    gap_centre_frac: float | None = None
    """How far a squeezed plateau moves off the boundary-clearance limit
    toward the midpoint of its free gap. ``0.0`` is the clamped placement.

    See ``adr:0064-corridor-by-depth-and-clearance-budget`` for the geometry,
    the yaw margins and the shipped value (1.0, reversing an obsolete
    refutation).
    """

    def __post_init__(self) -> None:
        """Resolve unspecified policy fields from the shipped tuning group.

        These three rested as dataclass defaults whose values restated
        ``SignRouterParams`` (whose defaults come from
        signs/sign_router.toml); the documented copies in this family have
        diverged before, and ``corner_entry_m`` in fact disagrees with the
        shipped 0.50. Resolving unset fields from the same group the
        navigator passes from makes the bare construction agree with the one
        that races by construction, instead of by concurrent manual edits.
        """
        from src.config.tuning_helpers import get_tuning  # noqa: PLC0415

        get_tuning(None).sign_router.resolve_unset(self, prefix="SIGN_LANE_")


def _axis_coords(wp: Waypoint, axis: Axis) -> tuple[float, float]:
    """Return ``(lateral, depth)`` for a waypoint under ``axis``."""
    return (wp.y, wp.x) if axis is Axis.Y else (wp.x, wp.y)


def _rebuild(wp: Waypoint, axis: Axis, lateral: float) -> Waypoint:
    """Rebuild a waypoint with its lateral coordinate replaced."""
    return Waypoint(wp.x, lateral) if axis is Axis.Y else Waypoint(lateral, wp.y)


def _lane_span(corner_entry_m: float) -> tuple[float, float]:
    """Depth window the lane may rewrite, straight plus any borrowed corner."""
    return (
        TrackDimensions.CORNER_MIN - corner_entry_m,
        TrackDimensions.CORNER_MAX + corner_entry_m,
    )


def _in_lane_span(wp: Waypoint, corridor: Section, axis: Axis, corner_entry_m: float) -> bool:
    """True if ``wp`` is a point this corridor's lane may move.

    The depth test widens by ``corner_entry_m`` to borrow corner-arc runway
    (see ``SignLaneParams.corner_entry_m``); the LATERAL test does not widen,
    and that is what keeps the widening safe. A corner arc leaves this
    corridor's side of the track partway round, so once its points have
    crossed into the neighbouring corridor's band they stop qualifying on
    their own -- the borrowed region self-limits to the part of the arc still
    geometrically belonging to this corridor, rather than running away around
    the whole turn.

    A variant that widened the lateral test too (out to the far edge of the
    corner square, gated to only apply at a sign's own plateau depth) was tried
    and measured catastrophically WORSE on the full 256-scenario corpus, then
    reverted; a widened lateral bound at that depth pulls in swaths of the
    NEIGHBOURING corridor's own arc points too. Do not re-try without bounding
    the widened window far more tightly. See ``adr:0051-sign-lane-planner``.
    """
    lateral, depth = _axis_coords(wp, axis)
    low, high = _lane_span(corner_entry_m)
    if not low <= depth <= high:
        return False
    if corridor in (Section.SOUTH, Section.WEST):
        return lateral < TrackDimensions.CORNER_MIN
    return lateral > TrackDimensions.CORNER_MAX


def _control_points(
    signs: list[tuple[SignSpec, Section]],
    corridor: Section,
    axis: Axis,
    base_lateral: float,
    params: SignLaneParams,
    direction: Direction | None,
) -> list[tuple[float, float]]:
    """Piecewise-linear ``(depth, lateral)`` profile for one corridor's lane.

    Each sign contributes a full-offset plateau spanning ``hold_m`` either side
    of its own depth, and the profile ramps to the corridor's base lateral
    ``ramp_m`` beyond the outermost plateau at each end.

    Two signs in the same corridor requiring OPPOSITE sides simply produce two
    plateaux at opposite laterals with a straight interpolation between them:
    an S-bend. Averaging their influence instead would command the centreline
    between two obstacles the robot must pass on opposite sides. This is the
    common case, and the S-bend is comfortable because two signs in a section
    always sit exactly 1.00 m apart (see ``adr:0064-corridor-by-depth-and-clearance-budget``).
    """
    plateaux: list[tuple[float, float]] = []
    for spec, sign_corridor in signs:
        rule = pass_side_lateral_axis(sign_corridor, SignColor(spec.color), direction)
        if rule is None:
            continue
        _, mult = rule
        sign_lateral, sign_depth = _axis_coords(Waypoint(spec.x, spec.y), axis)
        target = pass_lateral(
            sign_lateral, mult, corridor, params.lateral_offset, params.gap_centre_frac
        )
        if params.skip_unsatisfiable and (target - sign_lateral) * mult <= 0.0:
            # The clamp put this sign's own target on the forbidden side of it,
            # so every point of its plateau would violate the rule it exists to
            # obey. Emit nothing rather than a line that is wrong by
            # construction; the corridor's other signs still get their plateaux.
            continue
        plateaux.append((sign_depth, target))

    if not plateaux:
        return []

    points: list[tuple[float, float]] = []
    plateaux.sort(key=lambda entry: entry[0])
    for index, (sign_depth, target) in enumerate(plateaux):
        low = sign_depth - params.hold_m
        high = sign_depth + params.hold_m
        if params.split_overlap:
            # A plateau nested inside another one puts a HOLE in the enclosing
            # sign's hold window: the profile dips to the neighbour's target
            # exactly where the robot is abeam this sign. Traced on a WEST
            # corridor holding three specs -- our plateau ran 2.15..2.65 at
            # lateral 0.781 while a neighbour's endpoints sat inside it at 2.22
            # and 2.24 at 0.325, and the plan passed the sign on that dip.
            #
            # Legal WRO geometry cannot produce this: a section holds at most
            # two signs and they sit 1.00 m apart, against a 0.50 m plateau. The
            # overlaps come from DISCOVERY emitting several specs per physical
            # sign (measured 2.50x). Splitting the overlap at the midpoint gives
            # each sign the half nearer itself, so every sign keeps a flat hold
            # over the stretch where it is actually passed.
            if index > 0:
                low = max(low, (plateaux[index - 1][0] + sign_depth) / 2.0)
            if index + 1 < len(plateaux):
                high = min(high, (sign_depth + plateaux[index + 1][0]) / 2.0)
            if high < low:
                low = high = sign_depth
        points.append((low, target))
        points.append((high, target))

    if not points:
        return []

    points.sort(key=lambda p: p[0])
    # Ramp endpoints are clamped into the corridor's own straight span rather
    # than allowed to fall outside it. A sign near the middle of a 1.45 m
    # straight puts the nominal ramp start behind CORNER_MIN, where there are
    # no straight waypoints to carry it -- the profile would then be truncated
    # at the boundary and the first straight waypoint would already sit a
    # sizeable step off the centreline, while the corner arc immediately
    # before it (deliberately untouched, see _on_corridor_straight) is still
    # exactly on it. That step is a kink in the path at precisely the place
    # the chassis is finishing a turn. Clamping instead COMPRESSES the ramp:
    # it still reaches the centreline, just sooner and more steeply, so the
    # lane meets the arc continuously.
    # Overlapping plateaux (signs closer together than 2 * hold_m) would
    # otherwise leave the profile non-monotonic in depth, which the
    # interpolation below reads as a fold-back. Collapsing ties to a shared
    # depth turns the fold into an instantaneous transition at that depth --
    # ugly, but it commands a real line rather than an undefined one, and the
    # ramps on either side still carry the chassis onto it.
    for i in range(1, len(points)):
        if points[i][0] < points[i - 1][0]:
            points[i] = (points[i - 1][0], points[i][1])

    low, high = _lane_span(params.corner_entry_m)
    entry = min(max(points[0][0] - params.ramp_m, low), points[0][0])
    exit_ = max(min(points[-1][0] + params.ramp_m, high), points[-1][0])
    return [(entry, base_lateral), *points, (exit_, base_lateral)]



def _clamp_shift(value: float, corridor: Section, lane: float, gap_centre_frac: float) -> float:
    """Bound a shifted waypoint, without undoing a deliberately gap-centred lane.

    ``clamp_lateral`` is doing two jobs here. For a borrowed CORNER ARC point
    it is real protection: the shift translates the whole arc, and an arc that
    already curves toward the boundary can be pushed through it. For the
    PLATEAU it is redundant -- ``pass_lateral`` has already placed that value
    and bounded it -- and worse than redundant under gap-centring, because its
    margin is precisely what gap-centring rebalances. Applied blindly it pulls
    the plateau straight back to the boundary-clearance limit, silently
    reducing the whole change to a different ramp shape.

    So the clamp is relaxed exactly as far as ``lane``, the profile value for
    this depth, and no further: a point may reach the lane the planner chose,
    while anything overshooting BEYOND it -- which is only ever arc curvature,
    never the plateau -- is still caught. At ``gap_centre_frac`` 0 this is
    ``clamp_lateral`` unchanged, since ``lane`` is then the clamped value
    itself and the relaxation has nothing to give.
    """
    clamped = clamp_lateral(value, corridor)
    if gap_centre_frac <= 0.0:
        return clamped
    if value < clamped:
        return max(value, min(clamped, lane))
    if value > clamped:
        return min(value, max(clamped, lane))
    return clamped


def _interpolate(profile: list[tuple[float, float]], depth: float) -> float | None:
    """Lateral value of ``profile`` at ``depth``; ``None`` outside its span."""
    if depth < profile[0][0] or depth > profile[-1][0]:
        return None
    for (d0, l0), (d1, l1) in pairwise(profile):
        if d0 <= depth <= d1:
            if d1 == d0:
                return l1
            return l0 + (l1 - l0) * (depth - d0) / (d1 - d0)
    return profile[-1][1]




@dataclass(frozen=True, slots=True)
class _LanePlan:
    """One corridor's lane, planned against the unmodified path."""

    corridor: Section
    axis: Axis
    indices: list[int]
    base_lateral: float
    profile: list[tuple[float, float]]
    signs: list[tuple[SignSpec, Section]]


def _nearest_sign_m(wp: Waypoint, corridor_signs: list[tuple[SignSpec, Section]]) -> float:
    """Distance from ``wp`` to the closest sign this corridor is routing around."""
    return min(math.hypot(wp.x - spec.x, wp.y - spec.y) for spec, _ in corridor_signs)


def _assign_owners(plans: list[_LanePlan], waypoints: list[Waypoint]) -> dict[int, int]:
    """One owning lane per waypoint, so no point is shifted twice.

    Corner runway is BORROWED, and two corridors either side of a corner borrow
    the SAME arc, so the shifts used to compound: the second corridor read a
    lateral the first had already moved and added its own offset on top, and
    most contested points ended up somewhere NEITHER lane asked for. See
    ``adr:0051-sign-lane-planner`` for the measured count.

    A contested point goes to the lane whose own sign is nearest to it, which is
    the lane whose pass that point actually serves. Exact ties keep the earlier
    plan, making the result independent of how discovery happened to order its
    specs.

    This is deliberately NOT one continuous lane through the corner. A lane is a
    one-dimensional profile over a corridor's lateral axis, and that axis
    rotates 90 degrees at a corner, so a single profile cannot span one.
    Expressing the offset along the path NORMAL instead would make a continuous
    lane fall out by construction, but it would change every shift on the track
    rather than only the contested ones, so it is a separate change with its own
    measurement.
    """
    owner: dict[int, int] = {}
    best: dict[int, float] = {}
    for position, plan in enumerate(plans):
        for i in plan.indices:
            distance = _nearest_sign_m(waypoints[i], plan.signs)
            if i not in best or distance < best[i]:
                best[i] = distance
                owner[i] = position
    return owner


def apply_sign_lanes(
    waypoints: list[Waypoint],
    signs: list[tuple[SignSpec, Section]],
    params: SignLaneParams,
    direction: Direction | None,
) -> list[Waypoint]:
    """Return ``waypoints`` with each signed corridor's straight shifted onto its pass-side lane.

    Args:
        waypoints: The planned path (one canonical lap). Not mutated.
        signs: ``(spec, corridor)`` for every sign still being routed around,
            corridor as the router itself labels it (``_sign_corridors``) --
            not recomputed here, so a discovery estimate that has been held
            steady against corner jitter stays steady in the lane too.
        params: Lane geometry.
        direction: The round's travel direction. REQUIRED because the pass-side
            rule is travel-relative -- the vehicle passes to its own right of a
            red pillar, which is the outer wall counterclockwise and the inner
            square clockwise. ``None`` (direction not yet settled) makes every
            lane unbuildable and returns the path unchanged, which is the
            correct conservative answer: a lane laid on a guessed direction is
            a wrong-side pass half the time, and that ENDS THE ROUND (9.24.5).

    Returns:
        A new list of the same length and order. Identical to the input when
        ``signs`` is empty, which is every Open Challenge run.
    """
    if not signs or not waypoints:
        return list(waypoints)

    result = list(waypoints)
    by_corridor: dict[Section, list[tuple[SignSpec, Section]]] = {}
    for entry in signs:
        by_corridor.setdefault(entry[1], []).append(entry)

    # Two passes. Every lane is planned against the UNMODIFIED path, then the
    # shifts are applied. Planning against a partly-shifted path made each
    # corridor's geometry depend on how many corridors happened to be processed
    # before it -- see _assign_owners for what that cost on the corner arcs.
    plans: list[_LanePlan] = []
    for corridor, corridor_signs in by_corridor.items():
        rule = pass_side_lateral_axis(corridor, SignColor(corridor_signs[0][0].color), direction)
        if rule is None:
            continue
        axis, _ = rule

        indices = [i for i, wp in enumerate(waypoints) if _in_lane_span(wp, corridor, axis, params.corner_entry_m)]
        if not indices:
            continue
        # The corridor's own centreline as PLANNED, which already carries
        # whichever center bias its width selected, and this scenario's
        # estimated corridor width -- taking
        # it from the waypoints rather than recomputing it keeps the lane's
        # ramp endpoints on the path the tracker would otherwise follow, so the
        # transform is continuous at both ends by construction.
        #
        # Measured over the STRAIGHT only, even when corner runway is being
        # borrowed: an arc's lateral coordinate sweeps away from the
        # centreline as it turns, so including arc points would drag this
        # median off the centreline it is supposed to represent.
        straight = [i for i in indices if _in_lane_span(waypoints[i], corridor, axis, 0.0)]
        laterals = sorted(_axis_coords(waypoints[i], axis)[0] for i in (straight or indices))
        base_lateral = laterals[len(laterals) // 2]

        profile = _control_points(corridor_signs, corridor, axis, base_lateral, params, direction)
        if not profile:
            continue

        plans.append(_LanePlan(corridor, axis, indices, base_lateral, profile, corridor_signs))

    owner = _assign_owners(plans, waypoints)

    for position, plan in enumerate(plans):
        corridor, axis = plan.corridor, plan.axis
        base_lateral, profile = plan.base_lateral, plan.profile
        for i in plan.indices:
            if owner[i] != position:
                continue
            lateral, depth = _axis_coords(waypoints[i], axis)
            lane = _interpolate(profile, depth)
            if lane is None:
                continue
            # Apply the profile as a SHIFT from the centreline, not as an
            # absolute lateral. On the straight the two are identical (every
            # point already sits at base_lateral), but on borrowed corner
            # runway they are not: an arc point's own lateral is partway
            # through the turn, and assigning it the profile's absolute value
            # would snap it back onto the corridor centreline -- destroying
            # the turn instead of offsetting it. Shifting translates the arc
            # while leaving its shape intact, which is exactly the "exit the
            # corner already on the lane" behaviour this borrows runway for.
            shifted = _clamp_shift(
                lateral + (lane - base_lateral), corridor, lane, params.gap_centre_frac
            )
            if shifted == lateral:
                continue
            result[i] = _rebuild(result[i], axis, shifted)

    return result
