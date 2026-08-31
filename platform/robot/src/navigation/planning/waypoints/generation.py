"""Waypoint generation orchestration for WRO 2026 track navigation.

Computes the list of Waypoints the TrackNavigator follows. Uses circular arc
waypoints at corners to stay within the Ackermann robot's minimum turning
radius. All functions are pure -- they accept data and return results without
I/O.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from shared.config.constants import CorridorDimensions, RobotSpecs, TrackDimensions
from shared.domain.enums import CorridorSide, Direction, Section
from shared.domain.models import (
    CorridorGeometry,
    CorridorWidthEntry,
    CorridorWidths,
    PathPlannability,
    Position2D,
    ScenarioMetadata,
    Waypoint,
)

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.waypoints.geometry import corner_arc_radius
from src.navigation.planning.waypoints.segments import (
    assemble_loop,
    build_all_segments,
    build_waypoint_sequence,
    validate_bounds,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning


def center_bias_for_corridor(
    width_m: float,
    tuning: NavigationTuning,
    override_m: float | None = None,
) -> float:
    """Signed centreline shift for ONE corridor, positive toward the inner block.

    Narrow corridors take ``NARROW_CENTER_BIAS_M`` and wide ones
    ``WIDE_CENTER_BIAS_M``, split at ``NARROW_WIDTH_THRESHOLD_M``. The same absolute
    shift spends a much larger fraction of a narrow corridor's margin than of a
    wide one's, so a single value is either unsafe narrow or slow wide.

    ``override_m`` (the Obstacles Challenge's ``OBSTACLES_CENTER_BIAS_M``)
    applies UNIFORMLY, ignoring the split. Obstacles corridors are all 1.0 m by
    rule, so there is no narrow case for it to describe, and letting the
    threshold reinterpret an explicitly-passed magnitude would silently change
    a value that was swept and measured as one number.

    Magnitude and SIDE are chosen together, from the same width class -- a
    narrow corridor takes ``NARROW_CENTER_BIAS_SIDE`` as well as
    ``NARROW_CENTER_BIAS_M``. They are one setting expressed as two fields, so
    reading the side from the other class would silently produce a shift
    neither class describes.

    The override takes the WIDE side. Not arbitrary: Obstacles corridors are
    1.0 m by rule, so they ARE the wide class, and the split's threshold would
    classify them that way if it were consulted at all.

    Args:
        width_m: This corridor's width (m).
        tuning: Tuning profile supplying the magnitudes, sides and threshold.
        override_m: Explicit magnitude that replaces both, applied uniformly
            with the wide side.

    Returns:
        Signed shift (m); positive toward the inner block.
    """
    params = tuning.waypoints
    if override_m is not None:
        magnitude, side = override_m, params.WIDE_CENTER_BIAS_SIDE
    elif width_m <= params.NARROW_WIDTH_THRESHOLD_M:
        magnitude, side = params.NARROW_CENTER_BIAS_M, params.NARROW_CENTER_BIAS_SIDE
    else:
        magnitude, side = params.WIDE_CENTER_BIAS_M, params.WIDE_CENTER_BIAS_SIDE
    return magnitude * (1.0 if side is CorridorSide.INNER else -1.0)


def validate_path_feasibility(min_corridor_width_m: float, center_bias_m: float) -> PathPlannability:
    """Check whether the chassis fits the narrowest corridor once biased off centre.

    The corner arcs deliberately do not appear here. The corner-arc helper caps
    every arc at the clearance the straights already have, so a corner can never
    be the tightest point on the path.

    The previous form added ``arc_radius`` to a lateral half-extent, which are
    not commensurable -- one is a path curvature, the other a width -- and it
    ignored ``center_bias_m`` entirely, so it scored a centred path and a biased
    one identically while the bias was the thing actually spending the margin.

    Args:
        min_corridor_width_m: Minimum corridor width across all four sides.
        center_bias_m: Signed offset of the path from the centreline, positive
            toward the inner block. Only the magnitude matters: biasing either
            way moves the chassis toward one wall by the same amount.

    Returns:
        PathPlannability with margin and reason, in corridor-width units. The
        margin is the total slack across both walls; clearance to the nearer
        wall is half of it.
    """
    required = RobotSpecs.WIDTH + 2 * abs(center_bias_m)
    margin = min_corridor_width_m - required
    if margin > 0:
        return PathPlannability(
            is_feasible=True,
            min_required_m=required,
            min_available_m=min_corridor_width_m,
            margin_m=margin,
        )
    return PathPlannability(
        is_feasible=False,
        min_required_m=required,
        min_available_m=min_corridor_width_m,
        margin_m=margin,
        reason=f"Corridor too narrow: required {required:.3f} m, got {min_corridor_width_m:.3f} m",
    )


def corridor_widths_dict_to_model(widths: dict[Section, float]) -> CorridorWidths:
    """Convert a per-section width belief (metres) into the on-model mm representation."""
    return CorridorWidths(
        **{s.value: CorridorWidthEntry(width_mm=round(width * 1000)) for s, width in widths.items()},
    )


def calculate_waypoints(
    metadata: ScenarioMetadata | dict[str, Any],
    num_laps: int,
    arc_radius: float | None = None,
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> list[Waypoint]:
    """Build the full multi-lap waypoint sequence for a scenario.

    Reads corridor widths and starting conditions from the scenario metadata
    and constructs arc waypoints at corners plus straight waypoints along each
    corridor centerline.

    Args:
        metadata: Scenario metadata (Pydantic model or coercible dict).
        num_laps: Total laps the robot must complete.
        arc_radius: Ceiling on the corner arc radius (m). Each corner picks its
            own radius from the two corridors it joins -- see the corner-arc
            helper -- and this only caps the result, so it no longer sets the
            geometry on its own. Must exceed the Ackermann minimum turning
            radius (~0.034 m, from WHEELBASE/tan(MAX_STEERING_ANGLE) with
            counter-phase steering). Defaults to the tuning profile's value so a
            loaded profile actually takes effect instead of a value frozen at
            import time.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: Centreline shift MAGNITUDE (m), overriding the tuning
            default. ``None`` keeps the tuning value, so every existing caller
            is unchanged. The Obstacles Challenge passes its own bias (0.0,
            centred) -- see the tuning field for why the inner bias is an
            Open-Challenge-only argument.

    Returns:
        Ordered list of world-frame Waypoints starting near the robot's spawn
        position, covering num_laps full loops.

    Raises:
        ValueError: If a generated or deformed waypoint would fall outside the
            track or inside the restricted inner square.

    Uses tuning: waypoints.ARC_RADIUS, WIDE_CENTER_BIAS_M, NARROW_CENTER_BIAS_M,
    NARROW_WIDTH_THRESHOLD_M, WIDE_CENTER_BIAS_SIDE, NARROW_CENTER_BIAS_SIDE
    """
    tuning = get_tuning(tuning)
    if not isinstance(metadata, ScenarioMetadata):
        metadata = ScenarioMetadata.model_validate(metadata)
    arc_radius = arc_radius if arc_radius is not None else tuning.waypoints.ARC_RADIUS

    corridor_widths = metadata.corridor_widths
    starting = metadata.starting_conditions
    if starting.direction is None:
        msg = (
            "calculate_waypoints requires a resolved starting_conditions.direction; "
            "callers must infer/assign it (see TrackNavigator._plan / ScenarioSimulator._plan) "
            "before planning a path"
        )
        raise ValueError(msg)
    direction = starting.direction

    cw_entries = {
        Section.NORTH: corridor_widths.north,
        Section.SOUTH: corridor_widths.south,
        Section.EAST: corridor_widths.east,
        Section.WEST: corridor_widths.west,
    }
    min_width_m = min(cw.width_mm for cw in cw_entries.values()) / 1000.0
    # The narrowest corridor is the one that can fail to fit, and it is also the
    # one carrying the narrow bias, so score feasibility with ITS bias rather
    # than a single track-wide figure -- the wide value would overstate what the
    # narrow corridor actually spends.
    feasibility = validate_path_feasibility(min_width_m, center_bias_for_corridor(min_width_m, tuning, center_bias_m))
    if not feasibility.is_feasible:
        raise ValueError(feasibility.reason)

    widths = {section: cw.width_mm / 1000.0 for section, cw in cw_entries.items()}
    north_width = widths[Section.NORTH]
    south_width = widths[Section.SOUTH]
    east_width = widths[Section.EAST]
    west_width = widths[Section.WEST]

    # Each corridor takes the bias for ITS OWN width (see
    # center_bias_for_corridor): narrow ones are planned centred, wide ones keep
    # the inner racing line. An explicit magnitude still overrides both
    # uniformly, which is how the Obstacles Challenge plans down the middle
    # without touching Open's values.
    north_bias = center_bias_for_corridor(north_width, tuning, center_bias_m)
    south_bias = center_bias_for_corridor(south_width, tuning, center_bias_m)
    east_bias = center_bias_for_corridor(east_width, tuning, center_bias_m)
    west_bias = center_bias_for_corridor(west_width, tuning, center_bias_m)

    # Signs put the bias toward the inner block on every side: north and east
    # corridors have the block below/left of them, south and west above/right.
    north_cy = TrackDimensions.MAX_COORD - north_width / 2 - north_bias
    south_cy = south_width / 2 + south_bias
    east_cx = TrackDimensions.MAX_COORD - east_width / 2 - east_bias
    west_cx = west_width / 2 + west_bias

    # Each corner is sized by the two corridors it joins, so a narrow-to-narrow
    # corner tightens while the rest keep the configured radius.
    #
    # The bias passed is the WIDER corridor's, because corner_arc_radius is
    # tangent to that corridor's centreline (it takes max(entry, exit)) and the
    # radius has to preserve the clearance THAT straight has. Passing the
    # narrow side's bias would size the arc against a centreline it is not
    # tangent to -- the same class of error the max/min choice already guards.
    # CORNER_ARC_ASSUME_WIDE sizes every corner as if both corridors were WIDE,
    # which makes the arc -- and therefore the turn-entry point, `r` back from a
    # 90 deg corner -- independent of a belief that starts out wrong.
    #
    # Blind rounds begin believing every corridor NARROW, so a narrow->wide
    # corner plans a 0.300 m entry where the true geometry wants 0.450 m and the
    # robot commits 0.15 m late (0.38 s at the medium tier). Confirming wide
    # needs MIN_SAMPLES=12 readings ~= 0.5 m of travel, so the correction
    # generally arrives AFTER the entry point has already passed: late is the
    # default on every corner touching a wide corridor, not an edge case.
    #
    # Turning early into a corridor wider than planned is the safe direction to
    # be wrong; turning late is what puts the nose in the outer wall.
    effective = (
        (lambda entry_w, exit_w: (CorridorDimensions.WIDE, CorridorDimensions.WIDE))
        if tuning.waypoints.CORNER_ARC_ASSUME_WIDE
        else (lambda entry_w, exit_w: (entry_w, exit_w))
    )
    corner_radii = {
        corner: corner_arc_radius(
            *effective(entry_w, exit_w),
            center_bias_for_corridor(max(entry_w, exit_w), tuning, center_bias_m),
            arc_radius,
        )
        for corner, (entry_w, exit_w) in {
            "se": (east_width, south_width),
            "sw": (south_width, west_width),
            "nw": (west_width, north_width),
            "ne": (north_width, east_width),
        }.items()
    }

    segments = build_all_segments(
        north_cy,
        south_cy,
        east_cx,
        west_cx,
        corner_radii,
        direction,
        tuning,
    )

    order = Section.loop_order(starting.section, direction)

    full_loop = assemble_loop(order, segments)
    start_x, start_y = starting.position.x, starting.position.y

    waypoints = build_waypoint_sequence(
        full_loop,
        segments,
        order,
        start_x,
        start_y,
        num_laps,
        tuning,
    )
    validate_bounds(waypoints)
    return waypoints


def plan_believed_path(
    metadata: ScenarioMetadata,
    geometry: CorridorGeometry,
    *,
    direction: Direction,
    believed_section: Section,
    believed_position: Position2D,
    believed_yaw: float,
    arc_radius: float | None,
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> list[Waypoint]:
    """Build a one-lap path for the layout the robot currently believes it is on.

    Shared by ``ScenarioSimulator._plan`` and ``TrackNavigator._plan``: both
    replan from a *believed* corridor-width estimate and a believed start pose
    that can differ from ``metadata.starting_conditions`` (the ground-truth /
    on-file record), which is why the believed section/position/yaw/direction
    are threaded in separately rather than read off ``metadata`` itself.

    ``num_laps=1`` is deliberate: :func:`calculate_waypoints` bakes the lap
    count into the returned list, but ``CoreNavigator`` already cycles one
    canonical lap the real lap count times (see its waypoint-wrap in ``step()``).
    Passing the real count here would multiply laps (e.g. 3 -> 9).

    Args:
        metadata: Validated scenario metadata; only its ``starting_conditions``
            and structure are used, both re-derived below with the believed
            widths/pose swapped in.
        geometry: Believed corridor geometry (widths per section, m). The model
            is the single currency for corridor widths; the legacy
            ``dict[Section, float]`` form is obtained via ``geometry.to_widths_dict()``.
        direction: Believed travel direction.
        believed_section: Section the robot believes it is standing in.
        believed_position: Position the robot believes it is standing at.
        believed_yaw: Heading the robot believes it is facing.
        arc_radius: Ceiling on the corner arc radius (m), forwarded to
            :func:`calculate_waypoints`.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: Centreline shift magnitude (m); forwarded to
            :func:`calculate_waypoints`. ``None`` keeps the tuning value.

    Returns:
        Single-lap ordered list of world-frame Waypoints.
    """
    new_widths = corridor_widths_dict_to_model(geometry.to_widths_dict())
    new_starting = metadata.starting_conditions.replanned_at(
        direction=direction,
        section=believed_section,
        position=believed_position,
        yaw=believed_yaw,
    )
    planning_metadata = metadata.replanned_with(
        corridor_widths=new_widths,
        starting_conditions=new_starting,
    )
    return calculate_waypoints(
        planning_metadata,
        num_laps=1,
        arc_radius=arc_radius,
        tuning=tuning,
        center_bias_m=center_bias_m,
    )
