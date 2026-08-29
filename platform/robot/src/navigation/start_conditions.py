"""Where the robot believes it starts, without being handed a scenario file.

The starting pose is pure track geometry -- corridor centreline, biased away
from the inner block, aligned with travel -- so it can be computed from the
layout the robot *believes* it is on rather than read from metadata. That is
what lets the deployed navigator run with no scenario file at all.

This lived in ``src/simulation/scenario_builder.py``, which meant the only code
that could derive a start pose was simulation code, and the real node had to be
told one. It is re-exported from there so existing callers are unaffected.

## Why the section can be assumed but the direction cannot

**Section is free.** The robot defines its own world frame: it declares its
starting corridor to be the south one and everything else follows. If it is
physically in the east corridor, its entire map is the true map rotated 90
degrees, and since it learns corridor widths against its own labels the map
stays self-consistent. The path it drives is correct in its own frame, which is
the only frame it acts in.

**Direction is not.** Assuming south-clockwise also asserts *the inner block is
on my right*. If the robot is actually travelling the other way round the loop
the block is on its left, the assumption is wrong by a reflection rather than a
rotation, and no amount of width learning recovers it -- it steers toward the
wall it thinks is the far one. Reflection is not a symmetry of the labelled
track, so travel direction has to come from outside: a launch parameter or a
jumper, the same way the challenge mode does.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, DictKeys, TrackDimensions
from shared.domain.enums import Direction, Section

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.waypoints.generation import center_bias_for_corridor
from src.navigation.race_tracker import TRAVEL_DIRS

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

_TRACK_CENTER = TrackDimensions.MAX_COORD / 2

CANONICAL_SECTION = Section.canonical()
"""The section a robot assumes when it has not been told which one it is in.

Any choice works -- see the module docstring -- so this is a label, not a claim
about where the robot physically is. Sourced from ``Section.canonical()``
rather than restated here so ``StartingConditions``'s own default (which
cannot import this module -- domain layer, see its docstring) stays in step
with this one automatically.
"""


def start_pose(
    section: Section,
    direction: Direction,
    widths_m: dict[str, float],
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> tuple[float, float, float]:
    """Spawn pose on the biased corridor centerline, aligned with travel.

    Args:
        section: Which corridor the robot starts in.
        direction: Travel direction, which sets the yaw.
        widths_m: Corridor widths by lowercase section name.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: When set, EVERY corridor takes this magnitude and the
            narrow/wide split is skipped. None (the default) applies the split,
            which is what the Open planner does.

            Callers that supply it are pinning the pre-split behaviour on
            purpose. The Obstacles Challenge is the one that does: its assumed
            start is computed from the all-narrow prior at the WIDE magnitude,
            so pinning that magnitude leaves the pose bit-identical to what its
            scenarios were tuned against. Verified 2026-08-29: its failure set
            is unchanged by the split, over a matched sweep of the same seven
            test modules at the same worker count.

            Obstacles' assumed start therefore does NOT sit on its own planned
            centreline, which uses OBSTACLES_CENTER_BIAS_M; that inconsistency
            predates the split and is left alone deliberately rather than fixed
            in passing, since changing a separately swept value is its own
            measurement.

    Uses tuning: waypoints.WIDE_CENTER_BIAS_M, NARROW_CENTER_BIAS_M,
    NARROW_WIDTH_THRESHOLD_M, WIDE_CENTER_BIAS_SIDE, NARROW_CENTER_BIAS_SIDE
    """
    tuning = get_tuning(tuning)

    # Per-corridor bias, via the same helper calculate_waypoints uses, so the
    # split is never reimplemented here and cannot drift from the planner's.
    # This matters for Open specifically: the blind prior below is all-narrow,
    # so a blind Open robot plans narrow corridors centred and must assume a
    # start on that same centreline, not on the wide value it used before the
    # split existed.
    def _bias(width_m: float) -> float:
        return center_bias_for_corridor(width_m, tuning, center_bias_m)

    south_cy = widths_m["south"] / 2 + _bias(widths_m["south"])
    north_cy = TrackDimensions.MAX_COORD - widths_m["north"] / 2 - _bias(widths_m["north"])
    east_cx = TrackDimensions.MAX_COORD - widths_m["east"] / 2 - _bias(widths_m["east"])
    west_cx = widths_m["west"] / 2 + _bias(widths_m["west"])
    center = {
        Section.SOUTH: (_TRACK_CENTER, south_cy),
        Section.NORTH: (_TRACK_CENTER, north_cy),
        Section.EAST: (east_cx, _TRACK_CENTER),
        Section.WEST: (west_cx, _TRACK_CENTER),
    }[section]
    normal = TRAVEL_DIRS[(section, direction)]
    return center[0], center[1], math.atan2(normal.ny, normal.nx)


def assumed_start_conditions(
    direction: Direction,
    widths_m: dict[Section, float] | None = None,
    section: Section = CANONICAL_SECTION,
    tuning: NavigationTuning | None = None,
    center_bias_m: float | None = None,
) -> dict:
    """Starting conditions for a robot that was told nothing but the direction.

    Args:
        direction: Travel direction for the round. The one thing that cannot be
            assumed -- see the module docstring.
        widths_m: The layout the robot believes it is on. Defaults to every
            corridor narrow, the safe prior blind operation starts from. Note
            this interacts with the narrow/wide bias split: the default belief
            is NARROW, so a blind robot assumes the narrow bias until it
            measures otherwise, which is what keeps the assumed start on the
            path it will actually be given.
        section: Which corridor the robot calls its starting one.
        tuning: Navigation tuning instance. Defaults to loaded defaults.
        center_bias_m: Centreline-shift override, forwarded to
            :func:`start_pose`. Pass the same value handed to
            ``calculate_waypoints`` for this challenge.

    Returns:
        A ``starting_conditions`` mapping in scenario-metadata shape.

    Uses tuning: waypoints.WIDE_CENTER_BIAS_M, NARROW_CENTER_BIAS_M,
    NARROW_WIDTH_THRESHOLD_M, WIDE_CENTER_BIAS_SIDE, NARROW_CENTER_BIAS_SIDE
    """
    believed = widths_m or dict.fromkeys(Section, CorridorDimensions.NARROW)
    by_name = {s.value.lower(): w for s, w in believed.items()}
    sx, sy, yaw = start_pose(section, direction, by_name, tuning, center_bias_m)
    return {
        DictKeys.DIRECTION: str(direction),
        DictKeys.SECTION: section.capitalized,
        DictKeys.POSITION: {DictKeys.X: sx, DictKeys.Y: sy},
        DictKeys.YAW: yaw,
    }
