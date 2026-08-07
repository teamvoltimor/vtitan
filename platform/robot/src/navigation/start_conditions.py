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

from shared.config.constants import CorridorDimensions, DictKeys, TrackDimensions
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.race_tracker import TRAVEL_DIRS

_TRACK_MAX = TrackDimensions.MAX_COORD
_TRACK_CENTER = _TRACK_MAX / 2

CANONICAL_SECTION = Section.SOUTH
"""The section a robot assumes when it has not been told which one it is in.

Any choice works -- see the module docstring -- so this is a label, not a claim
about where the robot physically is.
"""


def start_pose(
    section: Section,
    direction: Direction,
    widths_m: dict[str, float],
    tuning: NavigationTuning | None = None,
) -> tuple[float, float, float]:
    """Spawn pose on the biased corridor centerline, aligned with travel.

    Uses tuning: waypoints.CENTER_BIAS_M, CENTER_BIAS_SIDE
    """
    if tuning is None:
        tuning = NavigationTuning.load_default()

    # Derive center bias from tuning (positive toward inner block)
    from shared.config.enums import CorridorSide
    center_bias_m = tuning.waypoints.CENTER_BIAS_M * (
        1.0 if tuning.waypoints.CENTER_BIAS_SIDE is CorridorSide.INNER else -1.0
    )

    # Same signs as calculate_waypoints, so the assumed start sits on the
    # path the robot is about to be given rather than beside it.
    south_cy = widths_m["south"] / 2 + center_bias_m
    north_cy = _TRACK_MAX - widths_m["north"] / 2 - center_bias_m
    east_cx = _TRACK_MAX - widths_m["east"] / 2 - center_bias_m
    west_cx = widths_m["west"] / 2 + center_bias_m
    center = {
        Section.SOUTH: (_TRACK_CENTER, south_cy),
        Section.NORTH: (_TRACK_CENTER, north_cy),
        Section.EAST: (east_cx, _TRACK_CENTER),
        Section.WEST: (west_cx, _TRACK_CENTER),
    }[section]
    nx, ny = TRAVEL_DIRS[(section, direction)]
    return center[0], center[1], math.atan2(ny, nx)


def assumed_start_conditions(
    direction: Direction,
    widths_m: dict[Section, float] | None = None,
    section: Section = CANONICAL_SECTION,
    tuning: NavigationTuning | None = None,
) -> dict:
    """Starting conditions for a robot that was told nothing but the direction.

    Args:
        direction: Travel direction for the round. The one thing that cannot be
            assumed -- see the module docstring.
        widths_m: The layout the robot believes it is on. Defaults to every
            corridor narrow, the safe prior blind operation starts from.
        section: Which corridor the robot calls its starting one.
        tuning: Navigation tuning instance. Defaults to loaded defaults.

    Returns:
        A ``starting_conditions`` mapping in scenario-metadata shape.

    Uses tuning: waypoints.CENTER_BIAS_M, CENTER_BIAS_SIDE
    """
    believed = widths_m or dict.fromkeys(Section, CorridorDimensions.NARROW)
    by_name = {s.value.lower(): w for s, w in believed.items()}
    sx, sy, yaw = start_pose(section, direction, by_name, tuning)
    return {
        DictKeys.DIRECTION: str(direction),
        DictKeys.SECTION: section.capitalized,
        DictKeys.POSITION: {DictKeys.X: sx, DictKeys.Y: sy},
        DictKeys.YAW: yaw,
    }
