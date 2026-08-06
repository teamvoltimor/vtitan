"""Measure where the robot actually is, instead of assuming where it was put.

``start_conditions.start_pose`` computes a starting pose from geometry alone:
the corridor centreline, at the middle of the mat's side. Nothing measures it,
so it is an assertion about the operator's placement rather than an observation
of it -- and the assertion is wrong even when it is close. The marked starting
square spans one metre along the corridor, split into two half-metre cells
centred at 1.25 and 1.75, so the assumed 1.5 sits exactly on the boundary
between them: the one along-corridor position the robot can never legally
occupy.

Measured on real hardware 2026-08-05: two CCW rounds were set down near the far
end of the corridor with 0.69 m of clear track ahead while the plan, built from
the assumed start, expected roughly 1.5 m. The robot drove into the wall in six
seconds with the steering barely off centre, because nothing downstream can
discover a starting error the localizer is not looking for -- its search is
local (see :class:`~src.navigation.localization.LidarLocalizer`), so an error
of that size is permanently outside its reach.

The scan already contains the answer. With the chassis aligned to the corridor,
the four cardinal rays give the distance to the wall ahead, the wall behind and
each side, and those *are* the position, expressed relative to the corridor the
robot is standing in. Measured against the recorded rounds' true poses the four
rays agreed to within 1-4 cm.

Which side of the mat the robot is on is neither knowable nor needed: with equal
corridors the track is symmetric under 90 degree rotation, so the four candidate
sides score identically on any scan, and the robot declares its own starting
section anyway (see ``start_conditions``' module docstring). What is knowable,
and what actually matters, is how far along that corridor it stands and how far
it has before the corner it is driving at.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.config.enums import Direction, Section

_MAT = TrackDimensions.MAX_COORD
_SECTION_ROTATIONS: tuple[Section, ...] = (Section.SOUTH, Section.EAST, Section.NORTH, Section.WEST)
"""Sections in 90-degree rotation order, starting from the frame poses are built in."""

RAY_HALF_WIDTH_RAD: float = math.radians(4.0)
"""Half-width of the wedge each cardinal distance is taken over.

Wide enough to average out per-ray noise, narrow enough that the wedge still
sees one wall: at 2.5 m a 4 degree half-angle spans 17 cm of wall, well inside
a one metre corridor."""

CLOSING_TOLERANCE_M: float = 0.15
"""How far ``forward + back`` may fall short of the mat before the reading is rejected.

Opposite rays along a corridor must span the mat, so their sum is a free
validity check -- it needs no knowledge of where the robot is. The tolerance is
sized from real scans, not nominally: two recorded rounds on a properly set-up
track summed to 2.978 m and 2.971 m against a nominal 3.0, so the honest error
on good data is already 2-3 cm before LIDAR noise, mat seams, or walls that are
not quite square. 0.15 m is five times that, while the failure this rejects --
a hand, a bystander, or a sign standing in one of the rays -- misses by a metre
or more. The margin is deliberately generous: a false rejection costs a re-run,
and a false acceptance costs the round."""


@dataclass(frozen=True, slots=True)
class MeasuredStart:
    """A starting pose read off the track rather than assumed.

    Attributes:
        x: Position in the declared section's frame (m).
        y: Position in the declared section's frame (m).
        distance_ahead_m: Clear track between the robot and the wall it faces.
            The number whose absence caused the 2026-08-05 failures.
        outer_wall_distance_m: Distance to the outer wall, across the corridor.
        corridor_width_m: Width of the corridor the robot stands in, or ``None``
            when it is level with a corner rather than the inner block and both
            side rays reach outer walls, which measures the mat and not a
            corridor.
    """

    x: float
    y: float
    distance_ahead_m: float
    outer_wall_distance_m: float
    corridor_width_m: float | None


def _wedge_median(
    ranges: np.ndarray,
    angles: np.ndarray,
    center_rad: float,
    half_width_rad: float,
) -> float | None:
    """Median valid range in a wedge about ``center_rad``, or None if none are valid.

    Median rather than mean or minimum: a mean is dragged by the occasional
    max-range no-return, and a minimum reports whatever speck is nearest rather
    than the wall the wedge is pointed at.
    """
    delta = np.arctan2(np.sin(angles - center_rad), np.cos(angles - center_rad))
    usable = (
        (np.abs(delta) <= half_width_rad)
        & (ranges > RobotSpecs.LIDAR_MIN_RANGE)
        & (ranges < RobotSpecs.LIDAR_MAX_RANGE * 0.99)
    )
    if not usable.any():
        return None
    return float(np.median(ranges[usable]))


def _rotate_into(section: Section, x: float, y: float) -> tuple[float, float]:
    """Rotate a pose built in the SOUTH frame into ``section``'s frame.

    A quarter turn about the mat's centre maps each section onto the next, and
    with equal corridors the track is invariant under it -- which is exactly why
    the section is a free choice rather than something to be measured.
    """
    turns = _SECTION_ROTATIONS.index(section)
    for _ in range(turns):
        x, y = _MAT - y, x
    return x, y


def measure_start_pose(
    ranges_m: np.ndarray | tuple[float, ...] | list[float],
    angles_rad: np.ndarray | tuple[float, ...] | list[float],
    direction: Direction,
    section: Section = Section.SOUTH,
    closing_tolerance_m: float = CLOSING_TOLERANCE_M,
) -> MeasuredStart | None:
    """Read the robot's pose out of a scan, assuming only its corridor and direction.

    Args:
        ranges_m: LIDAR ranges, already sanitised (no NaN/inf).
        angles_rad: Matching bearings in the robot frame, 0 = forward.
        direction: Inferred travel direction. Needed because it decides which
            way along the corridor "ahead" points, and which side the outer
            wall is on -- clockwise keeps the inner block on the robot's right,
            counterclockwise on its left, so the outer wall is opposite it.
        section: The corridor the robot calls its starting one.
        closing_tolerance_m: See :data:`CLOSING_TOLERANCE_M`.

    Returns:
        The measured start, or ``None`` when the scan cannot support one --
        a ray with no valid return, or opposite rays that do not span the mat,
        which means something is standing in one of them. ``None`` is a refusal
        to guess, and callers should treat it as "do not race", not as "use the
        old assumption".
    """
    ranges = np.asarray(ranges_m, dtype=float)
    angles = np.asarray(angles_rad, dtype=float)

    forward = _wedge_median(ranges, angles, 0.0, RAY_HALF_WIDTH_RAD)
    back = _wedge_median(ranges, angles, math.pi, RAY_HALF_WIDTH_RAD)
    left = _wedge_median(ranges, angles, math.pi / 2, RAY_HALF_WIDTH_RAD)
    right = _wedge_median(ranges, angles, -math.pi / 2, RAY_HALF_WIDTH_RAD)
    if forward is None or back is None or left is None or right is None:
        return None

    if abs((forward + back) - _MAT) > closing_tolerance_m:
        return None

    # Clockwise travel keeps the inner block to the right, so the outer wall is
    # the left ray; counterclockwise is the mirror of that.
    clockwise = direction is Direction.CLOCKWISE
    outer, inner = (left, right) if clockwise else (right, left)

    # Both side rays reaching outer walls measures the mat across, not a
    # corridor: the robot is level with a corner, past the inner block. Only a
    # sum that falls short of the mat is a corridor width.
    span = outer + inner
    corridor_width = span if span < _MAT - closing_tolerance_m else None

    # ``back`` is the distance to the wall behind, so it *is* the along-corridor
    # coordinate when travelling in the axis' positive direction, and the mat
    # less that when travelling against it.
    along = _MAT - back if clockwise else back
    x, y = _rotate_into(section, along, outer)
    return MeasuredStart(
        x=x,
        y=y,
        distance_ahead_m=forward,
        outer_wall_distance_m=outer,
        corridor_width_m=corridor_width,
    )
