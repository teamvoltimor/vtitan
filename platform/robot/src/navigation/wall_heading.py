"""Read absolute heading off the walls, to bound what the gyro cannot.

Heading is the only quantity in the state estimate that nothing corrects.
Position is anchored by :class:`~src.navigation.localization.LidarLocalizer`
matching scans against the walls, but that solver takes yaw *as given* -- so
yaw comes from the IMU alone, and a BNO085 in UART-RVC mode is 6-axis with no
magnetometer and no absolute reference. Its error is a ramp, not a bound.

The measurements say that is the axis that decides rounds. Across the 28 Open
Challenge fixtures, 20 cm of position error costs nothing at all, while
0.1 deg/s of gyro drift takes 28/28 to 6/28 and 0.5% of gyro scale error takes
it to 25/28.

## The observation

The track is a Manhattan world: the outer boundary and every face of the inner
block run along x or y. So in the robot frame every wall segment lies at
``-yaw`` modulo 90 degrees, and a single sweep therefore *observes* absolute
heading -- mod 90, with the ambiguity resolved by whichever of the four
candidates is nearest the IMU's own estimate.

That makes heading bounded by the same geometry that already bounds position,
using a sensor the robot already has, and it does not care how long the round
has been running.

## How

Consecutive returns that lie on the same surface give a segment whose direction
is a wall direction. Taking the circular mean of ``4 * theta`` folds the four
90-degree-apart candidates onto one another, so they reinforce instead of
cancelling; dividing the resulting angle by 4 recovers the offset.

Traffic signs and parking blocks are also axis-aligned in this track, so they
contribute to the same estimate rather than corrupting it -- but a *rotated*
obstacle would, which is why the estimate is a weighted vote with a
concentration test rather than a mean over everything returned.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from shared.config.navigation_tuning import NavigationTuning

if TYPE_CHECKING:
    from collections.abc import Sequence

_QUARTER = math.pi / 2


@dataclass(frozen=True, slots=True)
class _WallHeadingConstants:
  """Tuning-derived wall heading constants, computed on-demand instead of frozen at module level."""
  min_concentration: float
  baseline_rays: int
  max_segment_jump_m: float
  min_segment_m: float
  near_max_range_m: float
  min_returns: int

  @classmethod
  def from_tuning(cls, tuning: NavigationTuning) -> _WallHeadingConstants:
    wh = tuning.wall_heading
    return cls(
        min_concentration=wh.MIN_CONCENTRATION,
        baseline_rays=wh.BASELINE_RAYS,
        max_segment_jump_m=wh.MAX_SEGMENT_JUMP_M,
        min_segment_m=wh.MIN_SEGMENT_M,
        near_max_range_m=wh.NEAR_MAX_RANGE_M,
        min_returns=wh.MIN_RETURNS,
    )


class WallHeadingContext:
  """Context holding tuning-derived wall-heading constants."""

  def __init__(self, tuning: NavigationTuning | None = None) -> None:
    """Initialize wall-heading context from tuning."""
    self.tuning = tuning or NavigationTuning()
    self.constants = _WallHeadingConstants.from_tuning(self.tuning)


_DEFAULT_WALL_HEADING_CONTEXT = WallHeadingContext()

# For backward compatibility, expose constants at module level but via context
MIN_CONCENTRATION = _DEFAULT_WALL_HEADING_CONTEXT.constants.min_concentration
"""How aligned the segment directions must be before the estimate is used.

The circular mean's resultant length on a rectilinear scan runs high; a low
value means the returns disagree about where the walls are, which happens
mid-corner, against a rotated obstacle, or when most rays are no-returns.
Reporting nothing is correct there -- the IMU carries heading between
corrections, so a skipped scan costs only that scan.
"""

_BASELINE_RAYS = _DEFAULT_WALL_HEADING_CONTEXT.constants.baseline_rays
"""How far apart the two returns forming a segment are taken.

Not adjacent, which is the obvious choice and does not work. At a typical
0.7 m wall distance neighbouring rays land 12 mm apart, against 30 mm of range
noise -- so the direction of an adjacent-point segment is mostly noise pointing
radially. Measured on one fixture, the concentration of adjacent segments is
0.08 with noise against 0.99 without: the signal is entirely buried.

Stepping 15 rays gives a baseline around 190 mm at that distance, six times the
noise, and lifts concentration back above 0.7. Longer would be steadier still
but starts spanning corners, where the segment joins two surfaces and means
nothing.
"""

_MAX_SEGMENT_JUMP_M = _DEFAULT_WALL_HEADING_CONTEXT.constants.max_segment_jump_m
"""Range step above which the two returns are treated as different surfaces.

Scaled for the baseline above: along a single flat wall the range genuinely
changes across 15 rays, more so at shallow incidence, so the adjacent-ray
threshold would reject the very segments being looked for. Still far below a
corridor width, so a ray pair spanning the inner block and the outer wall is
rejected.
"""

_MIN_SEGMENT_M = _DEFAULT_WALL_HEADING_CONTEXT.constants.min_segment_m
"""Segments shorter than this are dominated by range noise, not wall direction."""

_NEAR_MAX_RANGE_M = _DEFAULT_WALL_HEADING_CONTEXT.constants.near_max_range_m
"""Returns at or beyond this are no-return rays sanitised to max range."""

_MIN_RETURNS = _DEFAULT_WALL_HEADING_CONTEXT.constants.min_returns
"""Fewer returns than this cannot form a segment at all."""


def estimate_yaw_from_walls(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    prior_yaw: float,
) -> float | None:
    """Absolute yaw implied by the wall directions in this sweep.

    Args:
        ranges_m: LIDAR ranges, already sanitised (no NaN/inf).
        angles_rad: Matching robot-frame bearings (0 = forward).
        prior_yaw: Current heading estimate, used only to pick which of the
            four 90-degree-apart candidates is meant. It does not pull the
            answer -- the returned value is decided by the walls.

    Returns:
        Absolute yaw in the world frame, or ``None`` when the sweep does not
        show a clear rectilinear structure.
    """
    ranges = np.asarray(ranges_m, dtype=float)
    angles = np.asarray(angles_rad, dtype=float)
    if ranges.size < _MIN_RETURNS:
        return None

    # Points in the robot frame.
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    k = _BASELINE_RAYS
    if ranges.size <= k:
        return None
    dx = xs[k:] - xs[:-k]
    dy = ys[k:] - ys[:-k]
    seg_len = np.hypot(dx, dy)

    # Both endpoints must be real returns on one surface.
    valid = (
        (ranges[:-k] < _NEAR_MAX_RANGE_M)
        & (ranges[k:] < _NEAR_MAX_RANGE_M)
        & (np.abs(ranges[k:] - ranges[:-k]) < _MAX_SEGMENT_JUMP_M)
        & (seg_len > _MIN_SEGMENT_M)
    )
    if not np.any(valid):
        return None

    theta = np.arctan2(dy[valid], dx[valid])
    # Fold the four candidates together: 4*theta maps directions 90 degrees
    # apart onto the same angle, so they reinforce rather than cancel.
    # Weighted by segment length, so a long wall outvotes a short fragment.
    weights = seg_len[valid]
    resultant = np.sum(weights * np.exp(4j * theta)) / np.sum(weights)

    if abs(resultant) < MIN_CONCENTRATION:
        return None

    # Offset of the wall grid in the robot frame, in [-45, 45) degrees.
    offset = math.atan2(resultant.imag, resultant.real) / 4.0
    # Robot-frame wall direction is -yaw mod 90, so yaw is -offset mod 90.
    candidate = -offset
    # Choose the candidate nearest the prior; the walls decide the value, the
    # prior only decides which quadrant is meant.
    k = round((prior_yaw - candidate) / _QUARTER)
    return candidate + k * _QUARTER


def heading_error(measured_yaw: float, prior_yaw: float) -> float:
    """Signed difference between a wall-derived yaw and the current estimate."""
    return math.atan2(math.sin(measured_yaw - prior_yaw), math.cos(measured_yaw - prior_yaw))
