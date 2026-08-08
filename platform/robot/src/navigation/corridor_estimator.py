"""Estimate corridor widths from LIDAR alone, without being told the layout.

Every other part of navigation is handed ``metadata["corridor_widths"]`` — the
planned path is built from it and :class:`~src.navigation.localization.LidarLocalizer`
matches scans against a wall model built from it. That is only legitimate if
someone measured the mat and wrote the file first: WRO places the inner walls
randomly before each round, so the true widths cannot be known in advance.

This closes that gap. It exploits the one thing the rules *do* guarantee: each
corridor is either 0.6 m or 1.0 m. So the robot never has to measure a width,
only decide between two values 0.4 m apart — against a 0.03 m LIDAR sigma, a
better-than-4-sigma call.

The measurement needs no map and no position estimate. The LIDAR sits at the
chassis centre, so the range directly left plus the range directly right spans
wall to wall through the robot, wherever in the corridor it happens to be.
Measured over all 28 Open Challenge fixtures: 100% classification accuracy on
14839 usable ticks, mean error +0.03 cm.

Two gates keep bad readings out:

* **Alignment.** The side rays only span the corridor when the chassis is
  roughly parallel to it; off-axis they cut a longer diagonal.
* **Plausibility.** At a corner the inward ray misses the inner block entirely
  and runs off down the next corridor, giving a nonsense total.

Unknown corridors report as ``NARROW`` rather than ``None``, because that is
the *safe* assumption: planning a 1.0 m corridor as if it were 0.6 m puts the
path nearer the outer wall, which stays inside the true corridor. The converse
does not — planning a 0.6 m corridor as if it were 1.0 m puts the path 0.15 m
from the inner block face, closer than the chassis half-diagonal (0.180 m), so
the corner would clip it mid-turn.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import CorridorWidthMeasurement

from src.navigation.race_tracker import TRAVEL_DIRS
from src.navigation.utils import _nearest_ray, _wrap

if TYPE_CHECKING:
    from collections.abc import Sequence

_NARROW = CorridorDimensions.NARROW
_WIDE = CorridorDimensions.WIDE

_DECISION_BOUNDARY = (_NARROW + _WIDE) / 2.0
"""0.8 m — the only threshold needed, halfway between the two legal widths."""

_MIN_PLAUSIBLE_WIDTH = _NARROW - 0.25
_MAX_PLAUSIBLE_WIDTH = _WIDE + 0.25
"""Outside this band the inward ray has missed the inner block (a corner)."""


def measure_corridor_width(
    ranges_m: Sequence[float],
    angles_rad: Sequence[float],
    yaw: float,
    tuning: NavigationTuning | None = None,
) -> CorridorWidthMeasurement | None:
    """Wall-to-wall width through the robot, or ``None`` if this scan can't say.

    Args:
        ranges_m: LIDAR ranges.
        angles_rad: Matching robot-frame bearings (0 = forward).
        yaw: Current heading (radians, world frame).
        tuning: Navigation tuning instance. Defaults to the default tuning profile.

    Returns:
        CorridorWidthMeasurement with plausibility and alignment flags, or
        ``None`` when the chassis is too far off the corridor axis or the
        total is not physically plausible.
    """
    if tuning is None:
        tuning = NavigationTuning.load_default()

    # Heading error against the nearest track axis; corridors always run along one.
    axis_error = _wrap(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2))
    is_aligned = abs(axis_error) <= tuning.direction_estimator.ALIGNMENT_TOLERANCE_RAD

    left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
    right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
    width = (left + right) * math.cos(axis_error) if is_aligned else 0.0
    is_plausible = _MIN_PLAUSIBLE_WIDTH < width < _MAX_PLAUSIBLE_WIDTH

    if is_aligned and is_plausible:
        return CorridorWidthMeasurement(
            width_m=width,
            is_plausible=True,
            is_aligned=True,
            alignment_error_rad=abs(axis_error),
            side_range_left_m=left,
            side_range_right_m=right,
        )
    return None


def classify_width(width_m: float) -> float:
    """Snap a raw measurement to whichever of the two legal widths it is."""
    return _NARROW if width_m < _DECISION_BOUNDARY else _WIDE


def section_from_heading(yaw: float, direction: Direction) -> Section:
    """Which corridor the robot is in, from heading alone.

    On a rectangular loop driven in a known direction, each section is
    travelled along a different bearing — south-bound on the east side going
    clockwise, north-bound on the west side, and so on — so the four
    (section, direction) travel vectors are all distinct. Snapping the IMU
    heading to the nearest axis therefore identifies the corridor outright.

    This deliberately avoids using the position estimate. Attributing a width
    measurement via position is circular: the position comes from matching
    against a wall model built from the widths being estimated, so a wrong
    belief mis-attributes the reading that would have corrected it, and the
    error locks in. Heading breaks that loop — it comes from the IMU and owes
    nothing to the map.

    Args:
        yaw: Heading in radians, world frame (0 = +x).
        direction: Travel direction for this round, chosen before the start.
    """
    heading = (math.cos(yaw), math.sin(yaw))
    return max(
        (s for (s, d) in TRAVEL_DIRS if d == direction),
        key=lambda s: heading[0] * TRAVEL_DIRS[(s, direction)][0] + heading[1] * TRAVEL_DIRS[(s, direction)][1],
    )


class CorridorWidthEstimator:
    """Running per-section estimate of the track layout, from LIDAR only.

    Starts from ``assumed_width`` and re-classifies each corridor only after
    repeated agreeing observations.

    The default prior is narrow, which is the safe one for the Open Challenge
    (see the module docstring): its corridors are independently 60 or 100 cm, so
    neither value is a better guess than the other and the tighter one fails
    safe. **The Obstacles Challenge is not that round.** Its corridors are all
    100 cm, which is a rule of the event and therefore knowable before the
    robot is placed — exactly like "the track is 3x3 m and the loop is
    rectangular". Assuming narrow there is not conservative, it is *known to be
    wrong for every corridor*, and it costs real runs: measured over the 16
    obstacles fixtures, 9 of 16 blind collisions happened in a corridor still
    held at the narrow default, because the robot turns into a corridor and
    meets a traffic sign there before ``_MIN_SAMPLES`` readings have accumulated
    to correct it. In the Open Challenge that same latency is harmless -- there
    is nothing in the corridor to hit.

    The estimator still runs and can still override the prior: this changes
    where it starts, not whether it measures.
    """

    def __init__(self, min_samples: int | None = None, assumed_width: float = _NARROW) -> None:
        if min_samples is None:
            min_samples = NavigationTuning.load_default().corridor_estimator.MIN_SAMPLES
        self._min_samples = min_samples
        self._widths: dict[Section, float] = dict.fromkeys(Section, assumed_width)
        self._observed: set[Section] = set()
        self._votes: dict[Section, list[int]] = {s: [0, 0] for s in Section}
        """Per section, ``[narrow_votes, wide_votes]``."""

    @property
    def widths(self) -> dict[Section, float]:
        """Current best estimate for every section (metres)."""
        return dict(self._widths)

    @property
    def observed_sections(self) -> set[Section]:
        """Sections that have been confirmed at least once, rather than assumed."""
        return set(self._observed)

    @property
    def is_complete(self) -> bool:
        """True once every corridor has been measured rather than assumed."""
        return len(self._observed) == len(Section)

    def observe(
        self,
        section: Section,
        ranges_m: Sequence[float],
        angles_rad: Sequence[float],
        yaw: float,
    ) -> bool:
        """Fold one scan into the estimate for ``section``.

        Returns:
            ``True`` if this observation changed the estimate, so the caller
            knows to replan against the new layout.
        """
        m = measure_corridor_width(ranges_m, angles_rad, yaw)
        if m is None:
            return False
        return self.observe_measurement(section, m.width_m)

    def observe_measurement(self, section: Section, measured: float) -> bool:
        """Fold in a width already measured by :func:`measure_corridor_width`.

        Separate from :meth:`observe` so a reading can be taken before it can
        be attributed. A blind round infers its travel direction after it has
        started driving, and attribution needs that direction, so the scans
        from before it settles would otherwise be discarded -- which throws
        away the cleanest readings of the starting corridor and leaves the
        first surviving ones to be taken at a corner, where the side rays span
        the *next* corridor. Buffer the measurements, replay them here once the
        direction is known.
        """
        votes = self._votes[section]
        votes[1 if measured >= _DECISION_BOUNDARY else 0] += 1
        if sum(votes) < self._min_samples:
            return False

        verdict = _WIDE if votes[1] > votes[0] else _NARROW
        self._observed.add(section)
        if math.isclose(self._widths[section], verdict):
            return False
        self._widths[section] = verdict
        return True
