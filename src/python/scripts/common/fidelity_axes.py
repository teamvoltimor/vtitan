"""The axis definitions the hardware and simulator fidelity probes SHARE.

``scripts/bag/diag_bag_fidelity_axes.py`` measures a recorded round and
``scripts/sim/diag_fidelity_axes.py`` measures a simulated one. The whole point
of that pair is that their two outputs can be read side by side, and that only
works if every axis is defined ONCE. The 2026-09-15 divergence audit ran them as
two independent throwaway scripts and the LIDAR sector bands drifted apart
between them -- the hardware side bucketed ``120-180`` and the sim side split it
``120-160`` / ``160-180``, which is exactly the boundary the audit turned out to
hinge on. A shared band table makes that class of mistake impossible.

Nothing here reads a bag or drives a simulator. It holds the definitions both
sides must agree on:

* :data:`SECTOR_BANDS` -- the LIDAR bearing bands, in the ROBOT frame.
* :func:`contiguous_episodes` -- how a run of manoeuvre ticks becomes one episode.
* :class:`SectorCensus` -- streaming accumulation of finite/sub-floor ray shares.
* the ``format_*`` helpers -- identical line shapes, so a diff of the two
  outputs is a diff of the ROBOT, not of two authors' f-strings.

TRAP, and it is the one that voided a whole night of measurement: the bands are
stated in the ROBOT frame, and a bag's ``/scan`` is in the SENSOR frame. They
differ by ``RobotSpecs.lidar_yaw_offset_rad()``, which is 180 degrees on this
mount because ``robot.toml`` sets ``lidar.inverted = true``. Feed raw
``msg.ranges`` angles in here and every band lands on the opposite side of the
car. Use ``bag_io.scan_to_ranges_angles``, which applies the offset, or add
``bag_io.LIDAR_YAW_OFFSET_RAD`` yourself. See
``diag_bag_lidar_frame_census.py``, which exists to make that error visible.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from scripts.common.stats import percentile

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

SECTOR_BANDS: tuple[tuple[str, float, float], ...] = (
    ("front<25", 0.0, 25.0),
    ("corner25-60", 25.0, 60.0),
    ("side60-120", 60.0, 120.0),
    ("rear120-160", 120.0, 160.0),
    ("tail160-180", 160.0, 180.0),
)
"""Absolute bearing bands in the ROBOT frame, in degrees.

Not arbitrary: ``120-160`` and ``160-180`` are split rather than lumped into one
rear band because the occlusion the mount actually produces sits in the first
and NOT the second -- measured 22-48 percent finite against 75-84 percent on the
same bags. A single ``120-180`` band averages those two into a number that
matches neither and hides the wedge.
"""

SUB_FLOOR_M = 0.044
"""The ``min_valid_range_m`` the sector filter keeps rays above.

A ray below this is dropped before navigation sees it, so counting it as a
return overstates what the robot had to work with. Tracked separately rather
than subtracted, because on hardware it is 7 percent of all rays and a share
that large is itself a finding.
"""


def contiguous_episodes(flags: Sequence[bool]) -> list[tuple[int, int]]:
    """Runs of consecutive ``True`` in ``flags``, as inclusive ``(first, last)`` index pairs.

    One manoeuvre is one episode. Counting ticks instead inflates a slow escape
    over a fast one; counting transitions of a COUNTER instead is what
    ``escape_count`` does, and that counter resets on the escape's own reverse
    (measured 2026-09-15: it reads 1 on 96 percent of triggers), so it cannot be
    used to segment episodes. Segment on the manoeuvre being active, which is a
    state and not a tally.
    """
    episodes: list[tuple[int, int]] = []
    start: int | None = None
    for i, on in enumerate(flags):
        if on and start is None:
            start = i
        elif not on and start is not None:
            episodes.append((start, i - 1))
            start = None
    if start is not None:
        episodes.append((start, len(flags) - 1))
    return episodes


def signed_yaw_delta(later: float, earlier: float) -> float:
    """``later - earlier`` wrapped into ``(-pi, pi]``, for a single small step.

    Only valid between ADJACENT samples. Across a whole escape the chassis can
    turn more than 180 degrees (measured: a k_turn burst reached 275), and
    wrapping there silently reports the short way round. Unwrap the whole series
    with ``numpy.unwrap`` and subtract, which is what both probes do.
    """
    return (later - earlier + math.pi) % (2 * math.pi) - math.pi


class SectorCensus:
    """Streaming share of finite and sub-floor rays per :data:`SECTOR_BANDS` band.

    Accumulates over many scans so the answer is a property of the round rather
    than of whichever scan a script happened to print. Both probes feed it the
    same way: absolute bearings in the ROBOT frame, plus the raw range array.
    """

    def __init__(self) -> None:
        self._finite = dict.fromkeys((b[0] for b in SECTOR_BANDS), 0)
        self._sub = dict.fromkeys((b[0] for b in SECTOR_BANDS), 0)
        self._total = dict.fromkeys((b[0] for b in SECTOR_BANDS), 0)
        self.scans = 0

    def add(self, bearings_deg: np.ndarray, ranges_m: np.ndarray, *, finite: np.ndarray) -> None:
        """Fold one scan in.

        ``finite`` is passed explicitly rather than derived, because the two
        sides mean different things by it: a bag carries NaN/inf for a dropout,
        while the simulator substitutes ``LIDAR_MAX_RANGE`` and never emits a
        non-finite float. Each probe decides what "the sensor returned nothing"
        means for its own source and says so.
        """
        self.scans += 1
        abs_deg = np.abs(bearings_deg)
        for name, lo, hi in SECTOR_BANDS:
            in_band = (abs_deg >= lo) & (abs_deg < hi)
            self._total[name] += int(in_band.sum())
            self._finite[name] += int((in_band & finite).sum())
            self._sub[name] += int((in_band & finite & (ranges_m < SUB_FLOOR_M)).sum())

    def format(self) -> str:
        """One line: ``band=finite%/sub%`` for every band, in band order."""
        parts = []
        for name, _lo, _hi in SECTOR_BANDS:
            total = max(self._total[name], 1)
            parts.append(f"{name}={100 * self._finite[name] / total:.0f}%/sub{100 * self._sub[name] / total:.0f}%")
        return "lidar " + "  ".join(parts)


def format_escapes(
    *,
    kinds: dict[str, int],
    durations_s: Iterable[float],
    yaw_deltas_deg: Iterable[float],
    short_episodes: int,
) -> list[str]:
    """The escape block, identical on both sides.

    ``short_episodes`` is the count of episodes lasting two ticks or fewer --
    the signature of a reverse that the rear-gap fit clipped to nothing, which
    is a different failure from an escape that ran and did not help.
    """
    durations = list(durations_s)
    deltas = list(yaw_deltas_deg)
    return [
        f"escapes n={len(durations)} kinds={kinds}",
        f"  dur s      p50={percentile(durations, 0.5):.2f} p90={percentile(durations, 0.9):.2f}  ticks<=2: {short_episodes}",
        f"  |dyaw| deg p50={percentile(deltas, 0.5):.1f} p90={percentile(deltas, 0.9):.1f}  >20deg: {sum(1 for d in deltas if d > 20)}",
    ]


def format_steering(values: Sequence[float], span_s: float) -> list[str]:
    """The steering block, identical on both sides.

    ``values`` are normalised steering commands on NON-manoeuvre ticks only.
    Manoeuvre ticks are excluded because an escape saturates the servo by
    design, and leaving them in makes a car that drives straight look like one
    that thrashes.
    """
    array = np.asarray(values, dtype=float)
    if array.size < 2:
        return ["steer: too few ticks to characterise"]
    steps = np.abs(np.diff(array))
    flips = int(np.sum(np.sign(array[1:]) * np.sign(array[:-1]) < 0))
    return [
        f"steer norm: |v| p50={percentile(np.abs(array), 0.5):.2f} p90={percentile(np.abs(array), 0.9):.2f} "
        f"changed={100 * float(np.mean(steps > 1e-6)):.0f}% |d|/tick p50={percentile(steps, 0.5):.3f} "
        f"p90={percentile(steps, 0.9):.3f}",
        f"  saturated(|v|>0.95)={100 * float(np.mean(np.abs(array) > 0.95)):.1f}%  "
        f"sign flips/s={flips / max(span_s, 1e-6):.2f}",
    ]


def format_committed(
    *,
    committed_ticks: int,
    total_ticks: int,
    commanded_mps: Iterable[float],
    achieved_mps: Iterable[float],
) -> str:
    """The cruise block: how much of the round ran with a sign committed, and how fast.

    Commanded and achieved are printed together because their RATIO is the
    finding. The simulator tracks its own command almost exactly; the chassis
    delivers 0.43-0.84 of it at creep speeds, and a planner tuned against the
    former plans distances the latter never covers.
    """
    commanded = list(commanded_mps)
    achieved = list(achieved_mps)
    share = 100 * committed_ticks / max(total_ticks, 1)
    return (
        f"committed ticks={committed_ticks} ({share:.1f}%) "
        f"cmd_speed p50={percentile(commanded, 0.5):.3f} p10={percentile(commanded, 0.1):.3f}  "
        f"achieved p50={percentile(achieved, 0.5):.3f}"
    )
