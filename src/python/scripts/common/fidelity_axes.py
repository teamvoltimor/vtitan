"""The axis definitions the hardware and simulator fidelity probes SHARE.

``scripts/bag/diag_bag_fidelity_axes.py`` measures a recorded round and
``scripts/sim/diag_fidelity_axes.py`` measures a simulated one. The whole point
of that pair is that their two outputs can be read side by side, and that only
works if every axis is defined ONCE. Before the shared band table, the two
sides disagreed on the LIDAR sector bands, which is exactly the boundary a
sim-vs-hardware divergence audit hinges on; see
``adr:0086-simulator-realism``. A shared band table makes that class of mistake
impossible.

Nothing here reads a bag or drives a simulator. It holds the definitions both
sides must agree on:

* :data:`SECTOR_BANDS` -- the LIDAR bearing bands, in the ROBOT frame.
* :func:`contiguous_episodes` -- how a run of manoeuvre ticks becomes one episode.
* :class:`SectorCensus` -- streaming accumulation of finite/sub-floor ray shares.
* :class:`VisionFreshness` and :func:`frame_carries_observation` -- the VISION
  axis, which this module did not own until 2026-09-16 and which the two halves
  had therefore drifted apart on twice over. See below.
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

SECOND TRAP, and the reason the vision axis moved in here: an axis this module
does not OWN is an axis the two halves will define differently, and they did.
Both biases were measured on ``run_20260915_140358`` and both inflated the
hardware side:

1. The bag half counted a frame as vision if ``"red"`` or ``"green"`` appeared
   anywhere in ``str(payload)``. That counts detections production THROWS AWAY:
   ``bag_io.decode_detections`` skips a record with no usable bbox, and
   ``sign_discovery.detection_to_observation`` then drops anything whose aspect
   ratio is not a pillar's. The three numbers on that round are 50.2% raw,
   39.7% typed, and 32.2% surviving to an observation -- so the loose match
   overstated the real axis by more than half again.
2. The bag half divided by ALL nav ticks and the simulator half by GATEWAY
   POLLS, which on hardware are 81.3% of ticks. Two shares with different
   denominators printed in the same line shape read as one comparison and are
   not one.

So the hardware number is the PRODUCTION-OBSERVATION share -- what the navigator
could actually act on -- and every printed share now names its own denominator.
Measured under those definitions: hardware 32.2% / 31.7% / 24.9% on
``run_20260915_140358`` / ``140852`` / ``141413``, mean 29.6%, against a
simulator that returns a detection on 29.6% of polls at
``simulation.vision_frame_miss_rate = 0.30``. That agreement is what calibrated
the knob, and it only exists because both sides finally mean the same thing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from shared.config.constants import RobotSpecs

from scripts.common.stats import percentile
from src.navigation.planning.sign_discovery import detection_to_observation

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

    from shared.domain.models import Detection, Pose

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
and NOT the second, so the two bands carry very different finite shares. A
single ``120-180`` band averages those two into a number that matches neither
and hides the wedge. See ``adr:0080-lidar-mount-and-scan-plane`` for the
measured shares.
"""

SUB_FLOOR_M = 0.044
"""The ``min_valid_range_m`` the sector filter keeps rays above.

A ray below this is dropped before navigation sees it, so counting it as a
return overstates what the robot had to work with. Tracked separately rather
than subtracted, because on hardware it is a share of all rays large enough to
be a finding in itself; see ``adr:0086-simulator-realism``.
"""


HARDWARE_POLLS_PER_NAV_TICK = 0.813
"""Camera polls per navigation tick on hardware, measured 2026-09-15.

The bag half's natural denominator is NAV TICKS, because a tick is the unit the
navigator acts on. The simulator half's is GATEWAY POLLS, because that is what
it can instrument. Those are not the same denominator and this is the measured
ratio between them.

Stated here rather than corrected FOR. Scaling one share into the other assumes
the ticks that carry no poll are a uniform sample of the rest, and nothing has
established that -- they are concentrated wherever the loop ran long, which is
exactly where the camera matters. So declare which denominator a number used,
which is what :func:`format_vision_freshness` prints on every line.
"""


@dataclass(frozen=True)
class VisionFreshness:
    """One side's answer to "how often did the navigator have fresh colour to act on?".

    ``opportunities`` and ``denominator`` travel TOGETHER on purpose. The whole
    defect this type exists to prevent is a share whose denominator is implicit:
    the bag half counted nav ticks and the simulator half counted gateway polls,
    and printed both in the same line shape as if they were one axis.
    """

    hits: int
    opportunities: int
    denominator: str

    @property
    def share_pct(self) -> float:
        """``hits`` as a percentage of ``opportunities``; 0.0 when nothing was counted."""
        return 100.0 * self.hits / self.opportunities if self.opportunities else 0.0


def frame_carries_observation(detections: Iterable[Detection], pose: Pose) -> bool:
    """Does this camera frame yield a colour observation PRODUCTION would keep?

    The gate is the shipped one, called directly rather than approximated:
    ``detection_to_observation`` rejects a non-red/green class, a bbox too short
    to range from, and -- the one that actually bites -- a box whose aspect ratio
    is not a pillar's, which is what keeps the parking barrier out of the sign
    map. A probe that counts raw detector output instead is measuring a quantity
    no navigator ever saw: on ``run_20260915_140358`` that is 50.2% of ticks
    against the 32.2% production kept.

    ``pose`` only places the observation in the world; it cannot change whether
    one survives, so an interpolated pose is good enough here even though it
    would not be for a position measurement.
    """
    return any(detection_to_observation(det, pose) is not None for det in detections)


def format_vision_freshness(freshness: VisionFreshness, *, extra: str = "") -> list[str]:
    """The vision block, identical on both sides, with the denominator NAMED.

    ``extra`` carries whatever is true of one source only -- camera fps and the
    raw/typed/kept funnel on a bag, poll count in the simulator -- and is
    printed BELOW the shared line so the shared line stays diffable.
    """
    lines = [
        f"vision: fresh usable colour observation on {freshness.share_pct:.1f}% of "
        f"{freshness.opportunities} {freshness.denominator} "
        f"(n={freshness.hits}; production gate: decode + pillar aspect)"
    ]
    if extra:
        lines.append(f"  {extra}")
    lines.append(
        f"  denominator: {freshness.denominator}. The bag half counts NAV TICKS and the simulator half "
        f"GATEWAY POLLS, which on hardware are {100 * HARDWARE_POLLS_PER_NAV_TICK:.1f}% of ticks -- read the "
        f"two shares as near-comparable, not as identical"
    )
    return lines


def contiguous_episodes(flags: Sequence[bool]) -> list[tuple[int, int]]:
    """Runs of consecutive ``True`` in ``flags``, as inclusive ``(first, last)`` index pairs.

    One manoeuvre is one episode. Counting ticks instead inflates a slow escape
    over a fast one; counting transitions of a COUNTER instead is what
    ``escape_count`` does, and that counter resets on the escape's own reverse,
    so it reads 1 on almost every trigger and cannot be used to segment episodes;
    see ``adr:0055-escape-maneuver-selection``. Segment on the manoeuvre being
    active, which is a state and not a tally.
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
    turn more than 180 degrees, and wrapping there silently reports the short way
    round; see ``adr:0055-escape-maneuver-selection``. Unwrap the whole series
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
    delivers only a fraction of it at creep speeds, so a planner tuned against
    the former plans distances the latter never covers. See
    ``adr:0076-drivetrain-and-steering-hardware`` for the delivered fractions.
    """
    commanded = list(commanded_mps)
    achieved = list(achieved_mps)
    share = 100 * committed_ticks / max(total_ticks, 1)
    return (
        f"committed ticks={committed_ticks} ({share:.1f}%) "
        f"cmd_speed p50={percentile(commanded, 0.5):.3f} p10={percentile(commanded, 0.1):.3f}  "
        f"achieved p50={percentile(achieved, 0.5):.3f}"
    )


FLANK_HALF_FOV_DEG = 15.0
"""Half-width of the abeam cone the flank gap is read from.

Wide enough to survive the C1's dropouts (the hardware loses a quarter of its
rays) and narrow enough that a pillar 45 degrees off the beam does not stand in
for the wall.
"""


def flank_gaps(bearings_deg, ranges_m, *, max_range_m: float) -> tuple[float | None, float | None]:
    """Closest LEFT and RIGHT returns abeam, from the chassis side, or None each.

    Pose-free on purpose. The believed pose wanders 7-15 cm inside one round,
    which is the same size as the lane differences this axis exists to measure,
    so the wall has to be read off the sensor rather than off the belief. The
    gap is measured from the chassis SIDE (half the width), not the LIDAR
    origin, so the number is the clearance the paint would see.
    """
    bearings = np.asarray(bearings_deg, dtype=float)
    ranges = np.asarray(ranges_m, dtype=float)
    valid = np.isfinite(ranges) & (ranges > SUB_FLOOR_M) & (ranges < max_range_m - 1e-6)
    out: list[float | None] = []
    for centre in (90.0, -90.0):
        delta = (bearings - centre + 180.0) % 360.0 - 180.0
        side = valid & (np.abs(delta) <= FLANK_HALF_FOV_DEG)
        out.append(float(np.min(ranges[side])) - RobotSpecs.WIDTH / 2 if bool(np.any(side)) else None)
    return out[0], out[1]


def format_flank(
    left: Sequence[float | None], right: Sequence[float | None], *, corridor_width_m: float
) -> list[str]:
    """The lane-placement block: how close the FLANKS run to the walls, and how centred.

    Why this is an axis at all: the parking lot's fins stand 0.20 m out from the
    outer wall and the car brushes them, while the simulator drives the same
    corridor 0.30 m further from that wall and so never touches them. A margin
    knob measured in the simulator is then measuring a car that was never close
    enough to need it. The offset is signed as RIGHT MINUS LEFT halved, i.e.
    positive means hugging the left wall, and it is only meaningful when both
    flanks answered -- in a corner one of them sees the far side of the mat.
    """
    lefts = [v for v in left if v is not None]
    rights = [v for v in right if v is not None]
    if not lefts or not rights:
        return ["flank: too few abeam returns to characterise"]
    nearer = [min(a, b) for a, b in zip(left, right, strict=True) if a is not None and b is not None]
    offsets = [(b - a) / 2 for a, b in zip(left, right, strict=True) if a is not None and b is not None]
    hug = 100 * float(np.mean(np.asarray(nearer) < 0.10)) if nearer else 0.0
    return [
        f"flank gap m: left p10={percentile(lefts, 0.1):.2f} p50={percentile(lefts, 0.5):.2f}  "
        f"right p10={percentile(rights, 0.1):.2f} p50={percentile(rights, 0.5):.2f}",
        f"  nearer flank p10={percentile(nearer, 0.1):.2f} p50={percentile(nearer, 0.5):.2f}  "
        f"under 0.10 m: {hug:.1f}% of ticks  (corridor {corridor_width_m:.2f} m)",
        f"  offset from centre m (right-left)/2: p10={percentile(offsets, 0.1):+.2f} "
        f"p50={percentile(offsets, 0.5):+.2f} p90={percentile(offsets, 0.9):+.2f}  n={len(offsets)}",
    ]
