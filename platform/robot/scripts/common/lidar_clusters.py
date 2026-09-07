"""Pillar-shaped LIDAR cluster detection, shared by the bag and sim proposer diags.

The LIDAR-proposes/camera-confirms question is asked twice: against recorded
hardware (`scripts/bag/diag_bag_lidar_proposer.py`), where there is no ground
truth and the camera is only a weak anchor, and against the simulator
(`scripts/sim/diag_sim_lidar_proposer.py`), where the sign layout is known
exactly. Both must run the SAME detector, or a difference between them says
something about the detector rather than about the sensors -- so it lives here
instead of being copied into each.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from scripts.common.stats import percentile

if TYPE_CHECKING:
    from collections.abc import Sequence

    from src.navigation.ports import LidarScan


_MIN_SCAN_POINTS = 3
"""Below this a scan cannot contain a cluster with a step on both sides."""

_MIN_WALL_RANGE_M = 0.10
"""Returns nearer than this are rear self-returns off the chassis (the mount
scalar sits at ~0.08 m, INSIDE the body) and would read as a narrow corridor."""


@dataclass(slots=True)
class Cluster:
    """One pillar-shaped return in a single scan, in the robot frame."""

    range_m: float
    bearing_rad: float
    chord_m: float


@dataclass(slots=True)
class Track:
    """A cluster associated across scans, in world coordinates."""

    xs: list[float] = field(default_factory=list)
    ys: list[float] = field(default_factory=list)
    chords: list[float] = field(default_factory=list)
    wall_dists: list[float] = field(default_factory=list)
    """Distance to the NEARER corridor wall, on the ticks where both walls were
    measurable. Shorter than `xs`: a mid-corner or one-wall tick contributes a
    position but no usable lateral offset."""
    widths: list[float] = field(default_factory=list)
    """Corridor width measured on those same ticks, to audit the wall estimate."""
    first_range_m: float = 0.0
    first_t_s: float = 0.0

    @property
    def hits(self) -> int:
        return len(self.xs)

    @property
    def wall_dist_m(self) -> float | None:
        """Median distance to the nearer wall, or None if never measurable.

        The lattice prior in track coordinates: a sign stands 0.4 m from one
        lateral border or the other, so its distance to the NEARER wall is 0.4 m
        whichever slot it occupies. A wall corner sits at ~0.
        """
        return percentile(self.wall_dists, 0.5) if self.wall_dists else None

    @property
    def width_m(self) -> float | None:
        """Median corridor width measured while this track was in view."""
        return percentile(self.widths, 0.5) if self.widths else None

    @property
    def x(self) -> float:
        return sum(self.xs) / len(self.xs)

    @property
    def y(self) -> float:
        return sum(self.ys) / len(self.ys)

    @property
    def spread_m(self) -> float:
        """RMS distance of the associated positions from their own centroid.

        The discriminating feature: a pillar's returns pile up, an occlusion
        edge's slide with the viewpoint.
        """
        cx, cy = self.x, self.y
        return math.sqrt(sum((x - cx) ** 2 + (y - cy) ** 2 for x, y in zip(self.xs, self.ys, strict=True)) / len(self.xs))

    @property
    def chord_cv(self) -> float:
        """Coefficient of variation of the observed chord; 0 for a true cylinder."""
        mean = sum(self.chords) / len(self.chords)
        if mean <= 0:
            return math.inf
        var = sum((c - mean) ** 2 for c in self.chords) / len(self.chords)
        return math.sqrt(var) / mean


def find_clusters(scan: LidarScan, *, min_m: float, max_m: float, depth_m: float, isolation_m: float) -> list[Cluster]:
    """Every free-standing, pillar-width return in one scan.

    A cluster is a contiguous angular run whose consecutive ranges differ by less
    than `depth_m`, bounded on BOTH sides by a step of at least `isolation_m` --
    the shape `SIGN_LIDAR_ALIGN_DEPTH_M`/`_MAX_WIDTH_M` already encode. The
    two-sided isolation is what makes it "free-standing": a flat wall segment is
    contiguous but never steps away on both ends.
    """
    pts = [
        (r, a)
        for r, a in zip(scan.ranges_m, scan.angles_rad, strict=True)
        if math.isfinite(r) and min_m <= r <= max_m
    ]
    if len(pts) < _MIN_SCAN_POINTS:
        return []

    clusters: list[Cluster] = []
    run: list[tuple[float, float]] = [pts[0]]
    for i in range(1, len(pts)):
        r, a = pts[i]
        prev_r, _ = pts[i - 1]
        if abs(r - prev_r) < depth_m:
            run.append((r, a))
            continue
        # The run ended. It is free-standing only if it also STARTED with a step.
        gap_before = i - len(run) - 1
        started_clear = gap_before < 0 or abs(run[0][0] - pts[gap_before][0]) >= isolation_m
        if started_clear and abs(r - prev_r) >= isolation_m:
            clusters.append(_as_cluster(run))
        run = [(r, a)]
    return clusters


def _as_cluster(run: Sequence[tuple[float, float]]) -> Cluster:
    """Collapse a contiguous run of (range, angle) samples into its centre and chord."""
    ranges = [r for r, _ in run]
    angles = [a for _, a in run]
    closest = min(ranges)
    return Cluster(
        range_m=closest,
        bearing_rad=(angles[0] + angles[-1]) / 2.0,
        chord_m=closest * abs(angles[-1] - angles[0]),
    )


def corridor_walls(
    scan: LidarScan,
    window_rad: float,
    max_wall_m: float,
    expected_width_m: float | None = None,
    width_tol_m: float = 0.15,
) -> tuple[float, float] | None:
    """Perpendicular distance to the LEFT and RIGHT walls, or None if either is missing.

    A WINDOWED MEDIAN either side of +/-90 deg, never a single ray: the -90 deg
    ray alone drops out on 66% of hardware ticks, and a dropout reads as a 12 m
    "open" side -- the same failure that ratcheted a bay exit into a wall.
    Requiring BOTH walls is what makes the result a corridor width rather than
    one unverified number.

    Returns are measured perpendicular to the robot's heading, so this is only
    a corridor width while the robot is roughly aligned with the corridor; a
    mid-corner tick returns a widened, meaningless pair and the caller drops it
    via `max_wall_m`.
    """
    left: list[float] = []
    right: list[float] = []
    for r, a in zip(scan.ranges_m, scan.angles_rad, strict=True):
        # Rear self-returns sit INSIDE the chassis at ~0.08 m and would read as
        # an impossibly narrow corridor.
        if not math.isfinite(r) or r < _MIN_WALL_RANGE_M or r > max_wall_m:
            continue
        norm = math.atan2(math.sin(a), math.cos(a))
        if abs(norm - math.pi / 2) <= window_rad:
            left.append(r * math.sin(norm))
        elif abs(norm + math.pi / 2) <= window_rad:
            right.append(-r * math.sin(norm))
    if not left or not right:
        return None
    l_m, r_m = percentile(left, 0.5), percentile(right, 0.5)
    # PLAUSIBILITY GATE. The corridor is a KNOWN width, so a pair that does not
    # add up to it is not a corridor -- it is a corner, an opening into the next
    # section, or a ray that found the far wall. Ungated, the measured width
    # drifts to p50 1.24 m against a true 1.00 m, and every lateral offset
    # derived from it is stretched by that error.
    if expected_width_m is not None and abs((l_m + r_m) - expected_width_m) > width_tol_m:
        return None
    return l_m, r_m


def to_world(pose: tuple[float, float, float], range_m: float, bearing_rad: float) -> tuple[float, float]:
    """Project a robot-frame (range, bearing) observation into world coordinates."""
    x, y, yaw = pose
    return x + range_m * math.cos(yaw + bearing_rad), y + range_m * math.sin(yaw + bearing_rad)


def associate(
    observations: Sequence[tuple[float, float, float, float, float, float | None, float | None]], radius_m: float
) -> list[Track]:
    """Group world-frame observations `(t, x, y, chord, range, wall_dist, width)` into tracks.

    Greedy nearest-centroid association against the running mean, the same rule
    the sign map itself uses. Ordered by time, so `first_range_m` is genuinely
    the range at which the candidate first became available.
    """
    tracks: list[Track] = []
    for t, x, y, chord, rng, wall, width in observations:
        match = min(
            (tr for tr in tracks if math.hypot(tr.x - x, tr.y - y) <= radius_m),
            key=lambda tr: math.hypot(tr.x - x, tr.y - y),
            default=None,
        )
        if match is None:
            match = Track(first_range_m=rng, first_t_s=t)
            tracks.append(match)
        match.xs.append(x)
        match.ys.append(y)
        match.chords.append(chord)
        if wall is not None:
            match.wall_dists.append(wall)
        if width is not None:
            match.widths.append(width)
    return tracks


def width_of(walls: tuple[float, float] | None) -> float | None:
    """Measured corridor width, for auditing the wall estimate against the known 1.0 m."""
    return None if walls is None else walls[0] + walls[1]


def wall_distance(c: Cluster, walls: tuple[float, float] | None) -> float | None:
    """How far this cluster sits from the NEARER corridor wall, in metres.

    The cluster's lateral offset is `r * sin(bearing)`, positive to the left.
    Adding the measured left-wall distance puts it on an axis running from the
    right wall (0) to the left wall (the corridor width), so the distance to
    the nearer wall is the smaller of the two -- which is 0.4 m for a sign in
    either lattice slot, and ~0 for a wall corner.
    """
    if walls is None:
        return None
    left_m, right_m = walls
    from_right = right_m + c.range_m * math.sin(c.bearing_rad)
    from_left = left_m - c.range_m * math.sin(c.bearing_rad)
    if from_right < 0 or from_left < 0:
        return None  # Outside the measured corridor: another section, or a bad wall pair.
    return min(from_right, from_left)


