"""Pillar-shaped LIDAR returns, proposed as sign POSITIONS for the camera to colour.

The camera stops resolving signs past ~1.1 m while the LIDAR picks up
pillar-shaped clusters at a median 1.31 m, so the LIDAR can say WHERE an object
is well before the camera can say WHAT it is. It cannot replace the camera: the
WRO pass-side rule is colour-keyed, and a colourless proposal has no side (see
``sign_router.routing.pass_side_lateral_axis``). What it buys is POSITION EARLY,
COLOUR LATE -- when the colour finally arrives, the geometry is already settled
instead of being established from scratch inside the last 0.3 m.

Measured against the simulator's known layout (`diag_sim_lidar_proposer.py`,
16 scenarios / 84 signs / 563 tracks):

    raw detector                precision 46%, recall 100% (84/84 signs)
    + the placement lattice     precision 84%, recall  85%

**The lattice is what makes a proposal trustworthy.** Signs stand on a 6-point
grid per section -- ``GRID_WIDTH_OUTER``/``_INNER`` 0.4/0.6 m across a 1.0 m
corridor, ``GRID_DEPTH_NEAR/MIDDLE/FAR`` 1.0/1.5/2.0 m -- so a sign is always
0.4 m from its nearer lateral border while a wall corner is at ~0. Two
shape-based filters (world-position spread, chord stability) were tried FIRST
and are REFUTED: each discarded ~29% of tracks to move precision by 2 points,
because a corner viewed across a short arc of travel holds just as still as a
pillar does.

TRAPS, both of which nearly cost the measurement:

* The medians do NOT separate -- true signs sit at p50 0.31 m from the nearer
  wall and false tracks at 0.24 m. The filter works on the BAND, so never judge
  it by comparing medians.
* The wall estimate MUST be gated on the known corridor width. Ungated it drifts
  to p50 1.24 m against a true 1.00 m (corners and section openings), stretching
  every lateral offset derived from it; gated it reads 1.02 m in sim and 0.99 m
  on hardware.

Hardware transfer is measured, not assumed: cluster YIELD matches (3.10 per scan
against the sim's 2.95, 204 tracks against 198), but the sim spots a candidate
~0.44 m earlier and hardware's lattice-consistent population is a third thinner
(23% of tracks in the sign band against 35%). Treat 84% as an upper bound.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import CorridorDimensions, TrafficSignSpecs

if TYPE_CHECKING:
    from collections.abc import Sequence

    from src.navigation.ports import LidarScan

_MIN_SCAN_POINTS = 3
"""Below this a scan cannot hold a cluster bounded by a step on both sides."""

_MIN_WALL_RANGE_M = 0.10
"""Returns nearer than this are rear self-returns off the chassis -- the mount
scalar sits at ~0.08 m, INSIDE the body -- and would read as a narrow corridor."""


@dataclass(frozen=True, slots=True)
class ProposerParams:
    """Geometry of what counts as a pillar, and of where a pillar may stand.

    Defaults are the values the sim measurement was taken at; the cluster shape
    ones match the ``SIGN_LIDAR_ALIGN_*`` tuning fields that already encode this
    same shape for a different purpose.
    """

    min_range_m: float = 0.30
    max_range_m: float = 2.50
    depth_m: float = 0.08
    """Range step between adjacent samples that BREAKS a cluster."""

    isolation_m: float = 0.20
    """Step that must bound a cluster on BOTH sides for it to be free-standing."""

    min_chord_m: float = 0.02
    max_chord_m: float = 0.18
    """A sign is 0.05 m across (``TrafficSignSpecs.WIDTH``); the band allows for
    range noise and for grazing incidence widening the apparent chord."""

    wall_window_deg: float = 20.0
    """Half-window either side of +/-90 deg for the wall medians. A WINDOW, never
    a single ray: the -90 deg ray alone has no return on 12-19% of hardware
    ticks, which the window brings to 0.0-1.7%."""

    max_wall_range_m: float = 1.50
    corridor_width_m: float = CorridorDimensions.OBSTACLES_WIDTH
    width_tol_m: float = 0.15
    """How far the measured corridor width may differ from the known one before
    the pair is rejected as not-a-corridor. See the module docstring."""

    lattice_offset_m: float = CorridorDimensions.DIVISION_OUTER
    """Distance from the NEARER lateral border a sign stands at -- 0.4 m, which
    is ``GRID_WIDTH_OUTER`` and equally ``GRID_WIDTH_INNER`` mirrored."""

    lattice_tol_m: float = 0.12


@dataclass(frozen=True, slots=True)
class Cluster:
    """One pillar-shaped return in a single scan, in the robot frame."""

    range_m: float
    bearing_rad: float
    chord_m: float


def find_clusters(scan: LidarScan, params: ProposerParams) -> list[Cluster]:
    """Every free-standing, pillar-width return in one scan.

    A cluster is a contiguous angular run whose consecutive ranges differ by
    less than ``depth_m``, bounded on BOTH sides by a step of at least
    ``isolation_m``. The two-sided isolation is what makes it "free-standing":
    a flat wall segment is contiguous but never steps away at both ends.
    """
    pts = [
        (r, a)
        for r, a in zip(scan.ranges_m, scan.angles_rad, strict=True)
        if math.isfinite(r) and params.min_range_m <= r <= params.max_range_m
    ]
    if len(pts) < _MIN_SCAN_POINTS:
        return []

    clusters: list[Cluster] = []
    run: list[tuple[float, float]] = [pts[0]]
    for i in range(1, len(pts)):
        r, a = pts[i]
        prev_r, _ = pts[i - 1]
        if abs(r - prev_r) < params.depth_m:
            run.append((r, a))
            continue
        # The run ended. It is free-standing only if it also STARTED with a step.
        gap_before = i - len(run) - 1
        started_clear = gap_before < 0 or abs(run[0][0] - pts[gap_before][0]) >= params.isolation_m
        if started_clear and abs(r - prev_r) >= params.isolation_m:
            cluster = _as_cluster(run)
            if params.min_chord_m <= cluster.chord_m <= params.max_chord_m:
                clusters.append(cluster)
        run = [(r, a)]
    return clusters


def _as_cluster(run: Sequence[tuple[float, float]]) -> Cluster:
    """Collapse a contiguous run of ``(range, angle)`` samples into centre and chord."""
    ranges = [r for r, _ in run]
    angles = [a for _, a in run]
    closest = min(ranges)
    return Cluster(
        range_m=closest,
        bearing_rad=(angles[0] + angles[-1]) / 2.0,
        chord_m=closest * abs(angles[-1] - angles[0]),
    )


def corridor_walls(scan: LidarScan, params: ProposerParams) -> tuple[float, float] | None:
    """Perpendicular distance to the LEFT and RIGHT walls, or None if implausible.

    A windowed MEDIAN either side of +/-90 deg, gated on the known corridor
    width: a pair that does not add up to a corridor is a corner, a section
    opening, or a ray that found the far wall. Requiring BOTH walls is what
    makes the result a width rather than one unverified number.
    """
    left: list[float] = []
    right: list[float] = []
    window = math.radians(params.wall_window_deg)
    for r, a in zip(scan.ranges_m, scan.angles_rad, strict=True):
        if not math.isfinite(r) or r < _MIN_WALL_RANGE_M or r > params.max_wall_range_m:
            continue
        norm = math.atan2(math.sin(a), math.cos(a))
        if abs(norm - math.pi / 2) <= window:
            left.append(r * math.sin(norm))
        elif abs(norm + math.pi / 2) <= window:
            right.append(-r * math.sin(norm))
    if not left or not right:
        return None
    l_m, r_m = _median(left), _median(right)
    if abs((l_m + r_m) - params.corridor_width_m) > params.width_tol_m:
        return None
    return l_m, r_m


def wall_distance(cluster: Cluster, walls: tuple[float, float] | None) -> float | None:
    """How far ``cluster`` sits from the NEARER corridor wall, or None if unknown.

    The cluster's lateral offset is ``r * sin(bearing)``, positive to the left.
    Adding the measured left-wall distance puts it on an axis running from the
    right wall (0) to the left wall (the width), so the nearer wall is the
    smaller of the two -- 0.4 m for a sign in either lattice slot, ~0 for a
    wall corner.
    """
    if walls is None:
        return None
    left_m, right_m = walls
    lateral = cluster.range_m * math.sin(cluster.bearing_rad)
    from_right, from_left = right_m + lateral, left_m - lateral
    if from_right < 0 or from_left < 0:
        return None  # Outside the measured corridor: another section, or a bad pair.
    return min(from_right, from_left)


def on_lattice(cluster: Cluster, walls: tuple[float, float] | None, params: ProposerParams) -> bool:
    """Whether ``cluster`` stands where a sign is allowed to stand.

    False for an unmeasurable corridor: without walls there is no lateral
    offset, and admitting the cluster anyway would silently disable the one
    filter that works.
    """
    d = wall_distance(cluster, walls)
    return d is not None and abs(d - params.lattice_offset_m) <= params.lattice_tol_m


def propose(
    scan: LidarScan,
    robot_pose: tuple[float, float, float],
    params: ProposerParams | None = None,
) -> list[tuple[float, float]]:
    """World positions of every lattice-consistent pillar candidate in one scan.

    Returns positions only -- no colour, no confidence. A caller folds these
    into the sign map as position evidence and waits for the camera to say what
    they are.
    """
    params = params or ProposerParams()
    walls = corridor_walls(scan, params)
    x, y, yaw = robot_pose
    out: list[tuple[float, float]] = []
    for cluster in find_clusters(scan, params):
        if not on_lattice(cluster, walls, params):
            continue
        bearing = yaw + cluster.bearing_rad
        # The chord's near face is what the beam struck; the sign's CENTRE is
        # half a depth further out. Ignoring it biases every proposal 25 mm
        # toward the robot, which is a third of the lattice tolerance.
        centre_range = cluster.range_m + TrafficSignSpecs.DEPTH / 2.0
        out.append((x + centre_range * math.cos(bearing), y + centre_range * math.sin(bearing)))
    return out


def _median(values: Sequence[float]) -> float:
    """Median of a non-empty sample, without pulling numpy into the control path."""
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0
