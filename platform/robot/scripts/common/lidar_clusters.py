"""Association and per-track statistics for the LIDAR sign proposer's diagnostics.

The DETECTOR itself is not here -- it is
:mod:`src.navigation.planning.lidar_proposer`, the module the robot actually
runs. Both diagnostics import it rather than reimplementing it, so a measurement
is a statement about the shipped detector and a sim-vs-hardware difference is a
statement about the sensors.

What lives here is only what a diagnostic needs and the robot does not: grouping
observations into world-frame tracks after the fact, and the per-track summary
statistics used to score them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from scripts.common.stats import percentile
from src.navigation.planning.lidar_proposer import (
    Cluster,
    ProposerParams,
    corridor_walls,
    find_clusters,
    wall_distance,
)

if TYPE_CHECKING:
    import argparse
    from collections.abc import Sequence

__all__ = [
    "Cluster",
    "ProposerParams",
    "Track",
    "associate",
    "corridor_walls",
    "find_clusters",
    "proposer_params_from_args",
    "to_world",
    "wall_distance",
    "width_of",
]


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


def proposer_params_from_args(args: argparse.Namespace) -> ProposerParams:
    """The shipped detector's parameters, driven by this diagnostic's flags.

    Built from ``args`` rather than taken as defaults so a sweep can move one
    knob without editing the robot -- but it is the ROBOT'S dataclass, so a field
    added there cannot be silently missed here.
    """
    return ProposerParams(
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        depth_m=args.depth,
        isolation_m=args.isolation,
        min_chord_m=args.min_chord,
        max_chord_m=args.max_chord,
        wall_window_deg=args.wall_window_deg,
        max_wall_range_m=args.max_wall_m,
        corridor_width_m=args.corridor_width_m,
        width_tol_m=args.width_tol_m,
        lattice_offset_m=args.lattice_offset_m,
        lattice_tol_m=args.lattice_tol_m,
    )


