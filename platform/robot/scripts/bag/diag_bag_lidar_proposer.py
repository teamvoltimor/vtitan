"""Can the LIDAR PROPOSE sign positions for the camera to CONFIRM?

The camera stops resolving signs past ~1.1 m (`diag_vision_range_ceiling.py`),
while pillar-shaped LIDAR clusters first appear at a median 1.1-1.3 m. That gap
is only useful if a proposal can be trusted, and the raw cluster detector emits
30-196 persistent tracks against ~8 real signs -- so RANGE was never the open
question, PRECISION is.

This script measures precision directly instead of counting clusters. It scores
every LIDAR track on two features a wall corner should not be able to fake:

* **world-position spread** -- a pillar is a physical object and holds still in
  world coordinates; a corner is an OCCLUSION EDGE that slides as the robot's
  viewpoint changes, so its associated positions should smear.
* **chord stability** -- a ~0.05 m cylinder subtends the same chord from every
  angle and range; a corner's apparent chord should depend on approach angle.

**BOTH ARE REFUTED, and the CONTROL line is what says so.** Over six 09-07 runs
the filter discards 29% of tracks and moves the confirmation rate 47% -> 49%.
It is INERT: geometric stability does not separate pillars from corners, because
a corner viewed across a short arc of travel holds still too. Tightening these
thresholds measures nothing, so the control is printed unconditionally -- an
inert filter that looks effective is how this project has lost sweeps before.

Neither feature needs a track map, so nothing here assumes the corridor geometry
the robot is still inferring at the time the proposal would be made.

The camera side is the confirmation anchor, decoded with the SAME pinhole model
`sign_discovery.detection_to_observation` uses, including the RANGE_SCALE and
the `captured_at`/VISION_LATENCY_S pose pairing -- pairing a detection with the
pose at RECEIPT is what put the bearing residual at 20.2 deg. Camera tracks are
an ANCHOR, NOT GROUND TRUTH: roughly half of hardware RED detections are
wall-shaped, so a "confirmed" LIDAR track can still be a barrier both sensors
agree on. Read the confirmation rate as an upper bound on precision.

Measured over the six 09-07 runs (525 persistent tracks, 244 camera tracks):

    recall                   221/244 = 91%   camera objects already proposed
    lead in first-see range  p50 0.59 m, p90 1.30 m   MATCHED, per object
    precision (upper bound)  49%

The lead is a MATCHED per-object comparison -- the same object seen by both
sensors -- not the difference of two unpaired medians, which is what the
earlier "0.4-0.6 m earlier" estimate was and which cannot support a claim
about any individual sign.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        pixi run -e dev python scripts/bag/diag_bag_lidar_proposer.py \
        ../../data/live/runs/run_2026090*
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs, TrafficSignSpecs
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import (
    Topics,
    create_bags_parser,
    decode_nav_debug,
    decode_scan,
    elapsed_seconds,
    open_reader,
)
from scripts.common.stats import nearest_by_time, percentile

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.models import NavigatorDebugSnapshot

    from src.navigation.ports import LidarScan

VISION_DETECTIONS = "/vision/detections"

_MIN_SCAN_POINTS = 3
"""Below this a scan cannot contain a cluster with a step on both sides."""

_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)
"""Duplicated from `sign_discovery` rather than imported, so this measurement of
the camera's geometry cannot be silently changed by a tuning edit to that module."""


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
    first_range_m: float = 0.0
    first_t_s: float = 0.0

    @property
    def hits(self) -> int:
        return len(self.xs)

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


def pose_series(
    rows: Sequence[tuple[float, NavigatorDebugSnapshot]],
) -> tuple[list[tuple[float, tuple[float, float, float]]], list[float]]:
    """The `(t, (x, y, yaw))` series and its time index, for `nearest_by_time`.

    Snapshots published before the navigator has a fix carry a None pose and are
    dropped here rather than at each lookup, so a caller can never pair an
    observation with a half-populated pose.
    """
    series = [
        (t, (float(s.pose_x), float(s.pose_y), float(s.pose_yaw)))
        for t, s in rows
        if isinstance(s.pose_x, (int, float))
        and isinstance(s.pose_y, (int, float))
        and isinstance(s.pose_yaw, (int, float))
    ]
    return series, [t for t, _ in series]


def to_world(pose: tuple[float, float, float], range_m: float, bearing_rad: float) -> tuple[float, float]:
    """Project a robot-frame (range, bearing) observation into world coordinates."""
    x, y, yaw = pose
    return x + range_m * math.cos(yaw + bearing_rad), y + range_m * math.sin(yaw + bearing_rad)


def associate(
    observations: Sequence[tuple[float, float, float, float, float]], radius_m: float
) -> list[Track]:
    """Group world-frame observations `(t, x, y, chord, range)` into tracks.

    Greedy nearest-centroid association against the running mean, the same rule
    the sign map itself uses. Ordered by time, so `first_range_m` is genuinely
    the range at which the candidate first became available.
    """
    tracks: list[Track] = []
    for t, x, y, chord, rng in observations:
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
    return tracks


def read_run(
    run_dir: Path, yaw_offset_rad: float, latency_s: float, range_scale: float
) -> tuple[list[tuple[float, LidarScan]], list[tuple[float, NavigatorDebugSnapshot]], list[tuple[float, float, float]]]:
    """One replay pass: scans, poses, and camera detections as `(t, range, bearing)`.

    The detection time is the CAPTURE time -- `captured_at` when the payload
    carries it, else receipt minus `latency_s` -- because the world position
    depends on the pose the camera actually saw from.
    """
    reader = open_reader(run_dir)
    scans: list[tuple[float, LidarScan]] = []
    rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    dets: list[tuple[float, float, float]] = []
    t0: int | None = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        rel = elapsed_seconds(t, t0)
        if topic == Topics.SCAN:
            scans.append((rel, decode_scan(deserialize_message(data, LaserScan), yaw_offset_rad)))
        elif topic == Topics.NAV_DEBUG:
            rows.append((rel, decode_nav_debug(data)))
        elif topic == VISION_DETECTIONS:
            payload = json.loads(deserialize_message(data, String).data) or []
            for d in payload:
                height = d.get("height", 0.0)
                cx = d.get("x")
                if height <= 0 or cx is None:
                    continue
                # Bag timestamps are epoch NANOSECONDS, so `captured_at` (epoch
                # seconds) converts to bag-relative exactly. Deriving the epoch
                # offset from a detection instead would fold the very latency
                # this is correcting back into the answer, with the wrong sign.
                captured = d.get("captured_at")
                when = float(captured) - t0 / 1e9 if captured is not None else rel - latency_s
                rng = _CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT / height * range_scale
                bearing = (0.5 - float(cx) / RobotSpecs.CAMERA_WIDTH) * RobotSpecs.CAMERA_HFOV
                dets.append((when, rng, bearing))
    return scans, rows, dets


def _fmt(values: Sequence[float], unit: str = "m") -> str:
    """`p50 / p90` for a sample, or a placeholder when it is empty."""
    if not values:
        return "    --     "
    return f"{percentile(values, 0.5):.2f} / {percentile(values, 0.9):.2f} {unit}"


def build_tracks(
    scans: Sequence[tuple[float, LidarScan]],
    rows: Sequence[tuple[float, NavigatorDebugSnapshot]],
    args,  # noqa: ANN001
) -> tuple[list[Track], int]:
    """World-associated LIDAR tracks, plus the raw cluster count they came from."""
    series, times = pose_series(rows)
    observations: list[tuple[float, float, float, float, float]] = []
    raw = 0
    for t, scan in scans:
        clusters = find_clusters(
            scan,
            min_m=args.min_range,
            max_m=args.max_range,
            depth_m=args.depth,
            isolation_m=args.isolation,
        )
        raw += len(clusters)
        pose = nearest_by_time(series, times, t, tolerance=args.pose_tolerance)
        if pose is None:
            continue
        for c in clusters:
            if not args.min_chord <= c.chord_m <= args.max_chord:
                continue
            x, y = to_world(pose, c.range_m, c.bearing_rad)
            observations.append((t, x, y, c.chord_m, c.range_m))
    return associate(observations, args.assoc_radius), raw


def camera_tracks(
    dets: Sequence[tuple[float, float, float]],
    rows: Sequence[tuple[float, NavigatorDebugSnapshot]],
    args,  # noqa: ANN001
) -> list[Track]:
    """World-associated CAMERA tracks, the confirmation anchor."""
    series, times = pose_series(rows)
    observations: list[tuple[float, float, float, float, float]] = []
    for t, rng, bearing in sorted(dets):
        pose = nearest_by_time(series, times, t, tolerance=args.pose_tolerance)
        if pose is None:
            continue
        x, y = to_world(pose, rng, bearing)
        observations.append((t, x, y, 0.0, rng))
    return associate(observations, args.assoc_radius)


def main() -> None:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min-range", type=float, default=0.30, help="Nearest cluster range considered.")
    parser.add_argument("--max-range", type=float, default=2.50, help="Farthest cluster range considered.")
    parser.add_argument("--depth", type=float, default=0.08, help="Range step that breaks a cluster (SIGN_LIDAR_ALIGN_DEPTH_M).")
    parser.add_argument("--isolation", type=float, default=0.20, help="Step that must bound a cluster on BOTH sides.")
    parser.add_argument("--min-chord", type=float, default=0.02)
    parser.add_argument("--max-chord", type=float, default=0.18, help="Widest pillar chord accepted.")
    parser.add_argument("--assoc-radius", type=float, default=0.15, help="World association radius.")
    parser.add_argument("--pose-tolerance", type=float, default=0.20, help="Max seconds between an observation and its pose.")
    parser.add_argument("--min-hits", type=int, default=5, help="Hits before a track counts as persistent.")
    parser.add_argument(
        "--max-chord-cv",
        type=float,
        default=0.5,
        help="Chord coefficient-of-variation a track may have and still count as stable. "
        "Exposed so the REFUTED stability filter stays re-testable; check the CONTROL line before believing any value.",
    )
    parser.add_argument("--confirm-radius", type=float, default=0.30, help="Distance within which a camera track confirms a LIDAR track.")
    parser.add_argument("--latency", type=float, default=0.85, help="Fallback capture latency when captured_at is absent (VISION_LATENCY_S).")
    parser.add_argument("--range-scale", type=float, default=1.95, help="Camera RANGE_SCALE.")
    args = parser.parse_args()

    yaw_offset = RobotSpecs.lidar_yaw_offset_rad()
    print(f"{'run':<22} {'scans':>6} {'raw':>6} {'trk':>5} {'persist':>7} {'stable':>6} {'conf':>5} {'cam':>4}  {'lidar first-see':>15}  {'camera first-see':>16}  lead")
    totals = {"persist": 0, "stable": 0, "confirmed": 0, "confirmed_unfiltered": 0, "cam": 0, "recalled": 0}
    all_leads: list[float] = []

    for run_dir in args.bag_dirs:
        scans, rows, dets = read_run(run_dir, yaw_offset, args.latency, args.range_scale)
        if not scans or not rows:
            print(f"{run_dir.name:<22} {len(scans):>6}  (no scans or no posed /nav_debug rows)")
            continue

        tracks, raw = build_tracks(scans, rows, args)
        persistent = [t for t in tracks if t.hits >= args.min_hits]
        # The precision filter: a physical object holds still and keeps its width.
        stable = [
            t for t in persistent if t.spread_m <= args.assoc_radius / 2 and t.chord_cv <= args.max_chord_cv
        ]
        cams = [t for t in camera_tracks(dets, rows, args) if t.hits >= args.min_hits]

        def is_confirmed(t: Track, cams: Sequence[Track] = cams) -> bool:
            return any(math.hypot(t.x - c.x, t.y - c.y) <= args.confirm_radius for c in cams)

        confirmed = [t for t in stable if is_confirmed(t)]
        # The control. If the filter is discarding tracks UNIFORMLY rather than
        # selectively, this rate matches the filtered one and the filter is
        # inert -- a stricter threshold would then be measuring nothing.
        confirmed_unfiltered = [t for t in persistent if is_confirmed(t)]
        # Recall runs the other way: a camera track is only WORTH proposing early
        # if a LIDAR track was already sitting on it.
        leads: list[float] = []
        recalled = 0
        for c in cams:
            near = [t for t in stable if math.hypot(t.x - c.x, t.y - c.y) <= args.confirm_radius]
            if not near:
                continue
            recalled += 1
            leads.append(max(t.first_range_m for t in near) - c.first_range_m)

        totals["persist"] += len(persistent)
        totals["stable"] += len(stable)
        totals["confirmed"] += len(confirmed)
        totals["confirmed_unfiltered"] += len(confirmed_unfiltered)
        totals["cam"] += len(cams)
        totals["recalled"] += recalled
        all_leads.extend(leads)

        print(
            f"{run_dir.name:<22} {len(scans):>6} {raw:>6} {len(tracks):>5} {len(persistent):>7} "
            f"{len(stable):>6} {len(confirmed):>5} {len(cams):>4}  "
            f"{_fmt([t.first_range_m for t in stable]):>15}  {_fmt([c.first_range_m for c in cams]):>16}  "
            f"{_fmt(leads)}"
        )

    _print_summary(totals, all_leads, args.min_hits)


def _print_summary(totals: dict[str, int], all_leads: Sequence[float], min_hits: int) -> None:
    """The corpus-wide verdict, including the control that judges the filter."""
    print()
    print(f"persistent tracks (hits>={min_hits}):  {totals['persist']}")
    print(f"  after the stability filter:      {totals['stable']}"
          f"   ({_pct(totals['stable'], totals['persist'])} kept)")
    print(f"  of those, camera-confirmed:      {totals['confirmed']}"
          f"   ({_pct(totals['confirmed'], totals['stable'])} -- an UPPER BOUND on precision)")
    print(f"  CONTROL, confirmed WITHOUT it:   {totals['confirmed_unfiltered']}"
          f"   ({_pct(totals['confirmed_unfiltered'], totals['persist'])} -- if this matches, the filter is inert)")
    print(f"camera tracks with a LIDAR proposal: {totals['recalled']}/{totals['cam']}"
          f"   ({_pct(totals['recalled'], totals['cam'])} recall)")
    print(f"lead in first-detection range:       {_fmt(all_leads)}")


def _pct(n: int, d: int) -> str:
    return f"{100.0 * n / d:.0f}%" if d else "n/a"


if __name__ == "__main__":
    main()
