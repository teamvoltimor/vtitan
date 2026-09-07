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

**THE LATTICE FILTER IS UNDECIDED, NOT REFUTED, AND THE INSTRUMENT CHECK IS WHY.**
Signs stand on a 6-point lattice (0.4/0.6 m lateral, 1.0/1.5/2.0 m depth), so a
sign is 0.4 m from its nearer lateral border and a corner is at ~0. Filtering on
that keeps 33% of tracks and moves precision only 47% -> 52%, at a recall cost of
91% -> 72%. But the check below says that number cannot be read as a verdict on
the prior:

    measured corridor width      p50 1.06 m   against a known 1.00 m -- SOUND
    wall dist, camera-CONFIRMED  p50 0.27 m   the prior predicts ~0.40 m
    wall dist, UNCONFIRMED       p50 0.25 m   ** THE SAME DISTRIBUTION **

The wall estimator is fine, so the failure is the ANCHOR: if camera confirmation
selected real signs, confirmed tracks would pile up at 0.4 m and unconfirmed ones
would not. They are indistinguishable, which is what "half of hardware RED
detections are wall-shaped" predicts. A 5-point precision move scored against an
anchor that cannot separate signs from walls is noise.

**Deciding the prior needs GROUND TRUTH, not a better filter** -- a staged run
with recorded sign placement, or the simulator, where the layout is known.
Do not tune `--lattice-tol-m` against this anchor; it cannot answer.

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

_MIN_WALL_RANGE_M = 0.10
"""Returns nearer than this are rear self-returns off the chassis (the mount
scalar sits at ~0.08 m, INSIDE the body) and would read as a narrow corridor."""

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


def corridor_walls(scan: LidarScan, window_rad: float, max_wall_m: float) -> tuple[float, float] | None:
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
    return percentile(left, 0.5), percentile(right, 0.5)


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
        walls = corridor_walls(scan, math.radians(args.wall_window_deg), args.max_wall_m)
        for c in clusters:
            if not args.min_chord <= c.chord_m <= args.max_chord:
                continue
            x, y = to_world(pose, c.range_m, c.bearing_rad)
            observations.append((t, x, y, c.chord_m, c.range_m, _wall_distance(c, walls), _width(walls)))
    return associate(observations, args.assoc_radius), raw


def _width(walls: tuple[float, float] | None) -> float | None:
    """Measured corridor width, for auditing the wall estimate against the known 1.0 m."""
    return None if walls is None else walls[0] + walls[1]


def _wall_distance(c: Cluster, walls: tuple[float, float] | None) -> float | None:
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
        observations.append((t, x, y, 0.0, rng, None, None))
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
    parser.add_argument("--wall-window-deg", type=float, default=20.0, help="Half-window either side of +/-90 deg for the wall medians.")
    parser.add_argument("--max-wall-m", type=float, default=1.50, help="Ranges beyond this are not a corridor wall.")
    parser.add_argument(
        "--lattice-offset-m",
        type=float,
        default=0.40,
        help="Distance from the nearer lateral border a real sign stands at (GRID_WIDTH_OUTER).",
    )
    parser.add_argument("--lattice-tol-m", type=float, default=0.12, help="Tolerance around --lattice-offset-m.")
    args = parser.parse_args()

    yaw_offset = RobotSpecs.lidar_yaw_offset_rad()
    print(f"{'run':<22} {'scans':>6} {'raw':>6} {'trk':>5} {'persist':>7} {'stable':>6} {'conf':>5} {'cam':>4}  {'lidar first-see':>15}  {'camera first-see':>16}  lead")
    totals = {
        "persist": 0,
        "stable": 0,
        "confirmed": 0,
        "confirmed_unfiltered": 0,
        "cam": 0,
        "recalled": 0,
        "lattice": 0,
        "lattice_confirmed": 0,
        "lattice_recalled": 0,
        "wall_measurable": 0,
    }
    all_leads: list[float] = []
    wall_confirmed: list[float] = []
    wall_unconfirmed: list[float] = []
    corridor_widths: list[float] = []

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

        # The LATTICE filter, applied to `persistent` (NOT stacked on the
        # refuted shape filter) so its effect is attributable to it alone.
        def on_lattice(t: Track) -> bool:
            d = t.wall_dist_m
            return d is not None and abs(d - args.lattice_offset_m) <= args.lattice_tol_m

        lattice = [t for t in persistent if on_lattice(t)]
        lattice_confirmed = [t for t in lattice if is_confirmed(t)]
        # How many camera objects survive the same test -- the recall cost of
        # the filter, which a precision number alone would hide.
        lattice_recalled = sum(
            1
            for c in cams
            if any(math.hypot(t.x - c.x, t.y - c.y) <= args.confirm_radius for t in lattice)
        )
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
        totals["lattice"] += len(lattice)
        totals["lattice_confirmed"] += len(lattice_confirmed)
        totals["lattice_recalled"] += lattice_recalled
        totals["wall_measurable"] += sum(1 for t in persistent if t.wall_dist_m is not None)
        # Instrument check. If the prior is right, CONFIRMED tracks pile up at
        # the lattice offset and unconfirmed ones do not. If both are spread,
        # the wall estimate is the problem and the filter above is measuring
        # this script rather than the track.
        wall_confirmed.extend(t.wall_dist_m for t in persistent if t.wall_dist_m is not None and is_confirmed(t))
        wall_unconfirmed.extend(
            t.wall_dist_m for t in persistent if t.wall_dist_m is not None and not is_confirmed(t)
        )
        corridor_widths.extend(w for w in (t.width_m for t in persistent) if w is not None)
        totals["cam"] += len(cams)
        totals["recalled"] += recalled
        all_leads.extend(leads)

        print(
            f"{run_dir.name:<22} {len(scans):>6} {raw:>6} {len(tracks):>5} {len(persistent):>7} "
            f"{len(stable):>6} {len(confirmed):>5} {len(cams):>4}  "
            f"{_fmt([t.first_range_m for t in stable]):>15}  {_fmt([c.first_range_m for c in cams]):>16}  "
            f"{_fmt(leads)}"
        )

    _print_summary(totals, all_leads, args)
    print()
    print("INSTRUMENT CHECK -- is the wall estimate trustworthy at all?")
    print(f"  measured corridor width:           {_fmt(corridor_widths)}   (known truth: 1.00 m)")
    print(f"  wall dist, camera-CONFIRMED:       {_fmt(wall_confirmed)}   (prior predicts ~{args.lattice_offset_m:.2f} m)")
    print(f"  wall dist, UNCONFIRMED:            {_fmt(wall_unconfirmed)}")


def _print_summary(totals: dict[str, int], all_leads: Sequence[float], args) -> None:  # noqa: ANN001
    """The corpus-wide verdict, including the control that judges the filter."""
    print()
    print(f"persistent tracks (hits>={args.min_hits}):  {totals['persist']}")
    print(f"  after the stability filter:      {totals['stable']}"
          f"   ({_pct(totals['stable'], totals['persist'])} kept)")
    print(f"  of those, camera-confirmed:      {totals['confirmed']}"
          f"   ({_pct(totals['confirmed'], totals['stable'])} -- an UPPER BOUND on precision)")
    print(f"  CONTROL, confirmed WITHOUT it:   {totals['confirmed_unfiltered']}"
          f"   ({_pct(totals['confirmed_unfiltered'], totals['persist'])} -- if this matches, the filter is inert)")
    print(f"camera tracks with a LIDAR proposal: {totals['recalled']}/{totals['cam']}"
          f"   ({_pct(totals['recalled'], totals['cam'])} recall)")
    print(f"lead in first-detection range:       {_fmt(all_leads)}")
    print()
    print(f"LATTICE filter ({args.lattice_offset_m:.2f} +/- {args.lattice_tol_m:.2f} m from the nearer wall),")
    print("applied to the persistent tracks directly, NOT stacked on the shape filter:")
    print(f"  tracks with a measurable corridor: {totals['wall_measurable']}/{totals['persist']}"
          f"   ({_pct(totals['wall_measurable'], totals['persist'])} -- the rest never saw both walls)")
    print(f"  tracks ON the lattice:             {totals['lattice']}"
          f"   ({_pct(totals['lattice'], totals['persist'])} kept)")
    print(f"  of those, camera-confirmed:        {totals['lattice_confirmed']}"
          f"   ({_pct(totals['lattice_confirmed'], totals['lattice'])} vs the {_pct(totals['confirmed_unfiltered'], totals['persist'])} control)")
    print(f"  camera tracks still proposed:      {totals['lattice_recalled']}/{totals['cam']}"
          f"   ({_pct(totals['lattice_recalled'], totals['cam'])} recall, was {_pct(totals['recalled'], totals['cam'])})")


def _pct(n: int, d: int) -> str:
    return f"{100.0 * n / d:.0f}%" if d else "n/a"


if __name__ == "__main__":
    main()
