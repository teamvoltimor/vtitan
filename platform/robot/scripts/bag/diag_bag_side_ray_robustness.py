"""Test whether the direction gate's single side ray is what starved it.

``infer_direction`` reads the side ranges through ``_nearest_ray``, which
returns ONE ray -- the single beam closest to +/-90 deg. A Slamtec emits no
return off dark or shallow-incidence surfaces and the gateway substitutes max
range, so one unlucky beam turns a legitimate 2.5 m opening into a 12 m
"dropout" that the estimator then rejects outright.

This replays the raw ``/scan`` messages and, for each one, compares the single
ray against a windowed reading (median of the VALID returns within a few degrees
of +/-90 deg). If the windowed reading recovers in-track distances where the
single ray reads max range, the gate was starved by beam-level noise rather than
by an absent signal.

``--near-histogram`` adds a full-scan bearing histogram of sub-0.05 m returns
(folded in from a one-off tmp_scan_min.py): where do the near-zero readings
that get treated as "in contact with a wall" actually come from.

``--centre-offset`` asks where the robot actually drove in each corridor,
measured from ``/scan`` alone, and whether that matches where the controller
was aiming -- the centre-bias question. Reports true corridor width beside
the width belief, so a bias that looks wrong because the belief was wrong is
distinguishable from a genuine sign error.

``--dropout-symmetry`` adds a 15-degree-binned, robot-frame histogram of
no-return (dropout) and sub-0.05 m rays plus a left/right symmetry summary
(folded in from a one-off tmp_dropout.py) -- if one side drops out more than
the other, the direction-inference asymmetry test is being fed garbage. The
original tmp_dropout.py binned bearings with ``round(deg/15)*15``, which does
not wrap at +/-180 deg: a ray at +179 deg and one at -179 deg -- 2 degrees
apart on the actual sensor -- landed in different bins (+180 and -180)
instead of merging into one. Fixed here by wrapping the bin centre back into
(-180, 180] before using it as a key.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_side_ray_robustness.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX [--window-deg 5] [--near-histogram] [--dropout-symmetry]
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from typing import TYPE_CHECKING, NamedTuple

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import Topics, create_bag_parser, open_reader, read_bag
from scripts.common.stats import median, nearest_by_time
from scripts.common.tables import print_table
from src.navigation.utils import wrap_angle
from src.ros2.navigation.ros2_hardware_gateway import _LIDAR_YAW_OFFSET_RAD

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.models import NavigatorDebugSnapshot

    from src.navigation.ports import LidarScan

_MAX_RANGE_M = RobotSpecs.LIDAR_MAX_RANGE - 0.1
_NEAR_THRESHOLD_M = 0.05
"""Investigation threshold for --near-histogram/--dropout-symmetry: how close a return has
to be before it looks like the LIDAR is reading its own mount rather than a wall (see
memory/lidar_min_range_self_detection.md). Distinct from RobotSpecs.LIDAR_MIN_RANGE, which
is the sensor's own spec floor and is used above to bound the windowed-median calculation."""
_BEARING_BIN_DEG = 15.0
_REAR_VALID_FRACTION = 0.30
"""Fraction of a bearing's rays that must be valid returns before --rear-occlusion
calls it readable. Deliberately low: a rear bearing spends much of a round pointed
down an empty corridor, where a no-return is the honest answer rather than evidence
of occlusion. What separates the two is the spread below, not this."""
_REAR_RANGE_SPREAD_M = 0.20
"""Spread between a bearing's min and max valid return before it counts as readable.
An occluded bearing that sees the mount answers the same distance on every scan
however the robot moves; an open one cannot, because the robot drove. This is the
test that valid-fraction alone cannot make."""
_MIN_VOTES = 5
_MAT_CENTRE_X = 1.5
_MAT_CENTRE_Y = 1.5
_ALIGN_TOLERANCE_RAD = math.radians(10.0)
"""Beyond this off-axis angle the +/-90 deg rays stop being a wall measurement.

NOT a corner filter by itself: the chassis reads square to the corridor axis
well before a turn (see ``_MIN_CORNER_DISTANCE_M``), so this alone still lets
corner-arc ticks through."""
_MIN_CORNER_DISTANCE_M = 0.5
"""Reject ticks this close to either end of the corridor along the travel axis.

Measured live on run_20260829_140424 (see memory
narrow_corridor_bias_split_verified_centerline_2026_08_29.md): without this
gate, ``actual``/``planned`` read a spurious ~0.10m inward bias on every
corridor, because ``CORNER_PREVIEW_DISTANCE_M`` (0.80m) plus the narrow
corner arc radius (~0.20-0.30m) let the pure-pursuit lookahead target sit on
curved arc geometry while the chassis itself was still square enough to pass
``_ALIGN_TOLERANCE_RAD``. Excluding ticks within 0.5m of a corner collapsed
that to within +/-0.02m and stayed flat out to a 1.3m margin -- not a
gradual effect, a hard corner-vs-straight split, so 0.5m is a real boundary
rather than an arbitrary buffer."""
_MIN_PLAUSIBLE_WIDTH_M = 0.40
_MAX_PLAUSIBLE_WIDTH_M = 1.30
"""The mat's corridors are 0.60 or 1.00 m. Anything outside this bracket is a
ray that escaped through a gap or caught the inner block end-on, not a corridor."""
_PAIR_TOLERANCE_S = 0.10
"""Max age difference when pairing a scan with a /nav_debug row. The scan runs
at ~10 Hz and the control loop at 20 Hz, so a real pair is always well under this."""


class CorridorOffsetSample(NamedTuple):
    """One accepted ``_offset_sample`` tick: true width, width belief, actual and planned offset."""

    width: float
    belief: float
    actual: float
    planned: float


def _single(ranges: Sequence[float], angles: Sequence[float], target: float) -> float:
    idx = min(range(len(angles)), key=lambda i: abs(wrap_angle(angles[i] - target)))
    return ranges[idx]


def _windowed(ranges: Sequence[float], angles: Sequence[float], target: float, half_width: float) -> float | None:
    """Median of the in-track returns within ``half_width`` of ``target``.

    Returns None when every beam in the window is a dropout -- that is a
    genuinely absent reading, not a recoverable one.
    """
    vals = [
        r
        for r, a in zip(ranges, angles, strict=False)
        if abs(wrap_angle(a - target)) <= half_width and RobotSpecs.LIDAR_MIN_RANGE < r < _MAX_RANGE_M
    ]
    if not vals:
        return None
    return median(vals)


def _paired_snapshot(
    rows: Sequence[tuple[float, NavigatorDebugSnapshot]],
    row_times: Sequence[float],
    t_scan: float,
) -> NavigatorDebugSnapshot | None:
    """The /nav_debug snapshot nearest ``t_scan``, or None if none is close enough."""
    return nearest_by_time(rows, row_times, t_scan, tolerance=_PAIR_TOLERANCE_S)


def _offset_sample(
    scan: LidarScan,
    snap: NavigatorDebugSnapshot,
    half_width: float,
) -> CorridorOffsetSample | str:
    """``(true_width, belief, actual, planned)`` for one scan, or why it was rejected.

    Returning the rejection reason rather than None keeps every discard
    attributable: a run that yields no samples has to say which gate ate them.
    """
    preconditions = (
        (snap.active_maneuver_type is not None, "escape/park maneuver active"),
        (snap.current_corridor is None or snap.direction is None, "corridor or direction not settled"),
        (
            not all(
                isinstance(v, (int, float))
                for v in (snap.pose_x, snap.pose_y, snap.pose_yaw, snap.steer_target_x, snap.steer_target_y)
            ),
            "pose or steer target missing",
        ),
    )
    for failed, reason in preconditions:
        if failed:
            return reason

    # The inward normal, taken from the mat rather than from the pose: the inner
    # block is the mat centre, so the bearing from the robot to it points inward
    # whichever corridor this is. Deriving it from `current_corridor` instead
    # would trust a field a bad pose can itself set wrong.
    to_centre = math.atan2(_MAT_CENTRE_Y - snap.pose_y, _MAT_CENTRE_X - snap.pose_x)
    # Misalignment from the corridor axis, mod 180 deg -- the axis is a line,
    # not an arrow, so travelling it either way counts as square.
    misalign = abs(abs(wrap_angle(to_centre - snap.pose_yaw)) - math.pi / 2)
    if misalign > _ALIGN_TOLERANCE_RAD:
        return "chassis not square to corridor (corner/turn)"

    # Distance to the nearest corridor end along the travel axis: whichever of
    # x/y dominates the heading is the "along corridor" coordinate. A square
    # chassis can still be sitting on curved corner-arc geometry (see
    # _MIN_CORNER_DISTANCE_M), so this is a second, independent corner gate.
    cos_yaw, sin_yaw = math.cos(snap.pose_yaw), math.sin(snap.pose_yaw)
    along = snap.pose_x if abs(cos_yaw) > abs(sin_yaw) else snap.pose_y
    if min(along, TrackDimensions.MAX_COORD - along) < _MIN_CORNER_DISTANCE_M:
        return "within corner-arc geometry (not a true straight)"

    left = _windowed(scan.ranges_m, scan.angles_rad, math.pi / 2, half_width)
    right = _windowed(scan.ranges_m, scan.angles_rad, -math.pi / 2, half_width)
    if left is None or right is None:
        return "a side had no valid return"
    # Rays at +/-90 deg are only perpendicular when square to the wall.
    cos_m = math.cos(misalign)
    left, right = left * cos_m, right * cos_m
    width = left + right
    if not _MIN_PLAUSIBLE_WIDTH_M <= width <= _MAX_PLAUSIBLE_WIDTH_M:
        return "implausible measured width"

    # Which hand points at the inner block: +pi/2 is left in the robot frame.
    left_is_inner = abs(wrap_angle(to_centre - (snap.pose_yaw + math.pi / 2))) < math.pi / 2
    d_inner, d_outer = (left, right) if left_is_inner else (right, left)
    actual = (d_outer - d_inner) / 2.0

    # Target offset relative to the robot, projected onto the inward normal.
    dx, dy = snap.steer_target_x - snap.pose_x, snap.steer_target_y - snap.pose_y
    planned = actual + dx * math.cos(to_centre) + dy * math.sin(to_centre)

    belief = snap.corridor_width_belief_m
    return CorridorOffsetSample(width, belief if belief is not None else math.nan, actual, planned)


def _centre_offset(bag_dir: Path, window_deg: float) -> None:
    """Where the robot actually drove in each corridor, measured from /scan alone.

    Answers whether the centre bias really lands on the inner wall, and if not,
    which layer is at fault. Hardware telemetry put the CW run inner on all four
    corridors but the CCW run OUTER on east and west; those numbers came from
    pose plus the corridor-width BELIEF, so a wrong belief would corrupt them --
    including the run that looked correct.

    Four quantities per corridor traversal, three of them independent of pose:

    * ``true width``   -- d_left + d_right from the scan. Self-validating: it
      should land near the mat's real 0.60/1.00 m corridors, and if it does not,
      nothing else in the row is trustworthy either.
    * ``belief``       -- what CorridorWidthEstimator thought at that moment.
    * ``actual``       -- the robot's offset from the TRUE centre, + = toward
      the inner block. Pure scan geometry.
    * ``planned``      -- the same offset for the point the controller was
      steering at. Taken as target-minus-pose projected onto the robot's own
      lateral axis, so a pose error cancels: both terms carry it equally, which
      is also exactly what the controller itself acts on.

    ``planned`` is the discriminator. A sign error puts it outer regardless of
    the belief; a belief error puts it outer only by as much as the width is
    wrong. ``actual - planned`` is tracking error and blames neither.
    """
    scans, rows = read_bag(open_reader(bag_dir), _LIDAR_YAW_OFFSET_RAD)
    if not scans or not rows:
        print(f"{bag_dir.name}: no scans or no /nav_debug rows")
        return

    half_width = math.radians(window_deg)
    tuning = NavigationTuning.load_default()
    intended = tuning.waypoints.WIDE_CENTER_BIAS_M
    row_times = [t for t, _ in rows]
    samples: dict[tuple[str, str], list[CorridorOffsetSample]] = {}
    rejected: Counter[str] = Counter()

    for t_scan, scan in scans:
        snap = _paired_snapshot(rows, row_times, t_scan)
        if snap is None:
            rejected["no paired /nav_debug within 0.1s"] += 1
            continue
        outcome = _offset_sample(scan, snap, half_width)
        if isinstance(outcome, str):
            rejected[outcome] += 1
            continue
        key = (snap.direction.value, snap.current_corridor.value)
        samples.setdefault(key, []).append(outcome)

    if not samples:
        print(f"{bag_dir.name}: no usable samples")
        for reason, n in rejected.most_common():
            print(f"  rejected {n:6d}: {reason}")
        return

    def med(values: Sequence[float]) -> float:
        clean = sorted(v for v in values if not math.isnan(v))
        return clean[len(clean) // 2] if clean else math.nan

    print(f"\n{bag_dir.name}  (intended bias {intended:+.3f} m toward inner, window +/-{window_deg:.0f} deg)")
    table = []
    for (direction, section), vals in sorted(samples.items()):
        width = [v.width for v in vals]
        belief = [v.belief for v in vals]
        actual = [v.actual for v in vals]
        planned = [v.planned for v in vals]
        table.append(
            [
                direction,
                section,
                len(vals),
                f"{med(width):.3f}",
                "n/a" if math.isnan(med(belief)) else f"{med(belief):.3f}",
                f"{med(actual):+.3f}",
                f"{med(planned):+.3f}",
                f"{med(actual) - med(planned):+.3f}",
            ]
        )
    print_table(
        table,
        ["dir", "corridor", "n", "true w", "belief", "actual", "planned", "track"],
    )
    print("  + = toward the inner block. 'planned' is where the controller aimed.")
    for reason, n in rejected.most_common(4):
        print(f"  rejected {n:6d}: {reason}")


def _bearing_bin(deg: float, bin_deg: float = _BEARING_BIN_DEG) -> int:
    """Bin a bearing (deg) to the nearest ``bin_deg`` multiple, wrapped into (-180, 180].

    A plain ``round(deg / bin_deg) * bin_deg`` puts +179 deg and -179 deg -- 2
    degrees apart on the sensor -- into different bins (+180 and -180). Wrapping the
    bin centre back into (-180, 180] merges them into the one bin that straddles the seam.
    """
    center = round(deg / bin_deg) * bin_deg
    return int(((center + 180) % 360) - 180)


def _near_histogram(bag_dir: Path) -> None:
    """Full-scan bearing histogram of sub-_NEAR_THRESHOLD_M returns (tmp_scan_min.py)."""
    reader = open_reader(bag_dir)
    n_msgs = 0
    hdr: tuple[float, float, float, float, int] | None = None
    tiny_counts: list[int] = []
    bearing_hist: Counter[int] = Counter()
    val_hist: Counter[float] = Counter()
    zero_exact = 0
    tiny_total = 0
    ray_total = 0

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != Topics.SCAN:
            continue
        msg = deserialize_message(data, LaserScan)
        n_msgs += 1
        if hdr is None:
            hdr = (msg.range_min, msg.range_max, msg.angle_min, msg.angle_max, len(msg.ranges))
        tiny = 0
        n = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            ray_total += 1
            if not math.isfinite(r) or r >= _NEAR_THRESHOLD_M:
                continue
            tiny += 1
            tiny_total += 1
            if r == 0.0:
                zero_exact += 1
            else:
                val_hist[round(r, 3)] += 1
                deg = math.degrees(msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1))
                bearing_hist[round(deg / 10) * 10] += 1
        tiny_counts.append(tiny)

    print(f"\n--- near histogram (sub-{_NEAR_THRESHOLD_M:.2f} m returns, raw sensor frame) ---")
    print(f"/scan messages: {n_msgs}")
    if hdr is None:
        print("no /scan messages")
        return
    print(f"declared range_min={hdr[0]:.3f} range_max={hdr[1]:.3f} rays/scan={hdr[4]}")
    print(f"rays below {_NEAR_THRESHOLD_M:.2f} m: {tiny_total} of {ray_total} ({100 * tiny_total / max(ray_total, 1):.3f}%)")
    print(f"  exactly 0.0: {zero_exact}")
    print(f"  nonzero sub-{_NEAR_THRESHOLD_M:.2f}: {tiny_total - zero_exact}")
    if tiny_counts:
        sorted_counts = sorted(tiny_counts)
        print(f"per-scan count: min={sorted_counts[0]} med={sorted_counts[len(sorted_counts) // 2]} max={sorted_counts[-1]}")
        print(f"scans with at least one: {sum(1 for c in tiny_counts if c)} / {len(tiny_counts)}")
    if val_hist:
        print(f"most common nonzero sub-{_NEAR_THRESHOLD_M:.2f} values (m): {val_hist.most_common(10)}")
    if bearing_hist:
        rows = list(sorted(bearing_hist.items(), key=lambda kv: -kv[1])[:12])
        print_table(rows, ["bearing_deg", "count"])


def _dropout_symmetry(bag_dir: Path) -> None:
    """No-return/near-zero symmetry by 15-deg-binned robot-frame bearing (tmp_dropout.py)."""
    reader = open_reader(bag_dir)
    bad: Counter[int] = Counter()
    tot: Counter[int] = Counter()
    near: Counter[int] = Counter()

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != Topics.SCAN:
            continue
        msg = deserialize_message(data, LaserScan)
        n = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            raw_deg = math.degrees(msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1))
            # Robot frame = sensor frame + mount offset -- see decode_scan's docstring.
            deg = (raw_deg + math.degrees(_LIDAR_YAW_OFFSET_RAD) + 180) % 360 - 180
            b = _bearing_bin(deg)
            tot[b] += 1
            if not math.isfinite(r):
                bad[b] += 1
            elif r < _NEAR_THRESHOLD_M:
                near[b] += 1

    print(f"\n--- dropout symmetry (robot frame, {_BEARING_BIN_DEG:.0f}-deg bins) ---")
    print(f"LIDAR yaw offset applied: {math.degrees(_LIDAR_YAW_OFFSET_RAD):.1f} deg")
    rows = [
        (b, tot[b], bad[b], f"{100 * bad[b] / tot[b]:.1f}%", near[b], f"{100 * near[b] / tot[b]:.1f}%")
        for b in sorted(tot)
    ]
    print_table(rows, ["bearing_deg", "rays", "no_return", "no_return_%", f"sub_{_NEAR_THRESHOLD_M:.2f}", "sub_%"])

    def side(lo: int, hi: int) -> tuple[int, int, int]:
        keys = [b for b in tot if lo <= b <= hi]
        return sum(tot[b] for b in keys), sum(bad[b] for b in keys), sum(near[b] for b in keys)

    print("\nleft/right symmetry (robot frame: +90=left, -90=right):")
    for label, lo, hi in (("LEFT  (+75..+105)", 75, 105), ("RIGHT (-105..-75)", -105, -75)):
        total, no_return, sub = side(lo, hi)
        print(
            f"  {label}: rays={total} no_return={no_return} ({100 * no_return / max(total, 1):.1f}%) "
            f"sub_{_NEAR_THRESHOLD_M:.2f}={sub} ({100 * sub / max(total, 1):.1f}%)",
        )


def _rear_occlusion(bag_dir: Path, bin_deg: float) -> None:
    """Measure which rear bearings the mount can actually READ, and derive the blind wedges.

    ``lidar_sectors.toml`` masks the rear by ANGLE, unconditionally, because a
    no-return cannot be told from open road on a single scan: the gateway
    substitutes max range, so an occluded bearing reads 12 m and looks
    maximally clear. That is why "is this bearing readable right now" is not a
    safe runtime test, and why the wedges are measured once instead.

    Over a whole bag it IS decidable, because occlusion is geometric and
    static. An occluded bearing gives the same degenerate answer on every scan
    however the robot moves -- a no-return, or a self-detection return off the
    chassis -- while an open bearing returns plausible ranges that VARY as the
    robot drives. So a bearing counts as readable here on two conditions, not
    one: enough valid returns, and spread among them. Valid-fraction alone
    would pass a bearing pinned to a constant by its own mount.

    Prints the per-bin evidence and the four wedge numbers it implies, rather
    than editing config: these bound whether the robot may reverse, so the
    numbers should be read by a human before they are shipped.

    Args:
        bag_dir: Bag to replay.
        bin_deg: Bearing bin width. Finer than ``_BEARING_BIN_DEG`` on purpose
            -- the 2026-08-04 slot was ~25 deg wide and 15 deg bins would
            straddle its edges.
    """
    sectors = NavigationTuning.load_default().lidar_sectors
    tot: Counter[int] = Counter()
    valid: Counter[int] = Counter()
    ranges: dict[int, list[float]] = {}

    reader = open_reader(bag_dir)
    scans = 0
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != Topics.SCAN:
            continue
        msg = deserialize_message(data, LaserScan)
        scans += 1
        n = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            raw_deg = math.degrees(msg.angle_min + i * (msg.angle_max - msg.angle_min) / max(n - 1, 1))
            deg = (raw_deg + math.degrees(_LIDAR_YAW_OFFSET_RAD) + 180) % 360 - 180
            b = int(math.floor(deg / bin_deg) * bin_deg)
            tot[b] += 1
            # Excluded for the same three reasons _rear_clearance excludes them:
            # no-returns, sub-spec readings, and the chassis seeing itself.
            if (
                math.isfinite(r)
                and r > sectors.MIN_VALID_RANGE_M
                and r > sectors.SELF_DETECTION_THRESHOLD_M
                and r < _MAX_RANGE_M
            ):
                valid[b] += 1
                ranges.setdefault(b, []).append(r)

    if not scans:
        print("\nno /scan messages in this bag")
        return

    def readable(b: int) -> bool:
        rs = ranges.get(b, [])
        if tot[b] == 0 or valid[b] / tot[b] < _REAR_VALID_FRACTION:
            return False
        return len(rs) >= _MIN_VOTES and (max(rs) - min(rs)) >= _REAR_RANGE_SPREAD_M

    print(f"\n--- rear occlusion ({scans} scans, {bin_deg:.0f}-deg bins, robot frame) ---")
    print(f"LIDAR yaw offset applied: {math.degrees(_LIDAR_YAW_OFFSET_RAD):.1f} deg")
    print(f"readable = valid >= {_REAR_VALID_FRACTION:.0%} of rays AND range spread >= {_REAR_RANGE_SPREAD_M} m")
    rows = []
    for b in sorted(tot):
        if abs(b) < 90:  # noqa: PLR2004 - rear half only; the front is not in question
            continue
        rs = ranges.get(b, [])
        spread = f"{max(rs) - min(rs):.2f}" if rs else "-"
        rows.append([
            b,
            tot[b],
            f"{100 * valid[b] / tot[b]:.1f}%",
            f"{median(rs):.2f}" if rs else "-",
            spread,
            "READABLE" if readable(b) else "blind",
        ])
    print_table(rows, ["bearing_deg", "rays", "valid_%", "median_m", "spread_m", "verdict"])

    # The wedges are the blind arcs either side of the readable slot, so the
    # slot is found first and the wedges are what is left over.
    rear_bins = sorted(b for b in tot if abs(b) >= 90)  # noqa: PLR2004
    slot = [b for b in rear_bins if readable(b)]
    print("\nCURRENT config (lidar_sectors.toml):")
    print(f"  left  {sectors.BLIND_WEDGE_LEFT_MIN_DEG:.1f}..{sectors.BLIND_WEDGE_LEFT_MAX_DEG:.1f}")
    print(f"  right {sectors.BLIND_WEDGE_RIGHT_MIN_DEG:.1f}..{sectors.BLIND_WEDGE_RIGHT_MAX_DEG:.1f}")
    if not slot:
        print("\nNo readable rear bearing in this bag -- the rear is genuinely blind. Leave the wedges closed.")
        return
    negative = [b for b in slot if b < 0]
    positive = [b for b in slot if b >= 0]
    print(f"\nreadable rear bearings: {slot}")
    print("MEASURED wedges (blind arcs either side of that slot):")
    print(f"  blind_wedge_left_min_deg  = -180.0")
    print(f"  blind_wedge_left_max_deg  = {min(negative) if negative else -180.0:.1f}")
    print(f"  blind_wedge_right_min_deg = {max(positive) + bin_deg if positive else 180.0:.1f}")
    print(f"  blind_wedge_right_max_deg = 180.0")
    print(
        "\nCheck against a SECOND bag before shipping: one bag can only show a bearing was readable "
        "in the situations that bag happened to contain.",
    )


def main() -> None:
    """Replay a bag's /scan and /nav_debug, print the side-ray recovery report, and any opt-in sections."""
    parser = create_bag_parser(
        "Compare a single side-facing LIDAR ray against a windowed median, to test whether "
        "the direction gate is starved by beam-level noise rather than an absent signal.",
    )
    parser.add_argument("--window-deg", type=float, default=5.0)
    parser.add_argument("--near-histogram", action="store_true", help="full-scan bearing histogram of sub-0.05m returns")
    parser.add_argument(
        "--dropout-symmetry",
        action="store_true",
        help="15-deg-binned robot-frame no-return/near-zero histogram + left/right symmetry summary",
    )
    parser.add_argument(
        "--centre-offset",
        action="store_true",
        help="per-corridor true width, width belief, and where the robot actually drove vs where it aimed",
    )
    parser.add_argument(
        "--rear-occlusion",
        action="store_true",
        help="which rear bearings this mount can actually read, and the blind wedges they imply",
    )
    parser.add_argument("--rear-bin-deg", type=float, default=5.0, help="bearing bin width for --rear-occlusion")
    args = parser.parse_args()
    half = math.radians(args.window_deg)

    if args.near_histogram:
        _near_histogram(args.bag_dir)
    if args.dropout_symmetry:
        _dropout_symmetry(args.bag_dir)
    if args.centre_offset:
        _centre_offset(args.bag_dir, args.window_deg)
    if args.rear_occlusion:
        _rear_occlusion(args.bag_dir, args.rear_bin_deg)
        return

    tuning = NavigationTuning.load_default()
    estimator = tuning.direction_estimator

    reader = open_reader(args.bag_dir)
    # Same rotation ROS2HardwareGateway._lidar_callback applies -- the LIDAR is
    # mounted inverted, so raw bearings are 180 deg out and a replay that skips
    # this reads left as right and infers the mirror image of the direction
    # the node actually inferred.
    scans, nav_rows = read_bag(reader, _LIDAR_YAW_OFFSET_RAD)
    yaws: list[tuple[float, float]] = [(t, snap.pose_yaw) for t, snap in nav_rows if snap.pose_yaw is not None]
    poses: list[tuple[float, float, float]] = [
        (t, snap.pose_x, snap.pose_y) for t, snap in nav_rows if snap.pose_x is not None and snap.pose_y is not None
    ]

    print(f"== {args.bag_dir.name}  scans={len(scans)}  window=+/-{args.window_deg:.0f}deg")

    if not scans:
        print("no /scan messages")
        return

    beams = len(scans[0][1].ranges_m)
    total = sum(len(scan.ranges_m) for _, scan in scans)
    dropouts = sum(1 for _, scan in scans for v in scan.ranges_m if v >= _MAX_RANGE_M)
    print(f"beams/scan={beams}  overall max-range fraction: {100.0 * dropouts / total:.1f}%")

    def nearest_yaw(t: float) -> float | None:
        if not yaws:
            return None
        return min(yaws, key=lambda p: abs(p[0] - t))[1]

    recovered = Counter()
    rows = []
    for t, scan in scans:
        yaw = nearest_yaw(t)
        if yaw is None:
            continue
        s_l = _single(scan.ranges_m, scan.angles_rad, math.pi / 2)
        s_r = _single(scan.ranges_m, scan.angles_rad, -math.pi / 2)
        w_l = _windowed(scan.ranges_m, scan.angles_rad, math.pi / 2, half)
        w_r = _windowed(scan.ranges_m, scan.angles_rad, -math.pi / 2, half)
        for single, win in ((s_l, w_l), (s_r, w_r)):
            if single >= _MAX_RANGE_M:
                recovered["single max-range"] += 1
                if win is None:
                    recovered["  window also empty (real dropout)"] += 1
                else:
                    recovered["  window recovers a reading"] += 1
                    if win > estimator.PLAUSIBLE_SPAN_THRESHOLD_M:
                        recovered["    ...and it is an OPEN side (>1.25m)"] += 1
        rows.append((t, yaw, s_l, s_r, w_l, w_r))

    print("\nside-ray recovery:")
    if recovered:
        recovery_rows = [(name, count) for name, count in recovered.items()]
        print_table(recovery_rows, ["metric", "count"])

    # Winding sense of the pose trace, as ground truth.
    total_ang = 0.0
    prev = None
    for _, x, y in poses:
        ang = math.atan2(y - _MAT_CENTRE_Y, x - _MAT_CENTRE_X)
        if prev is not None:
            total_ang += wrap_angle(ang - prev)
        prev = ang
    truth = "counterclockwise" if total_ang > 0 else "clockwise"
    print(f"\npose winding: {total_ang / (2 * math.pi):+.2f} turns -> travelling {truth}")

    def settle(use_window: bool) -> tuple[float, str, int] | None:
        votes: Counter[str] = Counter()
        cast = 0
        for t, yaw, s_l, s_r, w_l, w_r in rows:
            left, right = (w_l, w_r) if use_window else (s_l, s_r)
            if left is None or right is None:
                continue
            if left > estimator.MAX_IN_TRACK_RANGE_M or right > estimator.MAX_IN_TRACK_RANGE_M:
                continue
            axis_error = abs(wrap_angle(yaw - round(yaw / (math.pi / 2)) * (math.pi / 2)))
            if axis_error > estimator.ALIGNMENT_TOLERANCE_RAD:
                continue
            if left + right <= estimator.PLAUSIBLE_SPAN_THRESHOLD_M:
                continue
            if abs(left - right) < estimator.MIN_ASYMMETRY_M:
                continue
            inferred = "clockwise" if right > left else "counterclockwise"
            votes[inferred] += 1
            cast += 1
            if votes[inferred] >= _MIN_VOTES:
                return t, inferred, cast
        return None

    print("\nsettle with shipped thresholds:")
    for label, use_window in (("single ray (shipped)", False), ("windowed median", True)):
        result = settle(use_window)
        if result is None:
            print(f"  {label:24s} never settles")
        else:
            t, direction, cast = result
            mark = "OK" if direction == truth else "WRONG"
            print(f"  {label:24s} settles {direction} at {t:6.1f}s ({cast} votes) [{mark}]")


if __name__ == "__main__":
    main()
