"""Review a race bag against the three things the 2026-08-06 rounds raised.

Prints, for one bag:

  * What the start measurement read, and how far it sat from the assumption it
    replaced -- the fields added in 626a011, reported here for the first
    hardware runs that carry them.
  * Where the robot slowed and why, by attributing each commanded speed to the
    limiter that produced it (clearance vs heading), alongside the clearance
    and risk at that tick. The reported cause of a slowdown is the smaller of
    the two limiters, not a guess.
  * How close it came to a wall, and what it was doing at the closest ticks.

``--stats`` adds a min_lidar_range histogram, steer sign-flip rate, crosstrack/
angle_error percentiles and an open-path clearance-limiting check (folded in
from a one-off tmp_speed_probe.py).

``--center-bias`` computes, per corridor, the lateral offset from the OUTER
wall relative to the believed corridor width -- i.e. whether the path centres
itself or drifts toward one wall (folded in from a one-off tmp_review4.py).
It needs the per-corridor width belief the run used, passed via
``--widths north=1.0,south=0.6,east=1.0,west=0.6`` (there is no run-agnostic
default -- the belief is whatever that specific run settled on).

Usage:
    pixi run -e dev python scripts/bag/diag_bag_review.py data/live/runs/run_XXXXXXXX_XXXXXX
    pixi run -e dev python scripts/bag/diag_bag_review.py RUN_DIR --stats
    pixi run -e dev python scripts/bag/diag_bag_review.py RUN_DIR --center-bias \
        --widths north=1.0,south=0.6,east=1.0,west=0.6
"""

from __future__ import annotations

import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from typing import TYPE_CHECKING

from rclpy.serialization import deserialize_message
from shared.config.constants import TrackDimensions
from std_msgs.msg import String

from scripts.common.bag_io import Topics, create_bag_parser, decode_nav_debug, elapsed_seconds, open_reader
from scripts.common.stats import median, percentile
from scripts.common.tables import fmt_optional, print_table

if TYPE_CHECKING:
    from shared.domain.models import NavigatorDebugSnapshot

_MIN_LIDAR_RANGE_BUCKETS_M = (0.02, 0.05, 0.10, 0.20, 0.40)
_OPEN_PATH_CLEARANCE_M = 0.8
"""forward_clearance_m above which the path is considered open (tmp_speed_probe.py)."""
_WIDE_CORRIDOR_THRESHOLD_M = 0.9
"""Width at/above which a corridor is tagged WIDE rather than narrow, for display only."""
_TINY_LIDAR_RANGE_M = 0.10
"""min_lidar_range_m below which an open-path tick is flagged as suspiciously close."""


def _band(v: float, edges: tuple[float, ...]) -> str:
    for e in edges:
        if v < e:
            return f"<{e}"
    return f">={edges[-1]}"


def _parse_widths(text: str) -> dict[str, float]:
    """Parse ``north=1.0,south=0.6,...`` into a section-name -> width(m) dict."""
    widths: dict[str, float] = {}
    for pair in text.split(","):
        section, _, value = pair.partition("=")
        if not section or not value:
            msg = f"invalid --widths entry {pair!r}, expected section=width_m"
            raise ValueError(msg)
        widths[section.strip().lower()] = float(value)
    return widths


def _lateral_from_outer_wall(section: str, x: float, y: float, mat_size: float) -> float | None:
    """Distance from the OUTER wall for a pose believed to be in ``section``."""
    if section == "south":
        return y
    if section == "north":
        return mat_size - y
    if section == "west":
        return x
    if section == "east":
        return mat_size - x
    return None


def _print_stats(driving: list[tuple[float, NavigatorDebugSnapshot]]) -> None:
    """Aggregate diagnostics folded in from tmp_speed_probe.py."""
    nd = [(t, s) for t, s in driving if s.phase.value == "normal_drive"]
    print(f"\n--- stats (normal_drive ticks: {len(nd)}) ---")
    if not nd:
        print("  no normal_drive ticks in this bag -- skipping stats")
        return

    phases = Counter(s.phase.value for _, s in driving)
    print(f"  phase mix: {dict(phases.most_common())}")

    both = [(t, s) for t, s in nd if s.clearance_speed_mps is not None and s.heading_speed_mps is not None]
    if both:
        binder = Counter("clearance" if s.clearance_speed_mps <= s.heading_speed_mps else "heading" for _, s in both)
        print(f"  binding limiter (ticks with both limiters={len(both)}): {dict(binder)}")

        open_path = [s for _, s in both if s.forward_clearance_m is not None and s.forward_clearance_m > _OPEN_PATH_CLEARANCE_M]
        print(f"  clearance limiting on open path (forward_clearance > {_OPEN_PATH_CLEARANCE_M} m): {len(open_path)} ticks")
        if open_path:
            cs = [s.clearance_speed_mps for s in open_path]
            hs = [s.heading_speed_mps for s in open_path]
            print(f"    clearance_speed med={median(cs):.3f} min={min(cs):.3f} max={max(cs):.3f}")
            print(f"    heading_speed   med={median(hs):.3f} min={min(hs):.3f} max={max(hs):.3f}")
            mr = [s.min_lidar_range_m for s in open_path if s.min_lidar_range_m is not None]
            if mr:
                tiny = sum(1 for m in mr if m < _TINY_LIDAR_RANGE_M)
                print(f"    min_lidar_range < {_TINY_LIDAR_RANGE_M} m on {tiny}/{len(mr)} of these open-path ticks")

    mr_all = [s.min_lidar_range_m for _, s in nd if s.min_lidar_range_m is not None]
    if mr_all:
        buckets = Counter(_band(m, _MIN_LIDAR_RANGE_BUCKETS_M) for m in mr_all)
        print(f"  min_lidar_range buckets: {dict(sorted(buckets.items()))}")

    steers = [s.commanded_steering_norm for _, s in nd if s.commanded_steering_norm is not None]
    if steers:
        flips = sum(1 for i in range(1, len(steers)) if steers[i] * steers[i - 1] < 0)
        dur = nd[-1][0] - nd[0][0]
        print(f"  steer sign flips: {flips} over {dur:.0f}s = {flips / max(dur, 1):.2f}/s")

    xt = [abs(s.crosstrack_error_m) for _, s in nd if s.crosstrack_error_m is not None]
    if xt:
        print(f"  |crosstrack| med={median(xt):.3f} p90={percentile(xt, 0.9):.3f} max={max(xt):.3f} m")

    ae = [abs(s.angle_error_rad) for _, s in nd if s.angle_error_rad is not None]
    if ae:
        print(f"  |angle_error| med={median(ae):.3f} p90={percentile(ae, 0.9):.3f} max={max(ae):.3f} rad")


def _print_center_bias(driving: list[tuple[float, NavigatorDebugSnapshot]], widths: dict[str, float]) -> None:
    """Per-corridor centring bias (offset from centre, + toward inner wall)."""
    print("\n--- center bias (offset from corridor centre, + = toward INNER wall) ---")
    mat_size = TrackDimensions.TRACK_SIZE
    nd = [(t, s) for t, s in driving if s.phase.value == "normal_drive"]
    off: dict[str, list[float]] = defaultdict(list)
    for _, s in nd:
        if s.current_corridor is None or s.pose_x is None or s.pose_y is None:
            continue
        section = s.current_corridor.value
        width = widths.get(section)
        if width is None:
            continue
        lateral = _lateral_from_outer_wall(section, s.pose_x, s.pose_y, mat_size)
        if lateral is None or not (0 < lateral < width):
            continue
        off[section].append(lateral - width / 2)
    if not off:
        print("  no ticks matched the given --widths sections")
        return
    rows = []
    for section in sorted(off):
        values = off[section]
        tag = "WIDE" if widths[section] >= _WIDE_CORRIDOR_THRESHOLD_M else "narrow"
        rows.append((
            section,
            f"{widths[section] * 100:.0f}cm {tag}",
            len(values),
            f"{median(values):+.3f}",
            f"{percentile(values, 0.1):+.3f}",
            f"{percentile(values, 0.9):+.3f}",
        ))
    print_table(rows, ["corridor", "width", "n", "median", "p10", "p90"])


def _read(bag_dir: Path) -> tuple[list[tuple[float, NavigatorDebugSnapshot]], list[tuple[float, str]]]:
    """Return (nav_debug ticks, robot_state transitions), both stamped from bag start."""
    reader = open_reader(bag_dir)
    ticks: list[tuple[float, NavigatorDebugSnapshot]] = []
    states: list[tuple[float, str]] = []
    t0: int | None = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if t0 is None:
            t0 = stamp
        rel = elapsed_seconds(stamp, t0)
        if topic == Topics.NAV_DEBUG:
            ticks.append((rel, decode_nav_debug(data)))
        elif topic == Topics.ROBOT_STATE:
            value = deserialize_message(data, String).data
            if not states or states[-1][1] != value:
                states.append((rel, value))
    return ticks, states


def _print_slowdowns(driving: list[tuple[float, NavigatorDebugSnapshot]], slow_below: float) -> None:
    """Report every slow tick, attributed to whichever limiter produced it."""
    print(f"\n--- slowdowns (commanded < {slow_below} m/s while racing) ---")
    slow = [(t, d) for t, d in driving if (d.commanded_speed_mps or 1.0) < slow_below]
    print(f"  {len(slow)} of {len(driving)} driving ticks")
    if not slow:
        return
    by_clearance = sum(
        1
        for _, d in slow
        if d.clearance_speed_mps is not None and d.heading_speed_mps is not None and d.clearance_speed_mps <= d.heading_speed_mps
    )
    print(f"  limited by clearance: {by_clearance}   by heading: {len(slow) - by_clearance}")
    print("  worst 8 ticks:")
    rows = []
    for t, d in sorted(slow, key=lambda p: p[1].commanded_speed_mps or 0.0)[:8]:
        rows.append((
            t,
            d.commanded_speed_mps,
            d.clearance_speed_mps,
            d.heading_speed_mps,
            d.forward_clearance_m,
            d.min_lidar_range_m,
            d.crosstrack_error_m,
            d.risk,
            d.phase
        ))
    print_table(rows, ["t", "speed", "clear_v", "head_v", "fwd_clr", "min_rng", "xtrack", "risk", "phase"])


def main() -> None:
    """Print the review for the bag named on the command line."""
    parser = create_bag_parser("Review a race bag: start measurement, slowdowns, wall proximity, recovery.")
    parser.add_argument("--slow-below", type=float, default=0.14, help="m/s counted as a slowdown")
    parser.add_argument("--stats", action="store_true", help="print min_lidar_range histogram, steer/crosstrack/angle_error stats")
    parser.add_argument("--center-bias", action="store_true", help="print per-corridor centring bias (needs --widths)")
    parser.add_argument(
        "--widths",
        type=str,
        default=None,
        help="per-corridor width belief for --center-bias, e.g. north=1.0,south=0.6,east=1.0,west=0.6",
    )
    args = parser.parse_args()
    if args.center_bias and not args.widths:
        parser.error("--center-bias requires --widths north=..,south=..,east=..,west=..")

    ticks, states = _read(args.bag_dir)
    driving = [(t, d) for t, d in ticks if d.pose_x is not None]

    print(f"bag: {args.bag_dir.name}   ticks: {len(ticks)}   driving: {len(driving)}")
    print(f"states: {', '.join(f'{t:.1f}s {s}' for t, s in states)}")

    # Start measurement.
    print("\n--- start measurement ---")
    measured = next((d for _, d in ticks if d.start_measurement_ahead_m is not None), None)
    if measured is None:
        print("  never populated: the measurement refused every scan, or this bag predates it")
    else:
        mx, my = measured.start_measured_x, measured.start_measured_y
        print(f"  measured pose:    ({fmt_optional(mx)}, {fmt_optional(my)})")
        print(f"  track ahead:      {fmt_optional(measured.start_measurement_ahead_m)} m")
        print(f"  corridor width:   {fmt_optional(measured.start_measured_corridor_width_m)} m")
        first_pose = driving[0][1] if driving else None
        if first_pose is not None:
            fx, fy = first_pose.pose_x, first_pose.pose_y
            offset = ((mx - fx) ** 2 + (my - fy) ** 2) ** 0.5
            print(f"  first logged pose:({fmt_optional(fx)}, {fmt_optional(fy)})  -> measurement moved it {offset:.3f} m")

    # Laps and direction.
    print("\n--- progress ---")
    laps = [(t, d.laps_completed) for t, d in driving]
    if laps:
        bumps = [(t, n) for (t, n), (_, prev) in zip(laps[1:], laps, strict=False) if n != prev]
        print(f"  laps completed: {laps[-1][1]} of {driving[-1][1].num_laps}")
        print(f"  lap timestamps: {', '.join(f'{t:.1f}s->{n}' for t, n in bumps) or 'none'}")
    direction = next((d.direction for _, d in driving if d.direction), None)
    print(f"  direction: {direction}")

    _print_slowdowns(driving, args.slow_below)

    # Wall proximity.
    print("\n--- closest approaches ---")
    near = sorted(
        (p for p in driving if p[1].min_lidar_range_m is not None),
        key=lambda p: p[1].min_lidar_range_m,
    )[:8]
    rows = []
    for t, d in near:
        rows.append((
            t,
            d.min_lidar_range_m,
            d.forward_clearance_m,
            d.crosstrack_error_m,
            d.commanded_steering_norm,
            d.active_maneuver_type,
            d.phase
        ))
    print_table(rows, ["t", "min_rng", "fwd_clr", "xtrack", "steer", "maneuver", "phase"])

    # Escapes and stuck detection.
    escapes = [(t, d) for t, d in driving if (d.escape_count or 0) > 0]
    stuck = [(t, d) for t, d in driving if d.is_stuck]
    print(f"\n--- recovery ---   escape ticks: {len(escapes)}   stuck ticks: {len(stuck)}")
    if escapes:
        print(f"  first escape at {escapes[0][0]:.1f}s, max escape_count {max(d.escape_count or 0 for _, d in escapes)}")
    maneuvers = {d.active_maneuver_type for _, d in driving if d.active_maneuver_type}
    print(f"  maneuver types seen: {', '.join(sorted(maneuvers)) or 'none'}")

    if args.stats:
        _print_stats(driving)
    if args.center_bias:
        _print_center_bias(driving, _parse_widths(args.widths))


if __name__ == "__main__":
    main()
