"""Replay /nav_debug (and optionally /ackermann_cmd) from a recorded race bag.

Supersedes the former ``diag_bag_timeline.py`` and ``diag_bag_trace.py`` --
all three were "replay /nav_debug, print a table of columns" scripts that
differed mainly in table granularity and time sampling. One script, three
column presets:

  * ``coarse`` (was diag_bag_timeline.py): one wide row per sampled tick --
    phase/corridor/waypoint/pose/clearance/risk/stuck. Coarse time sampling
    (``--every 1.0`` by default) for a whole-run overview.
  * ``trace``  (was diag_bag_trace.py): full-rate pose/risk/escape/stuck
    trace within a ``[--start, --until]`` window, optionally interleaved
    with ``/ackermann_cmd`` rows via ``--cmd``.
  * ``steer``  (default, the original diag_bag_steer.py): pure-pursuit
    inputs behind each steering command -- the waypoint chased, the target
    point, the heading error steering is proportional to, and the
    corridor-width belief the plan was built from.

``--stats`` adds a block of aggregate diagnostics folded in from two
one-off tmp scripts (tmp_compare.py, tmp_trace.py): target-distance vs.
lookahead ratio, waypoint-advance/stall-rate, a steering-value histogram,
the angle_error sign split, and a "target behind robot" (x_local<=0) check.
It works from the same normal_drive rows the table presets already decode,
so it does not duplicate any column/sampling logic.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset steer --until 7
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset coarse --every 1.0
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset trace --start 0 --until 12 --cmd
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --stats
"""

from __future__ import annotations

import math
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message

from scripts.common.bag_io import (
    Topics,
    create_bag_parser,
    decode_nav_debug,
    elapsed_seconds,
    load_nav_debug_rows,
    open_reader,
)
from scripts.common.tables import fmt_optional, print_table

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.models import NavigatorDebugSnapshot

_DEFAULT_COARSE_INTERVAL_S = 1.0
_TARGET_LOOKAHEAD_RATIO_ALERT = 3.0
"""tmp_compare.py's threshold for flagging a target that has drifted far past its lookahead distance."""


def _f6(v: float | None) -> str:
    return fmt_optional(v, "6.3f")


def _print_coarse(rows: Sequence[tuple[float, NavigatorDebugSnapshot]], every: float) -> None:
    next_print = 0.0
    for ts, snap in rows:
        if ts < next_print:
            continue
        next_print += every
        print(
            f"{ts:7.2f}s phase={snap.phase!s:14} corridor={snap.current_corridor!s:6} "
            f"wp={snap.waypoint_index!s:4} laps={snap.laps_completed} "
            f"pose=({snap.pose_x:.3f},{snap.pose_y:.3f},{snap.pose_yaw:.2f}) "
            f"fwd={snap.forward_clearance_m!s:6} min_r={snap.min_lidar_range_m!s:6} "
            f"risk={snap.risk!s:6} stuck={snap.is_stuck} stuck_cnt={snap.stuck_count}",
        )


def _print_trace(
    bag_dir: Path,
    start: float,
    until: float,
    show_cmd: bool,
) -> list[tuple[float, NavigatorDebugSnapshot]]:
    """Replay the raw bag once for the trace preset. Returns the nav_debug rows collected."""
    reader = open_reader(bag_dir)
    t_start = None
    nav_rows: list[tuple[float, NavigatorDebugSnapshot]] = []
    table_rows = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = elapsed_seconds(t, t_start)
        if not (start <= ts <= until):
            continue
        if topic == Topics.ACKERMANN_CMD and show_cmd:
            msg = deserialize_message(data, AckermannDriveStamped)
            print(f"{ts:7.2f}s CMD  speed={msg.drive.speed:6.3f} steer_rad={msg.drive.steering_angle:6.3f}")
            continue
        if topic != Topics.NAV_DEBUG:
            continue
        snap = decode_nav_debug(data)
        nav_rows.append((ts, snap))
        table_rows.append((
            ts,
            snap.phase,
            snap.pose_x,
            snap.pose_y,
            snap.pose_yaw,
            snap.forward_clearance_m,
            snap.risk,
            snap.escape_risk,
            snap.current_corridor,
            snap.commanded_speed_mps,
            snap.commanded_steering_norm,
            snap.escape_count,
            snap.stuck_count,
        ))
    if table_rows:
        print_table(
            table_rows,
            ["t", "phase", "x", "y", "yaw", "fwd", "risk", "erisk", "corr", "cmd_v", "cmd_s", "esc", "stuck"],
        )
    return nav_rows


def _print_steer(rows: Sequence[tuple[float, NavigatorDebugSnapshot]], start: float, until: float, every: float) -> None:
    table_rows = []
    next_print = start
    for ts, snap in rows:
        if not (start <= ts <= until) or ts < next_print:
            continue
        next_print += every
        table_rows.append((
            ts,
            snap.waypoint_index,
            snap.current_corridor,
            f"{snap.pose_x or 0:.3f}",
            f"{snap.pose_y or 0:.3f}",
            f"{snap.pose_yaw or 0:.3f}",
            f"{snap.steer_target_x or 0:.3f}",
            f"{snap.steer_target_y or 0:.3f}",
            snap.angle_error_rad,
            snap.crosstrack_error_m,
            snap.lookahead_distance_m,
            snap.commanded_steering_norm,
            snap.belief_north_m,
            snap.belief_south_m,
            snap.belief_east_m,
            snap.belief_west_m,
        ))
    print_table(
        table_rows,
        ["t", "wp", "corr", "x", "y", "yaw", "tgt_x", "tgt_y", "aerr", "xtrack", "look", "steer", "N", "S", "E", "W"],
    )


def _print_stats(rows: Sequence[tuple[float, NavigatorDebugSnapshot]]) -> None:
    """Aggregate steering diagnostics folded in from tmp_compare.py and tmp_trace.py.

    Guards the tmp_compare.py bug where a bag with zero normal_drive rows threw
    an unhandled IndexError/StatisticsError -- this prints a warning and returns
    instead of crashing.
    """
    nd = [(t, s) for t, s in rows if s.phase.value == "normal_drive"]
    print("\n--- stats (normal_drive ticks) ---")
    if not nd:
        print("  no normal_drive ticks in this bag -- skipping stats")
        return

    dur = nd[-1][0] - nd[0][0]

    def q(values: list[float], p: float) -> float:
        s = sorted(values)
        return s[min(int(len(s) * p), len(s) - 1)]

    # Target distance vs. lookahead (tmp_compare.py).
    dists = [
        math.hypot(s.steer_target_x - s.pose_x, s.steer_target_y - s.pose_y)
        for _, s in nd
        if None not in (s.steer_target_x, s.pose_x, s.steer_target_y, s.pose_y)
    ]
    looks = [s.lookahead_distance_m for _, s in nd if s.lookahead_distance_m is not None]
    if dists and looks:
        ratio = [d / look for d, look in zip(dists, looks, strict=False) if look]
        print(f"  target distance   med={statistics.median(dists):.2f}  p90={q(dists, 0.9):.2f}  max={max(dists):.2f} m")
        print(f"  lookahead         med={statistics.median(looks):.2f}  p90={q(looks, 0.9):.2f}  max={max(looks):.2f} m")
        if ratio:
            print(f"  dist/lookahead    med={statistics.median(ratio):.1f}x  p90={q(ratio, 0.9):.1f}x  max={max(ratio):.1f}x")
            over = sum(1 for r in ratio if r > _TARGET_LOOKAHEAD_RATIO_ALERT)
            print(f"  target >{_TARGET_LOOKAHEAD_RATIO_ALERT:.0f}x lookahead on {over}/{len(ratio)} ticks")

    # Waypoint advance / stall rate (tmp_compare.py).
    idx = [s.waypoint_index for _, s in nd if s.waypoint_index is not None]
    if idx:
        advances = sum(1 for i in range(1, len(idx)) if idx[i] != idx[i - 1])
        best = cur = 0
        for i in range(1, len(idx)):
            cur = cur + 1 if idx[i] == idx[i - 1] else 1
            best = max(best, cur)
        stall = best * (dur / max(len(idx), 1))
        print(f"  waypoint index advances: {advances} ({advances / max(dur, 1):.2f}/s)  longest stall ~{stall:.0f}s")

    # Steering value histogram + angle_error sign split (tmp_trace.py).
    steer_vals = Counter(round(s.commanded_steering_norm, 4) for _, s in nd if s.commanded_steering_norm is not None)
    if steer_vals:
        print(f"  top steering values: {steer_vals.most_common(8)}")

    ae_signed = [s.angle_error_rad for _, s in nd if s.angle_error_rad is not None]
    if ae_signed:
        pos = sum(1 for v in ae_signed if v > 0)
        print(f"  angle_error sign: +{pos} / -{len(ae_signed) - pos}   median signed={statistics.median(ae_signed):.3f}")

    # "Target behind robot" check (tmp_trace.py): reconstruct the target in the
    # robot's local frame and count ticks where it sits behind (x_local <= 0).
    behind = total = 0
    for _, s in nd:
        if None in (s.pose_x, s.pose_y, s.pose_yaw, s.steer_target_x, s.steer_target_y):
            continue
        dx = s.steer_target_x - s.pose_x
        dy = s.steer_target_y - s.pose_y
        x_local = dx * math.cos(-s.pose_yaw) - dy * math.sin(-s.pose_yaw)
        total += 1
        if x_local <= 0:
            behind += 1
    if total:
        print(f"  target behind robot (x_local<=0): {behind}/{total}")


def main() -> None:
    """Parse CLI args, print the selected preset's table, and optionally the --stats block."""
    parser = create_bag_parser(
        "Replay /nav_debug from a race bag under one of three column presets "
        "(coarse/trace/steer), optionally with an aggregate --stats block.",
    )
    parser.add_argument(
        "--preset",
        choices=("coarse", "trace", "steer"),
        default="steer",
        help="Column set: coarse=whole-run overview, trace=full-rate escape-decision "
        "fields, steer=pure-pursuit inputs (default, the original diag_bag_steer.py).",
    )
    parser.add_argument("--start", type=float, default=0.0, help="Seconds into the bag to start printing (trace/steer).")
    parser.add_argument("--until", type=float, default=1e9, help="Seconds into the bag to stop printing (trace/steer).")
    parser.add_argument(
        "--every",
        type=float,
        default=None,
        help="Sample interval in seconds (0 = every row). Defaults to 1.0 for --preset coarse, 0.0 otherwise.",
    )
    parser.add_argument("--cmd", action="store_true", help="also print /ackermann_cmd rows (--preset trace only)")
    parser.add_argument("--stats", action="store_true", help="print aggregate steering/target diagnostics")
    args = parser.parse_args()

    every = args.every if args.every is not None else (_DEFAULT_COARSE_INTERVAL_S if args.preset == "coarse" else 0.0)

    if args.preset == "coarse":
        rows, _topics = load_nav_debug_rows(args.bag_dir)
        _print_coarse(rows, every)
    elif args.preset == "trace":
        rows = _print_trace(args.bag_dir, args.start, args.until, args.cmd)
    else:
        rows, _topics = load_nav_debug_rows(args.bag_dir)
        _print_steer(rows, args.start, args.until, every)

    if args.stats:
        if args.preset == "trace":
            # _print_trace only collected the windowed rows; stats look at the whole bag.
            rows, _topics = load_nav_debug_rows(args.bag_dir)
        _print_stats(rows)


if __name__ == "__main__":
    main()
