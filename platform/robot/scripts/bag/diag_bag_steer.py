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

``--effectiveness`` asks the question the table presets cannot: does the
chassis actually achieve the yaw rate its MEASURED steering angle implies?
Pure pursuit can command correctly and the servo obey exactly while the robot
still corners wide. A left/right split in the answer is a steering trim
offset, which shows up on track as a direction-dependent path error.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset steer --until 7
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset coarse --every 1.0
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --preset trace --start 0 --until 12 --cmd
    pixi run -e dev python scripts/bag/diag_bag_steer.py RUN_DIR --stats
    pixi run -e dev python scripts/bag/diag_bag_steer.py CW_RUN --effectiveness --pool CCW_RUN
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.serialization import deserialize_message
from shared.config.constants import RobotSpecs

from scripts.common.bag_io import (
    Topics,
    create_bag_parser,
    decode_nav_debug,
    elapsed_seconds,
    load_nav_debug_rows,
    open_reader,
    read_motion_streams,
)
from scripts.common.stats import median, nearest_by_time, percentile
from scripts.common.tables import fmt_optional, print_table
from src.navigation.utils import wrap_angle

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.models import NavigatorDebugSnapshot

_DEFAULT_COARSE_INTERVAL_S = 1.0
_TARGET_LOOKAHEAD_RATIO_ALERT = 3.0
"""tmp_compare.py's threshold for flagging a target that has drifted far past its lookahead distance."""

_L_EFF = RobotSpecs.WHEELBASE / (1.0 + abs(RobotSpecs.REAR_STEER_RATIO))
"""Counter-phase steering pivots about the chassis centre, so the turn reference
is half the wheelbase. Whether the chassis actually does this is exactly what
--effectiveness is testing, so this is the hypothesis under test, not a given."""
_YAW_WINDOW_S = 0.20
_MIN_STEER_DEG = 5.0
"""Below this the predicted yaw rate is small enough that the ratio is noise."""
_MIN_SPEED_MPS = 0.05
_PAIR_TOL_S = 0.15
_TRIM_SWEEP_DEG = 8.0
_TRIM_STEP_DEG = 0.5
_MIN_BUCKET = 50
"""Fewest samples a left/right bucket needs before its median is worth printing."""
_MIN_EFFECTIVE_STEER_DEG = 2.0
"""Once the trim is subtracted, a near-zero effective angle predicts a near-zero
yaw rate, and the ratio to it is meaningless. Dropping those is why the sample
count falls off on one side as the swept trim grows."""


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

    # Target distance vs. lookahead (tmp_compare.py).
    dists = [
        math.hypot(s.steer_target_x - s.pose_x, s.steer_target_y - s.pose_y)
        for _, s in nd
        if None not in (s.steer_target_x, s.pose_x, s.steer_target_y, s.pose_y)
    ]
    looks = [s.lookahead_distance_m for _, s in nd if s.lookahead_distance_m is not None]
    if dists and looks:
        ratio = [d / look for d, look in zip(dists, looks, strict=False) if look]
        print(f"  target distance   med={median(dists):.2f}  p90={percentile(dists, 0.9):.2f}  max={max(dists):.2f} m")
        print(f"  lookahead         med={median(looks):.2f}  p90={percentile(looks, 0.9):.2f}  max={max(looks):.2f} m")
        if ratio:
            print(f"  dist/lookahead    med={median(ratio):.1f}x  p90={percentile(ratio, 0.9):.1f}x  max={max(ratio):.1f}x")
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
        print(f"  angle_error sign: +{pos} / -{len(ae_signed) - pos}   median signed={median(ae_signed):.3f}")

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


def _effectiveness_samples(bag_dir: Path) -> list[tuple[float, float, float]]:
    """``(measured_steer_deg, speed_mps, achieved_yaw_rate)`` per IMU window.

    Steering is read from ``/motor/steering_position`` rather than from the
    command. On a Build HAT chassis that was a real encoder reading, which put
    servo tracking out of the loop; on the servo chassis it is the command
    echoed back (see :class:`~scripts.common.bag_io.MotionStreams`), so servo
    lag is back inside the measurement and any shortfall below is an upper
    bound on the geometric one.

    Speed is the commanded value. ``/motor/drive_speed`` is now trustworthy
    (see ``diag_bag_sim_fidelity.py``) but commanded speed is kept here so the
    trim figures stay comparable with the 2026-08-09 measurements this table
    was first read against.
    """
    streams = read_motion_streams(bag_dir)
    imu = streams.imu_yaw_rad
    steer = streams.steer_pos_deg
    speed = streams.cmd_speed_mps
    if not imu:
        return []
    step = max(1, int(len(imu) * _YAW_WINDOW_S / max(imu[-1][0], 1e-6)))
    steer_times = [t for t, _ in steer]
    speed_times = [t for t, _ in speed]
    out: list[tuple[float, float, float]] = []
    for i in range(len(imu) - step):
        t_a, y_a = imu[i]
        t_b, y_b = imu[i + step]
        dt = t_b - t_a
        if not 0.5 * _YAW_WINDOW_S <= dt <= 2.0 * _YAW_WINDOW_S:
            continue
        mid = 0.5 * (t_a + t_b)
        sdeg = nearest_by_time(steer, steer_times, mid, tolerance=_PAIR_TOL_S)
        v = nearest_by_time(speed, speed_times, mid, tolerance=_PAIR_TOL_S)
        if sdeg is None or v is None or abs(sdeg) < _MIN_STEER_DEG or v < _MIN_SPEED_MPS:
            continue
        out.append((sdeg, v, wrap_angle(y_b - y_a) / dt))
    return out


def _ratios(samples: Sequence[tuple[float, float, float]], trim_deg: float) -> tuple[list[float], list[float]]:
    """Achieved/predicted yaw ratios, split into (left, right), under a trim offset."""
    left: list[float] = []
    right: list[float] = []
    for sdeg, v, achieved in samples:
        effective = sdeg - trim_deg
        if abs(effective) < _MIN_EFFECTIVE_STEER_DEG:
            continue
        predicted = v / _L_EFF * math.tan(math.radians(effective))
        (left if sdeg > 0 else right).append(achieved / predicted)
    return left, right


def _print_effectiveness(bag_dirs: Sequence[Path]) -> None:
    """Does the chassis achieve the yaw its MEASURED steering angle implies?

    Pure pursuit can be commanding correctly and the servo obeying exactly while
    the robot still corners wide -- that is what the 2026-08-09 bags showed. This
    isolates the last link: actual wheel angle in, actual yaw rate out.

    A ratio near 1.0 means the kinematic model is right. A left/right split means
    a steering trim offset, which shows up as a direction-dependent path error
    (and is why CCW drifted ~3x further outward than CW). A symmetric shortfall
    means a scale error -- REAR_STEER_RATIO or linkage_ratio -- which this cannot
    tell apart, because both scale the prediction identically. That needs a
    protractor, not a bag.
    """
    samples: list[tuple[float, float, float]] = []
    for bag_dir in bag_dirs:
        got = _effectiveness_samples(bag_dir)
        print(f"  {bag_dir.name}: {len(got)} samples")
        samples += got
    if not samples:
        print("no usable samples (needs /imu/data, /motor/steering_position and /ackermann_cmd)")
        return

    print(f"\nL_eff = {_L_EFF:.4f} m  (wheelbase {RobotSpecs.WHEELBASE}, rear_steer_ratio {RobotSpecs.REAR_STEER_RATIO})")
    print("ratio = achieved yaw rate / predicted from measured steering angle\n")

    rows = []
    steps = int(_TRIM_SWEEP_DEG / _TRIM_STEP_DEG)
    best: tuple[float, float, float, float] | None = None
    for i in range(-steps, steps + 1):
        trim = i * _TRIM_STEP_DEG
        left, right = _ratios(samples, trim)
        if len(left) < _MIN_BUCKET or len(right) < _MIN_BUCKET:
            continue
        ml, mr = median(left), median(right)
        gap = abs(ml - mr)
        rows.append([f"{trim:+.1f}", len(left), f"{ml:.2f}", len(right), f"{mr:.2f}", f"{gap:.2f}"])
        if best is None or gap < best[3]:
            best = (trim, ml, mr, gap)
    print_table(rows, ["trim deg", "n_left", "left", "n_right", "right", "|gap|"])

    if best is None:
        return
    trim, ml, mr, _ = best
    print(f"\n  left/right agree at trim = {trim:+.1f} road-wheel deg: left {ml:.2f}, right {mr:.2f}")
    print(f"  -> steering.offset is in SERVO degrees: {trim:+.1f} / {RobotSpecs.LINKAGE_RATIO} = {trim / RobotSpecs.LINKAGE_RATIO:+.1f}")
    print("     (sign must be confirmed by eye -- move_steering_to flips it on `reversed`)")
    residual = 0.5 * (ml + mr)
    print(f"  residual scale after trim: {residual:.2f}; front-only L_eff would give {2 * residual:.2f}")
    print("     1.0 means the model is right. A symmetric shortfall is REAR_STEER_RATIO or")
    print("     linkage_ratio -- indistinguishable here, both scale the prediction alike.")


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
    parser.add_argument(
        "--effectiveness",
        action="store_true",
        help="does the chassis achieve the yaw its measured steering angle implies, and is it "
        "symmetric left/right (steering trim)? Needs /imu/data + /motor/steering_position.",
    )
    parser.add_argument(
        "--pool",
        type=Path,
        nargs="*",
        default=[],
        help="extra bags to pool into --effectiveness. One run is mostly one turn direction, "
        "so a CW and a CCW bag together are what make the left/right split readable.",
    )
    args = parser.parse_args()

    if args.effectiveness:
        _print_effectiveness([args.bag_dir, *args.pool])
        return

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
