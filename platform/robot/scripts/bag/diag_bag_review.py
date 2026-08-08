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

Usage:
    pixi run -e dev python scripts/bag/diag_bag_review.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from shared.domain.models import NavigatorDebugSnapshot
from std_msgs.msg import String

from scripts.common.bag_io import Topics, create_bag_parser, decode_nav_debug, elapsed_seconds, open_reader
from scripts.common.tables import fmt_optional, print_table


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
    parser = create_bag_parser("TODO: add description")
    parser.add_argument("--slow-below", type=float, default=0.14, help="m/s counted as a slowdown")
    args = parser.parse_args()

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


if __name__ == "__main__":
    main()
