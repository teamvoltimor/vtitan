"""Ask whether the pursuit controller is leaving steering authority unused.

The visual symptom is a robot that rounds a wide corner with a gentle arc when
it had lock to spare. This prints, per lap and per corridor, how much of the
steering range was actually commanded, which lookahead was selected, and what
the corridor-width belief was at the time -- so a shallow arc can be traced to
a long lookahead, to a rate limit, or to the plan itself asking for little.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_steer_headroom.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader, print_table

_LOOKAHEAD_PRECISION = 0.02
_STEER_HIGH_THRESHOLD = 0.8
_DELTA_TIME_MIN_S = 0.01
_DELTA_TIME_MAX_S = 0.5
_BELIEF_SAMPLE_INTERVAL_S = 20.0
_SHORT_LOOKAHEAD_M = 0.20
_TURN_THRESHOLD_RAD = 0.35
_SPEED_LOW_THRESHOLD_MPS = 0.10


def _pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)

    t_start = None
    rows = []
    states: list[tuple[float, str]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = elapsed_seconds(t, t_start)
        if topic == Topics.ROBOT_STATE:
            msg = deserialize_message(data, String)
            if not states or states[-1][1] != msg.data:
                states.append((ts, msg.data))
        elif topic == Topics.NAV_DEBUG:
            rows.append((ts, decode_nav_debug(data)))

    print(f"== {args.bag_dir.name} ==")
    print("state transitions: " + ", ".join(f"{t:.1f}s->{s}" for t, s in states))
    if not rows:
        print("no /nav_debug samples")
        return
    print(f"nav_debug samples: {len(rows)}  span {rows[0][0]:.1f}s..{rows[-1][0]:.1f}s")

    # Lap boundaries as reported by the navigator itself.
    lap_marks = []
    prev = None
    for t, snap in rows:
        lap = snap.laps_completed
        if lap != prev:
            lap_marks.append((t, prev, lap))
            prev = lap
    print("lap counter: " + ", ".join(f"{t:.1f}s {a}->{b}" for t, a, b in lap_marks))

    # Steering headroom, bucketed by which lookahead the controller selected.
    by_look: dict[str, list[float]] = defaultdict(list)
    by_corridor: dict[str, list[float]] = defaultdict(list)
    for t, snap in rows:
        steer = snap.commanded_steering_norm
        look = snap.lookahead_distance_m
        if not isinstance(steer, (int, float)) or not isinstance(look, (int, float)):
            continue
        by_look[f"look={look:.2f}"].append(abs(steer))
        corridor = str(snap.current_corridor)
        by_corridor[corridor].append(abs(steer))

    print("\n|steer| by selected lookahead (norm, 1.0 = full lock)")
    rows = []
    for key in sorted(by_look):
        v = by_look[key]
        rows.append((
            key,
            len(v),
            _pct(v, 0.5),
            _pct(v, 0.9),
            max(v),
            sum(x > _STEER_HIGH_THRESHOLD for x in v) / len(v)
        ))
    if rows:
        print_table(rows, ["lookahead", "n", "median", "p90", "max", f"frac>_{_STEER_HIGH_THRESHOLD}"])

    print("\n|steer| by corridor")
    rows = []
    for key in sorted(by_corridor):
        v = by_corridor[key]
        rows.append((key, len(v), _pct(v, 0.5), _pct(v, 0.9), max(v)))
    if rows:
        print_table(rows, ["corridor", "n", "median", "p90", "max"])

    # Rate-limit pressure: how often did the command move by the full allowance?
    steers = [(t, snap.commanded_steering_norm) for t, snap in rows if isinstance(snap.commanded_steering_norm, (int, float))]
    deltas = []
    for (t0, s0), (t1, s1) in zip(steers, steers[1:]):
        dt = t1 - t0
        if _DELTA_TIME_MIN_S < dt < _DELTA_TIME_MAX_S:
            deltas.append(abs(s1 - s0) / dt)
    if deltas:
        print(
            f"\nsteering slew (norm/s): median={_pct(deltas, 0.5):.2f} p90={_pct(deltas, 0.9):.2f} "
            f"max={max(deltas):.2f}"
        )

    # Corridor-width belief over time, sampled per lap.
    print(f"\ncorridor belief (N/S/E/W, m) and width belief, sampled every ~{_BELIEF_SAMPLE_INTERVAL_S:.0f}s")
    next_t = 0.0
    for t, snap in rows:
        if t < next_t:
            continue
        next_t = t + _BELIEF_SAMPLE_INTERVAL_S

        def g(k: str, snap=snap) -> str:
            v = getattr(snap, k)
            return f"{v:5.2f}" if isinstance(v, (int, float)) else " None"

        print(
            f"  {t:6.1f}s lap={snap.laps_completed} corr={snap.current_corridor!s:6} "
            f"N/S/E/W={g('belief_north_m')}/{g('belief_south_m')}/{g('belief_east_m')}/{g('belief_west_m')} "
            f"width={g('corridor_width_belief_m')} xtrack={g('crosstrack_error_m')} "
            f"turn={g('path_turn_ahead_rad')} "
            f"look={g('lookahead_distance_m')} steer={g('commanded_steering_norm')} "
            f"spd={g('commanded_speed_mps')}"
        )

    # Which signal actually armed the short lookahead. Before 2026-08-06 only
    # crosstrack could, and it cannot rise until the corner is already missed;
    # a healthy run should show the turn preview arming most of them.
    short = [
        (t, snap)
        for t, snap in rows
        if snap.lookahead_distance_m == _SHORT_LOOKAHEAD_M and isinstance(snap.crosstrack_error_m, (int, float))
    ]
    if short:
        by_turn = sum(1 for _, snap in short if (snap.path_turn_ahead_rad or 0.0) > _TURN_THRESHOLD_RAD)
        print(
            f"\nshort-lookahead ticks: {len(short)}  armed by turn preview: {by_turn} "
            f"({by_turn / len(short):.0%})  by crosstrack alone: {len(short) - by_turn}"
        )

    # Speed headroom.
    speeds = [snap.commanded_speed_mps for _, snap in rows if isinstance(snap.commanded_speed_mps, (int, float))]
    if speeds:
        print(
            f"\ncommanded speed (m/s): median={_pct(speeds, 0.5):.3f} p90={_pct(speeds, 0.9):.3f} "
            f"max={max(speeds):.3f}  frac<{_SPEED_LOW_THRESHOLD_MPS}={sum(s < _SPEED_LOW_THRESHOLD_MPS for s in speeds) / len(speeds):.3f}"
        )


if __name__ == "__main__":
    main()
