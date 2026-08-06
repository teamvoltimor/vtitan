"""Ask whether the pursuit controller is leaving steering authority unused.

The visual symptom is a robot that rounds a wide corner with a gentle arc when
it had lock to spare. This prints, per lap and per corridor, how much of the
steering range was actually commanded, which lookahead was selected, and what
the corridor-width belief was at the time -- so a shallow arc can be traced to
a long lookahead, to a rate limit, or to the plan itself asking for little.

Usage:
    pixi run -e dev python scripts/diag_bag_steer_headroom.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def _pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    rows: list[dict] = []
    states: list[tuple[float, str]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        ts = (t - t_start) / 1e9
        if topic == "/robot_state":
            msg = deserialize_message(data, String)
            if not states or states[-1][1] != msg.data:
                states.append((ts, msg.data))
        elif topic == "/nav_debug":
            payload = json.loads(deserialize_message(data, String).data)
            payload["_t"] = ts
            rows.append(payload)

    print(f"== {args.bag_dir.name} ==")
    print("state transitions: " + ", ".join(f"{t:.1f}s->{s}" for t, s in states))
    if not rows:
        print("no /nav_debug samples")
        return
    print(f"nav_debug samples: {len(rows)}  span {rows[0]['_t']:.1f}s..{rows[-1]['_t']:.1f}s")

    # Lap boundaries as reported by the navigator itself.
    lap_marks = []
    prev = None
    for r in rows:
        lap = r.get("laps_completed")
        if lap != prev:
            lap_marks.append((r["_t"], prev, lap))
            prev = lap
    print("lap counter: " + ", ".join(f"{t:.1f}s {a}->{b}" for t, a, b in lap_marks))

    # Steering headroom, bucketed by which lookahead the controller selected.
    by_look: dict[str, list[float]] = defaultdict(list)
    by_corridor: dict[str, list[float]] = defaultdict(list)
    for r in rows:
        steer = r.get("commanded_steering_norm")
        look = r.get("lookahead_distance_m")
        if not isinstance(steer, (int, float)) or not isinstance(look, (int, float)):
            continue
        by_look[f"look={look:.2f}"].append(abs(steer))
        corridor = str(r.get("current_corridor"))
        by_corridor[corridor].append(abs(steer))

    print("\n|steer| by selected lookahead (norm, 1.0 = full lock)")
    for key in sorted(by_look):
        v = by_look[key]
        print(
            f"  {key:12} n={len(v):5}  median={_pct(v, 0.5):.3f}  p90={_pct(v, 0.9):.3f}  "
            f"max={max(v):.3f}  frac|steer|>0.8={sum(x > 0.8 for x in v) / len(v):.3f}"
        )

    print("\n|steer| by corridor")
    for key in sorted(by_corridor):
        v = by_corridor[key]
        print(
            f"  {key:12} n={len(v):5}  median={_pct(v, 0.5):.3f}  p90={_pct(v, 0.9):.3f}  max={max(v):.3f}"
        )

    # Rate-limit pressure: how often did the command move by the full allowance?
    steers = [
        (r["_t"], r["commanded_steering_norm"])
        for r in rows
        if isinstance(r.get("commanded_steering_norm"), (int, float))
    ]
    deltas = []
    for (t0, s0), (t1, s1) in zip(steers, steers[1:]):
        dt = t1 - t0
        if 0.01 < dt < 0.5:
            deltas.append(abs(s1 - s0) / dt)
    if deltas:
        print(
            f"\nsteering slew (norm/s): median={_pct(deltas, 0.5):.2f} p90={_pct(deltas, 0.9):.2f} "
            f"max={max(deltas):.2f}"
        )

    # Corridor-width belief over time, sampled per lap.
    print("\ncorridor belief (N/S/E/W, m) and width belief, sampled every ~20s")
    next_t = 0.0
    for r in rows:
        if r["_t"] < next_t:
            continue
        next_t = r["_t"] + 20.0

        def g(k: str) -> str:
            v = r.get(k)
            return f"{v:5.2f}" if isinstance(v, (int, float)) else " None"

        print(
            f"  {r['_t']:6.1f}s lap={r.get('laps_completed')} corr={str(r.get('current_corridor')):6} "
            f"N/S/E/W={g('belief_north_m')}/{g('belief_south_m')}/{g('belief_east_m')}/{g('belief_west_m')} "
            f"width={g('corridor_width_belief_m')} xtrack={g('crosstrack_error_m')} "
            f"look={g('lookahead_distance_m')} steer={g('commanded_steering_norm')} "
            f"spd={g('commanded_speed_mps')}"
        )

    # Speed headroom.
    speeds = [
        r["commanded_speed_mps"]
        for r in rows
        if isinstance(r.get("commanded_speed_mps"), (int, float))
    ]
    if speeds:
        print(
            f"\ncommanded speed (m/s): median={_pct(speeds, 0.5):.3f} p90={_pct(speeds, 0.9):.3f} "
            f"max={max(speeds):.3f}  frac<0.10={sum(s < 0.10 for s in speeds) / len(speeds):.3f}"
        )


if __name__ == "__main__":
    main()
