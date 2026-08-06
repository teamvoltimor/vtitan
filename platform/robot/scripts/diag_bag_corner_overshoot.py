"""Show what the pursuit controller does through each corner of a race bag.

A corner is taken as a ``current_corridor`` transition. Around each one this
prints the crosstrack error, the selected lookahead and the commanded steering
in the seconds before and after -- so "it turned too lazily and ran wide" shows
up as steering that stays low while the lookahead stays long, followed by a
crosstrack spike that only then pulls the lookahead short.

Usage:
    pixi run -e dev python scripts/diag_bag_corner_overshoot.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

WINDOW_S = 4.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--max-corners", type=int, default=8)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    t_start = None
    rows: list[dict] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start is None:
            t_start = t
        if topic != "/nav_debug":
            continue
        payload = json.loads(deserialize_message(data, String).data)
        payload["_t"] = (t - t_start) / 1e9
        rows.append(payload)

    corners = []
    prev = None
    for r in rows:
        corridor = r.get("current_corridor")
        if corridor is not None and corridor != prev:
            if prev is not None:
                corners.append((r["_t"], prev, corridor))
            prev = corridor

    print(f"== {args.bag_dir.name} == {len(corners)} corridor transitions")

    def g(r: dict, k: str, spec: str = "5.2f") -> str:
        v = r.get(k)
        return format(v, spec) if isinstance(v, (int, float)) else " None"

    for t_corner, a, b in corners[: args.max_corners]:
        print(f"\n-- {a} -> {b} at {t_corner:.1f}s")
        window = [r for r in rows if abs(r["_t"] - t_corner) <= WINDOW_S]
        for r in window[::4]:
            rel = r["_t"] - t_corner
            print(
                f"   {rel:+5.1f}s xtrack={g(r, 'crosstrack_error_m')} "
                f"look={g(r, 'lookahead_distance_m')} steer={g(r, 'commanded_steering_norm')} "
                f"aerr={g(r, 'angle_error_rad')} spd={g(r, 'commanded_speed_mps')} "
                f"fwd={g(r, 'forward_clearance_m')}"
            )
        xt = [
            r["crosstrack_error_m"]
            for r in rows
            if 0 <= r["_t"] - t_corner <= 8.0
            and isinstance(r.get("crosstrack_error_m"), (int, float))
        ]
        st = [
            abs(r["commanded_steering_norm"])
            for r in window
            if isinstance(r.get("commanded_steering_norm"), (int, float))
        ]
        if xt and st:
            print(f"   => peak xtrack in the 8s after: {max(xt):.2f} m; peak |steer| through: {max(st):.2f}")


if __name__ == "__main__":
    main()
