"""One row per bag: which CHALLENGE it was, and how wide the corridor it saw.

`diag_bag_session_inventory.py` answers "how did this run end". It cannot
answer "was this an Open run or an Obstacles run", and on a 55-run
competition day that split is the first thing every later question needs.
The discriminator is the sign router: a bag whose `active_sign_count` is
never positive and whose /vision/detections never carries a pillar is Open.

The corridor width is reported too, because the 2026-09-12 Open track used
1 m x 1 m inner walls (corridor 1.00 m) instead of the narrow layout the
tuning was fitted on, and every zig-zag question is conditional on which
one the robot was actually driving.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_session_classify.py data/live/runs --prefix run_20260912
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader
from scripts.common.stats import mean, median


def summarize(bag_dir: Path) -> dict[str, object]:
    reader = open_reader(bag_dir)
    t0 = None
    last_ts = 0.0
    widths: list[float] = []
    start_widths: list[float] = []
    sign_ticks = 0
    nav_ticks = 0
    max_signs = 0
    detection_frames = 0
    detection_colours: dict[str, int] = {}
    laps = 0
    num_laps = 0
    direction = None
    escapes = 0
    crosstrack: list[float] = []
    steering: list[float] = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        last_ts = elapsed_seconds(t, t0)

        if topic == Topics.NAV_DEBUG:
            snap = decode_nav_debug(data)
            nav_ticks += 1
            if snap.corridor_width_belief_m is not None:
                widths.append(snap.corridor_width_belief_m)
            if snap.start_measured_corridor_width_m is not None:
                start_widths.append(snap.start_measured_corridor_width_m)
            count = snap.active_sign_count or 0
            if count > 0:
                sign_ticks += 1
            max_signs = max(max_signs, count)
            laps = max(laps, snap.laps_completed or 0)
            num_laps = max(num_laps, snap.num_laps or 0)
            if snap.direction is not None:
                direction = str(getattr(snap.direction, "value", snap.direction))
            escapes = max(escapes, snap.escape_count or 0)
            if snap.crosstrack_error_m is not None:
                crosstrack.append(snap.crosstrack_error_m)
            if snap.commanded_steering_norm is not None:
                steering.append(snap.commanded_steering_norm)

        elif topic == Topics.VISION_DETECTIONS:
            payload = json.loads(deserialize_message(data, String).data)
            if not payload:
                continue
            detection_frames += 1
            for record in payload:
                name = str(record.get("class_name") or record.get("class") or "?").lower()
                detection_colours[name] = detection_colours.get(name, 0) + 1

    median_width = median(widths) if widths else None
    start_width = median(start_widths) if start_widths else None
    pillars = sum(n for name, n in detection_colours.items() if "red" in name or "green" in name)
    challenge = "obstacles" if (max_signs > 0 or pillars > 0) else "open"

    # Zig-zag proxy: how often the commanded wheel reverses sign tick to tick.
    reversals = 0
    for prev, cur in zip(steering, steering[1:]):
        if prev * cur < 0 and abs(prev) > 0.05 and abs(cur) > 0.05:
            reversals += 1
    reversal_rate = reversals / max(1, len(steering) - 1)

    return {
        "run": bag_dir.name,
        "challenge": challenge,
        "dur": last_ts,
        "ticks": nav_ticks,
        "laps": laps,
        "num_laps": num_laps,
        "dir": (direction or "-")[:4],
        "esc": escapes,
        "signs_max": max_signs,
        "sign_ticks": sign_ticks,
        "pillars": pillars,
        "width": median_width,
        "start_width": start_width,
        "xtrack_abs": mean(abs(v) for v in crosstrack) if crosstrack else None,
        "steer_abs": mean(abs(v) for v in steering) if steering else None,
        "rev_rate": reversal_rate,
    }


def _fmt(value: object, spec: str = "") -> str:
    if value is None:
        return "-"
    return format(value, spec)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--prefix", default="run_")
    args = parser.parse_args()

    bags = sorted(d for d in args.root.iterdir() if d.is_dir() and d.name.startswith(args.prefix))
    header = (
        f"{'run':<24}{'chal':<11}{'dur':>7}{'laps':>6}{'dir':>6}{'esc':>5}"
        f"{'sgn':>5}{'sgnTk':>7}{'pill':>7}{'width':>8}{'startW':>8}{'|xt|':>7}{'|st|':>7}{'rev%':>7}"
    )
    print(header)
    print("-" * len(header))
    for bag in bags:
        try:
            row = summarize(bag)
        except Exception as exc:  # noqa: BLE001 - one bad bag must not hide 58 good ones
            print(f"{bag.name:<24}FAILED {exc}")
            continue
        print(
            f"{row['run']:<24}{row['challenge']:<11}{row['dur']:>7.1f}"
            f"{str(row['laps']) + '/' + str(row['num_laps']):>6}{row['dir']:>6}{row['esc']:>5}"
            f"{row['signs_max']:>5}{row['sign_ticks']:>7}{row['pillars']:>7}"
            f"{_fmt(row['width'], '>8.3f')}{_fmt(row['start_width'], '>8.3f')}"
            f"{_fmt(row['xtrack_abs'], '>7.3f')}{_fmt(row['steer_abs'], '>7.3f')}"
            f"{row['rev_rate'] * 100:>7.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
