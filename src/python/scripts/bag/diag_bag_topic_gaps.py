r"""Did a sensor or a node STOP PUBLISHING during the round?

The first question to ask about a bad round, and the cheapest -- because if a
topic went silent for two seconds, every downstream explanation is wasted work.
It is also the question most likely to be answered by assumption: the Pi
throttles at 82.5 C with the fan maxed and ``vision_node`` eats a whole core, so
"the camera must have dropped out" is an easy story to believe without checking.

On the fifteen 2026-09-15 Obstacles rounds the answer was NO, uniformly: zero
gaps over 0.5 s on ``/scan``, ``/vision/detections``, ``/imu/data`` and
``/motor/drive_speed`` in every bag. That negative result is what let the whole
session be attributed to geometry and tracking instead. Ruling the sensors out
is worth one cheap pass over the bag.

The one thing that DOES freeze is ``/nav_debug``, with 1.8-2.0 s gaps in the
first ~30 s while the bay exit runs, up to twelve of them in a bad round. That
is the navigator's own publishing stalling, not a sensor, and it matters chiefly
because every other diagnostic in this folder samples the world through
``/nav_debug`` -- a gap there is a hole in the EVIDENCE, not necessarily in the
robot.

METHOD

One replay pass. For every topic, the inter-message interval; report the count
of gaps above ``--threshold``, the largest, and when the worst few happened
(seconds from the start of the bag, so they line up with every other script's
elapsed-time axis).

TRAP: a topic that never published at all has NO gaps and reads clean here. The
message COUNT is printed next to the gap count for exactly that reason -- a
count of zero is the finding, and it looks nothing like a dropout.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_topic_gaps.py RUN_DIR...
"""

from __future__ import annotations

import argparse

from scripts.common.bag_io import Topics, create_bags_parser, elapsed_seconds, open_reader

_WATCHED = (
    Topics.SCAN,
    Topics.NAV_DEBUG,
    Topics.VISION_DETECTIONS,
    Topics.IMU_DATA,
    Topics.MOTOR_DRIVE_SPEED,
)
"""The topics a round cannot survive losing, plus ``/nav_debug`` as the evidence channel.

Other topics are still reported when they gap; these are the ones reported even
when they do not, so that a silent topic shows up as ``n=0``.
"""


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--threshold", type=float, default=0.5, help="Gap size to report, seconds (default 0.5)")
    parser.add_argument("--worst", type=int, default=4, help="How many worst gaps to timestamp (default 4)")
    args = parser.parse_args()

    for bag_dir in args.bag_dirs:
        reader = open_reader(bag_dir)
        t0: int | None = None
        last: dict[str, int] = {}
        counts: dict[str, int] = {}
        gaps: dict[str, list[tuple[float, float]]] = {}
        while reader.has_next():
            topic, _data, t = reader.read_next()
            if t0 is None:
                t0 = t
            counts[topic] = counts.get(topic, 0) + 1
            if topic in last:
                gap = elapsed_seconds(t, last[topic])
                if gap > args.threshold:
                    gaps.setdefault(topic, []).append((elapsed_seconds(last[topic], t0), gap))
            last[topic] = t

        if t0 is None:
            print(f"\n=== {bag_dir.name}: empty bag")
            continue
        duration = elapsed_seconds(max(last.values()), t0)
        print(f"\n=== {bag_dir.name}  dur={duration:.0f}s  gaps >{args.threshold}s")

        reported = list(_WATCHED) + sorted(set(gaps) - set(_WATCHED))
        for topic in reported:
            found = gaps.get(topic, [])
            n = counts.get(topic, 0)
            rate = f"{n / duration:.1f} Hz" if duration > 0 else "-"
            if n == 0:
                print(f"  {topic:26} n=0  NEVER PUBLISHED")
                continue
            if not found:
                print(f"  {topic:26} n={n:<6} {rate:>8}  clean")
                continue
            worst = sorted(found, key=lambda g: -g[1])[: args.worst]
            when = ", ".join(f"{gap:.1f}s@{at:.0f}s" for at, gap in worst)
            print(f"  {topic:26} n={n:<6} {rate:>8}  {len(found)} gaps  worst: {when}")

    print(
        "\nA topic reading n=0 never published and cannot gap -- that is a finding, not a clean run.\n"
        "Gaps on /nav_debug are holes in the EVIDENCE every other diag script reads, not proof the\n"
        "robot stopped; gaps on the sensor topics are the robot."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
