r"""Does the escape counter ever ESCALATE, or does it reset itself first?

``escape.toml`` sets ``escalate_after_attempts = 3``: after three escapes from
the same spot the recovery is supposed to stop repeating a leg that is not
working and switch to the escalated manoeuvre. Every tuning discussion of the
escape loop has assumed that ladder exists. It does not run.

``navigator.py`` clears ``_escape_count`` on the first normal-drive tick whose
pose is at least ``stuck_move_threshold`` (0.03 m) from
``_escape_sequence_start_xy``.

Read that alone and the obvious conclusion is that the escape's own reverse leg
resets the counter, since one leg backs off about 0.10 m. That conclusion is
WRONG, and the code says so: ``escape_recovery.py`` re-anchors
``_escape_sequence_start_xy`` to where each manoeuvre ENDED, precisely so an
escape cannot certify itself as having worked with its own travel (adr:0055).
The threshold is measured from the end of the escape, not from its latch.

What actually happens is smaller and harder to argue with. 3 cm of travel AFTER
the escape ends is about a fifth of a second of ordinary driving, and the loop's
shape is not "sit still and retry" -- it is escape, drive away, come back,
escape again. Every one of those round trips clears the counter on its way out.
Reaching attempt three would need three escapes separated by under 3 cm of
progress, which is a robot pinned motionless, not a robot looping.

So the ladder is not mis-tuned. It is guarding a failure mode (the wedged robot)
that is not the failure mode the rounds actually die of (the robot that escapes
and returns).

The measured census of these triggers, the post-escape reset travel, and the
K-turn caveat that goes with acting on it are recorded in adr:0055.

WHAT THIS PRINTS

* the latch histogram -- ``escape_count`` on each tick the counter increments,
  which is the tick an escape is latched;
* the travel between the END of the escape manoeuvre and the tick the counter
  falls back to zero, against ``stuck_move_threshold``, plus how long that took.
  This is the mechanism rather than the symptom, and it is deliberately measured
  from the same anchor production uses: measuring from the latch instead
  reproduces the wrong explanation above and makes the guard look broken.

The pose here is the localizer's, which is what the navigator itself compares
against ``_escape_sequence_start_xy`` -- so a diverged localizer moves this
measurement and the production guard identically, and the comparison stays
honest even on a run whose pose is wrong.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        pixi run -e dev python scripts/bag/diag_bag_escape_counter.py RUN_DIR...
"""

from __future__ import annotations

import argparse
import math
import statistics
from collections import Counter

from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows


def main() -> int:
    parser = create_bags_parser(__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    args = parser.parse_args()

    escape = NavigationTuning.load_default().escape
    threshold = escape.stuck_move_threshold
    escalate = escape.escalate_after_attempts
    print(f"stuck_move_threshold = {threshold} m   escalate_after_attempts = {escalate}")

    for bag_dir in args.bag_dirs:
        rows, _topics = load_nav_debug_rows(bag_dir)
        hist: Counter[int] = Counter()
        resets: list[tuple[float, float]] = []
        previous: int | None = None
        anchor: tuple[float, float] | None = None
        anchor_ts: float | None = None

        for ts, snap in rows:
            count = snap.escape_count
            if count is None:
                continue
            here = (snap.pose_x, snap.pose_y) if snap.pose_x is not None and snap.pose_y is not None else None
            if previous is not None and count > previous:
                hist[count] += 1
            if count and snap.active_maneuver_type is not None and here is not None:
                # Production re-anchors the sequence at the END of each escape
                # manoeuvre, so the anchor is the LAST manoeuvre tick of the
                # sequence, not the tick the escape was latched.
                anchor, anchor_ts = here, ts
            elif previous and count == 0 and anchor is not None and here is not None:
                resets.append((math.dist(here, anchor), ts - (anchor_ts or ts)))
                anchor = None
            previous = count

        triggers = sum(hist.values())
        if not triggers:
            print(f"\n=== {bag_dir.name}: {len(rows)} nav_debug ticks, no escape latched")
            continue

        reached = sum(n for value, n in hist.items() if value > escalate)
        spread = " ".join(f"{value}:{n}" for value, n in sorted(hist.items()))
        print(f"\n=== {bag_dir.name}  ticks={len(rows)} triggers={triggers}")
        print(f"  escape_count at latch: {spread}")
        print(f"  attempt 1 share: {100 * hist[1] / triggers:.0f}%   past escalate_after_attempts: {reached}")
        if resets:
            distances = [d for d, _dt in resets]
            seconds = [dt for _d, dt in resets]
            print(
                f"  post-escape travel at reset n={len(resets)} "
                f"p50={statistics.median(distances):.3f} m  min={min(distances):.3f}  max={max(distances):.3f}  "
                f"(threshold {threshold} m)"
            )
            print(
                f"  seconds from the escape ending to the reset: "
                f"p50={statistics.median(seconds):.2f} s  max={max(seconds):.2f} s"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
