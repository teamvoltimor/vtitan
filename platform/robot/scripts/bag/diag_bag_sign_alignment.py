r"""Does the robot ACHIEVE a good sign offset and then correct it away?

Reported from the track on 2026-09-07: the robot repeatedly lined up to pass a
sign on the correct side, kept correcting, and arrived badly placed. That is a
different failure from "never got there" -- and the two call for opposite fixes,
so it is worth separating on recorded data before touching the controller.

For every commitment (`committed_sign_x_m`/`_y_m` held on a sign) this reports:

* **best lateral** -- the largest lateral offset from the sign, measured in the
  robot's own frame, reached at any point while closing from 1.2 m to the
  closest approach. This is the best alignment the run ever had.
* **final lateral** -- the lateral offset actually achieved at closest approach.
* **given back** -- best minus final. Large values ARE the reported behaviour:
  the robot was well placed and then steered out of it.
* **steering flips** -- sign changes of `commanded_steering_norm` during the
  approach, the direct symptom of a controller hunting rather than holding.

Lateral is signed in the robot frame (positive = sign to the LEFT, so the robot
passes to its right) and reported as magnitude for the offset stats; the SIDE it
settles on is a separate question this script does not judge, because colour is
not in the snapshot.

A pass needs ~0.122 m of lateral clearance (chassis half-width 0.097 + sign
half-width 0.025) to be contact-free.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_alignment.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

CONTACT_GAP_M = 0.122
"""Chassis half-width plus sign half-width: below this the pass is a contact."""

APPROACH_START_M = 1.2
"""Only ticks closer than this count as the approach, so a sign held from far
away does not drag its whole cross-corridor history into the statistics."""

SAME_SIGN_TOL_M = 0.15
"""Commitment breaks when the committed point jumps further than this."""


@dataclass
class Approach:
    """One commitment, reduced to the numbers the question needs."""

    run: str
    best_lateral_m: float
    final_lateral_m: float
    closest_range_m: float
    flips: int
    ticks: int

    @property
    def given_back_m(self) -> float:
        """How much of the best alignment was steered away before the pass."""
        return self.best_lateral_m - self.final_lateral_m


def _lateral_of(sign_x: float, sign_y: float, x: float, y: float, yaw: float) -> float:
    """Sign's lateral offset in the robot frame; positive means to the LEFT."""
    dx, dy = sign_x - x, sign_y - y
    return -dx * math.sin(yaw) + dy * math.cos(yaw)


def _approaches(run: str, rows: list) -> list[Approach]:
    """Split one run's ticks into commitments and reduce each to an Approach."""
    out: list[Approach] = []
    current: list[tuple[float, float, float]] = []  # (range, |lateral|, steering)
    anchor: tuple[float, float] | None = None

    def flush() -> None:
        if len(current) < 3:
            return
        closest_i = min(range(len(current)), key=lambda i: current[i][0])
        window = current[: closest_i + 1]
        if not window:
            return
        flips = sum(
            1
            for a, b in zip(window, window[1:])
            if a[2] * b[2] < 0 and abs(a[2]) > 0.02 and abs(b[2]) > 0.02
        )
        out.append(
            Approach(
                run=run,
                best_lateral_m=max(w[1] for w in window),
                final_lateral_m=window[-1][1],
                closest_range_m=current[closest_i][0],
                flips=flips,
                ticks=len(window),
            )
        )

    for _, d in rows:
        sx, sy = d.committed_sign_x_m, d.committed_sign_y_m
        if sx is None or sy is None:
            flush()
            current, anchor = [], None
            continue
        if anchor is not None and math.hypot(sx - anchor[0], sy - anchor[1]) > SAME_SIGN_TOL_M:
            flush()
            current = []
        anchor = (sx, sy)
        rng = math.hypot(sx - d.pose_x, sy - d.pose_y)
        if rng > APPROACH_START_M:
            continue
        lateral = abs(_lateral_of(sx, sy, d.pose_x, d.pose_y, d.pose_yaw))
        current.append((rng, lateral, d.commanded_steering_norm or 0.0))
    flush()
    return out


def main() -> None:
    parser = create_bags_parser(__doc__, formatter_class=__import__("argparse").RawDescriptionHelpFormatter)
    args = parser.parse_args()

    approaches: list[Approach] = []
    for bag in args.bag_dirs:
        rows, _ = load_nav_debug_rows(str(bag))
        approaches.extend(_approaches(Path(bag).name.replace("run_", ""), rows))

    if not approaches:
        print("No sign commitments found in these bags.")
        return

    rows_out = []
    for run in sorted({a.run for a in approaches}):
        group = [a for a in approaches if a.run == run]
        rows_out.append(
            [
                run,
                len(group),
                round(statistics.median(a.best_lateral_m for a in group), 3),
                round(statistics.median(a.final_lateral_m for a in group), 3),
                round(statistics.median(a.given_back_m for a in group), 3),
                round(statistics.median(a.flips for a in group), 1),
            ]
        )
    print(f"== SIGN ALIGNMENT  ({len(approaches)} approaches)")
    print_table(rows_out, ["run", "signs", "med best m", "med final m", "med given back", "med flips"])

    best = sorted(a.best_lateral_m for a in approaches)
    final = sorted(a.final_lateral_m for a in approaches)
    back = sorted(a.given_back_m for a in approaches)
    pct = lambda xs, p: xs[int(len(xs) * p)]  # noqa: E731
    print(f"  best lateral m:   p10 {pct(best, .1):.3f} / median {pct(best, .5):.3f} / p90 {pct(best, .9):.3f}")
    print(f"  final lateral m:  p10 {pct(final, .1):.3f} / median {pct(final, .5):.3f} / p90 {pct(final, .9):.3f}")
    print(f"  given back m:     p10 {pct(back, .1):.3f} / median {pct(back, .5):.3f} / p90 {pct(back, .9):.3f}")

    was_clear = [a for a in approaches if a.best_lateral_m >= CONTACT_GAP_M]
    lost_it = [a for a in was_clear if a.final_lateral_m < CONTACT_GAP_M]
    print()
    print(f"  reached a contact-free offset at some point: {len(was_clear)}/{len(approaches)}")
    print(f"  ...and then gave it back into contact:       {len(lost_it)}/{len(was_clear)}")
    never = len(approaches) - len(was_clear)
    print(f"  never reached a contact-free offset at all:  {never}/{len(approaches)}")
    if lost_it:
        print(f"  median flips when it gave it back: {statistics.median(a.flips for a in lost_it):.1f}")
    if was_clear:
        held = [a for a in was_clear if a.final_lateral_m >= CONTACT_GAP_M]
        if held:
            print(f"  median flips when it HELD:         {statistics.median(a.flips for a in held):.1f}")


if __name__ == "__main__":
    main()
