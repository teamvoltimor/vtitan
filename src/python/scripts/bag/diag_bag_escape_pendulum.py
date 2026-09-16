r"""When the escape goes forward-back-forward, does it ACCUMULATE or cancel?

Operator, 2026-09-15: *"when it evades an obstacle, sometimes it does a pendulum
movement going and reversing back toward the obstacle."*

``diag_bag_bay_ratchet`` already states the mechanism for the bay, and it is
the same physics here: a reversal accumulates only if the STEERING REVERSES
WITH THE DIRECTION. Drive forward turning one way, back up turning the other,
and both legs rotate the chassis the same way. Hold the steering across the
reversal and the reverse undoes exactly what the forward leg did -- which is
what a pendulum is.

``obstacles_escape_mirrors_reverse`` ships TRUE, so the mirroring is supposed to
be happening. This asks whether it does, on the open track rather than in the
bay, and what it buys when it does.

Per adjacent leg PAIR (one reversal of commanded speed) it reports:

* **mirrored** -- did the commanded steering flip sign across the reversal?
* **cancellation** -- net displacement across the pair over the path length
  travelled. 0.0 is a perfect pendulum (came back exactly); 1.0 is a straight
  line. This is the number the operator is describing.
* **yaw kept** -- net heading change over absolute heading turned, which is
  ``diag_bag_bay_ratchet``'s own metric. 1.0 is a true k-turn where both legs
  rotate the same way; 0.0 is a pendulum that gives back every degree. This is
  the better of the two axes: a pair can translate a little and still have
  wasted all its rotation.
RETRACTED 2026-09-15: this reported a "closed on the threat" column built on
``min_lidar_range_m``, which is ``min(scan.ranges_m)`` -- the RAW, unmasked
sweep minimum, i.e. the chassis. Measured over two rounds it spans 0.006-0.018 m
and sits below ``min_valid_range_m`` (0.044) on 100% of ticks, so it never
carries obstacle range at all. Comparing it across a pair was a coin flip, and
it duly read 41-53%. The column is gone. Use the pose-based axes below; they
measure what they claim.

CONTROLS, because a crashed diagnostic in this repo exits 0:

* Legs shorter than ``--min-leg`` ticks are dropped. A one-tick sign flicker
  from PWM noise is not a reversal, and counting it would invent pendulums.
* A pair counts only if at least one of its two legs contains a latched
  manoeuvre, rather than requiring BOTH to. This matters and it was got wrong
  first: 3,147 of 3,195 manoeuvre ticks on run_20260915_102714 are REVERSE, so
  the escape is almost entirely the backward half. The forward half of the
  pendulum is ORDINARY DRIVING. Filtering to manoeuvre ticks deletes it and
  reports 4 reversals in a round that had 166 escapes.
* The mirrored rate is reported beside the cancellation. If mirroring is high
  AND cancellation is low, the mirror is firing and not helping, which is a
  different bug from the mirror not firing.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_escape_pendulum.py RUN_DIR...
"""

from __future__ import annotations

import math
import sys
from itertools import pairwise
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

# isort: off
# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows, posed_rows  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

# isort: on

_MOVING = 0.01  # below this the chassis is not travelling, so it has no leg
_STEERING = 0.05  # below this the wheel is effectively centred, so it has no side
_MIN_PATH_M = 1e-3  # a pair that travelled less than this has no meaningful ratio
_GAVE_BACK = 0.25  # cancellation at or below this gave back three quarters
_MIN_TURN_RAD = math.radians(2.0)  # below this the pair barely rotated, so a ratio is noise


def _legs(rows, min_leg: int) -> list[dict]:
    """Split a run into travel legs of constant speed SIGN.

    Every travelling tick is kept, whether or not a manoeuvre is latched: the
    forward half of a pendulum is ordinary driving, so filtering to manoeuvre
    ticks deletes half of every cycle.
    """
    legs: list[dict] = []
    cur: list = []
    sign = 0
    for ts, s in rows:
        v = s.commanded_speed_mps
        if not isinstance(v, (int, float)) or abs(v) < _MOVING:
            continue
        this = 1 if v > 0 else -1
        if this != sign and cur:
            legs.append(_leg(cur, sign))
            cur = []
        sign = this
        cur.append((ts, s))
    if cur:
        legs.append(_leg(cur, sign))
    return [lg for lg in legs if lg["ticks"] >= min_leg]


def _leg(samples, sign: int) -> dict:
    """Summarise one constant-direction leg."""
    steers = [s.commanded_steering_norm for _, s in samples if isinstance(s.commanded_steering_norm, (int, float))]
    xs = [s.pose_x for _, s in samples]
    ys = [s.pose_y for _, s in samples]
    path = sum(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1))
    return {
        "sign": sign,
        "maneuver": any(s.active_maneuver_type is not None for _, s in samples),
        "ticks": len(samples),
        "t0": samples[0][0],
        "t1": samples[-1][0],
        "steer": float(np.median(steers)) if steers else 0.0,
        "x0": xs[0],
        "y0": ys[0],
        "x1": xs[-1],
        "y1": ys[-1],
        "path": path,
        "yaws": [s.pose_yaw for _, s in samples if isinstance(s.pose_yaw, (int, float))],
    }


def _wrap(a: float) -> float:
    """``a`` folded into (-pi, pi], so a heading difference never reads as a full turn."""
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_kept(a: dict, b: dict) -> float:
    """Net heading change over absolute heading turned across the pair."""
    yaws = a["yaws"] + b["yaws"]
    if len(yaws) < 2:
        return float("nan")
    turned = sum(abs(_wrap(yaws[i + 1] - yaws[i])) for i in range(len(yaws) - 1))
    if turned < _MIN_TURN_RAD:
        return float("nan")
    return abs(_wrap(yaws[-1] - yaws[0])) / turned


def _score(bag_dir: Path, min_leg: int, all_pairs: bool) -> list:
    """One row: how many reversals mirrored, and how much they cancelled."""
    rows, _ = load_nav_debug_rows(bag_dir)
    legs = _legs(posed_rows(rows), min_leg)
    mirrored = 0
    cancels: list[float] = []
    kept: list[float] = []
    pairs = 0
    # Split by whether the pair stays INSIDE the escape or crosses back out to
    # the planner. The mirror is the escape's own command; the planner's
    # forward leg is chosen by pursuit and knows nothing about it, so a low
    # mirrored rate on crossing pairs is a HANDOFF failure rather than a bug in
    # the mirror.
    inside = inside_mirrored = crossing = crossing_mirrored = 0
    for a, b in pairwise(legs):
        if a["sign"] == b["sign"]:
            continue
        if not all_pairs and not (a["maneuver"] or b["maneuver"]):
            continue
        pairs += 1
        # Mirrored: the wheel took the OTHER side across the reversal. Both
        # legs then rotate the chassis the same way and the pair accumulates.
        is_mirrored = (
            abs(a["steer"]) > _STEERING and abs(b["steer"]) > _STEERING and (a["steer"] > 0) != (b["steer"] > 0)
        )
        mirrored += int(is_mirrored)
        if a["maneuver"] and b["maneuver"]:
            inside += 1
            inside_mirrored += int(is_mirrored)
        else:
            crossing += 1
            crossing_mirrored += int(is_mirrored)
        path = a["path"] + b["path"]
        if path > _MIN_PATH_M:
            cancels.append(math.hypot(b["x1"] - a["x0"], b["y1"] - a["y0"]) / path)
        k = _yaw_kept(a, b)
        if not math.isnan(k):
            kept.append(k)
    if pairs == 0:
        return [bag_dir.name.replace("run_", ""), 0, "-", "-", "-", "-", "-", "-", "-", "-"]
    c = np.array(cancels) if cancels else np.array([float("nan")])
    k = np.array(kept) if kept else np.array([float("nan")])
    return [
        bag_dir.name.replace("run_", ""),
        pairs,
        f"{mirrored} ({100 * mirrored / pairs:.0f}%)",
        f"{np.nanpercentile(c, 50):.2f}",
        f"{np.nanpercentile(c, 90):.2f}",
        f"{100 * float(np.nanmean(c < _GAVE_BACK)):.0f}%",
        f"{np.nanpercentile(k, 50):.2f}",
        f"{100 * float(np.nanmean(k < _GAVE_BACK)):.0f}%",
        f"{inside_mirrored}/{inside}" + (f" ({100 * inside_mirrored / inside:.0f}%)" if inside else ""),
        f"{crossing_mirrored}/{crossing}" + (f" ({100 * crossing_mirrored / crossing:.0f}%)" if crossing else ""),
    ]


def main() -> int:
    """Report the mirroring rate and the cancellation it does or does not buy."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--min-leg", type=int, default=3, help="ticks a leg needs to count as a reversal")
    parser.add_argument("--all-pairs", action="store_true", help="count reversals with no manoeuvre in either leg too")
    args = parser.parse_args()

    rows = [_score(bag_dir, args.min_leg, args.all_pairs) for bag_dir in args.bag_dirs]
    print_table(
        rows,
        [
            "run",
            "reversals",
            "MIRRORED",
            "cancel p50",
            "cancel p90",
            "cancel < 0.25",
            "YAW KEPT p50",
            "kept < 25%",
            "inside escape",
            "CROSSING out",
        ],
    )
    print()
    print(
        "cancellation = net displacement / path travelled across the pair.\n"
        "0.0 is a perfect pendulum, 1.0 is a straight line. 'cancel < 0.25' is the\n"
        "share of reversals that gave back three quarters of what they travelled.\n"
        "MIRRORED high WITH cancellation low means the mirror fires and does not help,\n"
        "which is a different bug from the mirror not firing."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
