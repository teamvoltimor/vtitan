r"""Is ``angle_error_rad`` telling the truth, and if so what is it measuring?

The heading term cuts speed to the creep floor whenever ``|angle_error_rad|``
reaches ``HeadingErrorZones.CRAWL`` (1.0 rad, 57.3 deg), and that cut binds
44-64% of an Open round. ``HeadingErrorZones`` justifies the threshold by
asserting that normal cornering lives at 23-45 deg, so CRAWL only catches real
saturation. The 2026-09-08 bags measure p50 = 65 deg, which is either

* REAL -- the robot is genuinely pointed 65 deg away from its own target most
  of the time, which is a planning failure and makes the speed cut a symptom,
  or
* an ARTEFACT of how the field is computed, in which case the cut is a tax on
  a number that does not mean what the threshold assumes.

Those want opposite fixes, so this script separates them instead of tuning on
top of the number.

``angle_error_rad`` is ``atan2(y_local, x_local)`` for the steering target in
the robot frame (``WaypointController.compute_steering``). The snapshot also
publishes the pose and ``steer_target_x/y`` in the world frame, so the whole
computation can be REDONE here from its own inputs and compared. Three
independent checks:

1. RECOMPUTE. Transform the published target into the robot frame using the
   published pose and take the bearing. A field that disagrees with its own
   inputs is a bug in the field.

2. GEOMETRY. A bearing is ``atan(lateral / forward)``, so it blows up when the
   target is CLOSE, not only when the robot is badly aimed. Report the distance
   to the target against ``lookahead_distance_m``: a target sitting far inside
   the lookahead inflates the bearing for a lateral offset that has not
   changed, and 65 deg at 4.6 cm of crosstrack needs a forward distance of
   about 2 cm.

3. STEERING CONSISTENCY. The same ``x_local, y_local`` feed
   ``pure_pursuit_steer``. If a 65 deg bearing is real and the target is
   forward, the command should be near saturation on the same tick. A large
   bearing next to a small command means the two disagree about the geometry.

Also counts the ticks whose target is BEHIND the robot (``x_local <= 0``),
where ``compute_steering`` abandons the curvature formula and saturates by
sign. Those ticks have ``|angle_error| > 90 deg`` by construction and are a
different failure from being 65 deg off a forward target.

Pose note: ``pose_yaw`` is the localizer-fused, damped estimate rather than raw
TF. That is DELIBERATE here -- the question is what the controller computed,
and the fused yaw is the input it actually used.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_angle_error.py \
        data/live/runs/run_20260908_003520 data/live/runs/run_20260908_004023
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows

CRAWL_RAD = 1.0
"""``HeadingErrorZones.CRAWL``: at or above this the heading term commands creep."""

NEAR_SATURATION = 0.9
"""``|commanded_steering_norm|`` at or above this counts as saturated."""

MIN_TURN_RADIUS_M = 0.29
"""Measured saturation radius of the chassis (``RobotSpecs.MIN_TURN_RADIUS_M``).

An aim point whose pure-pursuit circle is tighter than this cannot be driven,
however hard the servo is commanded."""


def _pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(q * len(ordered)))
    return ordered[idx]


def _describe(name: str, values: list[float], unit: str = "") -> str:
    if not values:
        return f"  {name:<26} (no ticks)"
    return (
        f"  {name:<26} n={len(values):<6} "
        f"p25={_pct(values, 0.25):7.3f} p50={_pct(values, 0.50):7.3f} "
        f"p75={_pct(values, 0.75):7.3f} p95={_pct(values, 0.95):7.3f}{unit}"
    )


def analyse(bag_dir: Path) -> None:
    rows, _counts = load_nav_debug_rows(bag_dir)

    published: list[float] = []          # |angle_error_rad|, degrees
    residual: list[float] = []           # |published - recomputed|, degrees
    forward: list[float] = []            # x_local, metres
    target_dist: list[float] = []        # |target - pose|, metres
    lookahead: list[float] = []
    dist_over_lookahead: list[float] = []
    required_radius: list[float] = []   # radius the aim point demands, metres
    behind = 0
    recomputable = 0

    big_steer: list[float] = []          # |steering| on ticks with |err| >= CRAWL
    small_steer_big_err = 0
    big_err = 0

    for _t, s in rows:
        err = s.angle_error_rad
        if not isinstance(err, (int, float)):
            continue
        published.append(math.degrees(abs(err)))

        if abs(err) >= CRAWL_RAD:
            big_err += 1
            steer = s.commanded_steering_norm
            if isinstance(steer, (int, float)):
                big_steer.append(abs(steer))
                if abs(steer) < NEAR_SATURATION:
                    small_steer_big_err += 1

        px, py, pyaw = s.pose_x, s.pose_y, s.pose_yaw
        tx, ty = s.steer_target_x, s.steer_target_y
        if not all(isinstance(v, (int, float)) for v in (px, py, pyaw, tx, ty)):
            continue
        recomputable += 1

        dx, dy = tx - px, ty - py
        cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)
        x_local = dx * cos_y + dy * sin_y
        y_local = -dx * sin_y + dy * cos_y

        residual.append(abs(math.degrees(math.atan2(y_local, x_local) - err)))
        forward.append(x_local)
        dist = math.hypot(dx, dy)
        target_dist.append(dist)
        if x_local <= 0:
            behind += 1
        # Pure pursuit's own geometry: a target at distance d and bearing a
        # lies on a circle of radius d / (2 sin a). Compared against the
        # MEASURED minimum turn radius, this says whether the aim point is
        # reachable at all -- a target the chassis cannot curve onto produces a
        # large bearing that no amount of steering will reduce, and the heading
        # speed cut then fires on it every tick.
        sin_a = abs(math.sin(math.atan2(y_local, x_local)))
        if sin_a > 1e-6 and dist > 0:
            required_radius.append(dist / (2.0 * sin_a))

        la = s.lookahead_distance_m
        if isinstance(la, (int, float)) and la > 0:
            lookahead.append(la)
            dist_over_lookahead.append(dist / la)

    print(f"\n=== {bag_dir.name} ===")
    print(f"  ticks with angle_error_rad: {len(published)}   recomputable: {recomputable}")
    if not published:
        return

    print("\n  1. IS THE FIELD CONSISTENT WITH ITS OWN INPUTS?")
    print(_describe("|published - recomputed|", residual, " deg"))
    if residual:
        agree = sum(1 for r in residual if r < 1.0)
        print(f"    within 1 deg: {agree}/{len(residual)} ({100 * agree / len(residual):.1f}%)")

    print("\n  2. WHAT IS THE GEOMETRY?")
    print(_describe("|angle_error|", published, " deg"))
    print(_describe("x_local (forward)", forward, " m"))
    print(_describe("distance to target", target_dist, " m"))
    print(_describe("lookahead", lookahead, " m"))
    print(_describe("distance / lookahead", dist_over_lookahead))
    print(_describe("radius the aim demands", required_radius, " m"))
    if required_radius:
        unreachable = sum(1 for r in required_radius if r < MIN_TURN_RADIUS_M)
        print(
            f"    below the chassis minimum ({MIN_TURN_RADIUS_M} m): "
            f"{unreachable}/{len(required_radius)} ({100 * unreachable / len(required_radius):.1f}%)"
        )
    if recomputable:
        print(f"    target BEHIND the robot (x_local <= 0): {behind}/{recomputable} ({100 * behind / recomputable:.1f}%)")

    print("\n  3. DOES THE STEERING AGREE?")
    over = sum(1 for v in published if v >= math.degrees(CRAWL_RAD))
    print(f"    |angle_error| >= CRAWL: {over}/{len(published)} ({100 * over / len(published):.1f}%)")
    print(_describe("|steering| on those ticks", big_steer))
    if big_steer:
        print(
            f"    below {NEAR_SATURATION} while past CRAWL: "
            f"{small_steer_big_err}/{len(big_steer)} ({100 * small_steer_big_err / len(big_steer):.1f}%)"
        )
    if published:
        print(f"    mean |angle_error| = {statistics.fmean(published):.1f} deg")


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()
    for bag_dir in args.bag_dirs:
        analyse(Path(bag_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
