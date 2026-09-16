r"""Can the chassis change BAND in a corner going forwards, and what does reversing buy?

Operator proposal, 2026-09-15: *enter the corner aligned, and once the chassis
is fully inside the free 1 m x 1 m corner square, REVERSE with the opposite
steering angle so it backs up and aligns -- useful for outer-lane to inner-lane
and inner to outer.*

This decides that with geometry rather than opinion, because the constraint it
targets is already measured and hard:
``aim_point_demands_an_impossible_radius`` has 58% of ticks asking for 0.23 m
against the floor, and ``sign_pair_cells_and_corner_shear`` says every band
change demands a radius the chassis lacks.

THE THREE FACTS IT RESTS ON, all from config rather than assumption:

* The turn radius is a SPEED CURVE, ``R(v) = MIN_TURN_RADIUS_INTERCEPT_M +
  MIN_TURN_RADIUS_SLOPE_S * v`` capped at ``MIN_TURN_RADIUS_CAP_M``. Slower is
  tighter, which is why "just crawl" is a real alternative that has to be
  scored alongside the proposal rather than assumed away.
* A band change is NOT the 0.20 m between division lines. To pass a sign at
  0.40 on the wall side the chassis centre must clear it by half the sign, half
  the chassis and ``sign_clearance_margin_m``; to pass a sign at 0.60 on the
  far side it must clear the other way. The worst pair is the full width of
  that, and it is the number the router has to plan.
* Reversing an Ackermann chassis FLIPS the yaw response, which is why the
  proposal says "opposite angle". A reverse arc therefore adds heading in the
  SAME direction while spending longitudinal room BACKWARDS -- the corner
  square is the only place on the track with room to spend.

WHAT IT REPORTS. For a forward-only S-curve (two opposite arcs, the manoeuvre
the planner has today) the lateral offset is bounded by ``2R`` however long you
make it. So the first table is a pass/fail on arithmetic, not a simulation: if
the required shift exceeds ``2R`` at every speed whose radius fits the corner,
no amount of tuning the pursuit controller reaches it.

The second table gives the proposal its due: a reverse segment of length L at
radius R buys ``L/R`` radians of heading, and the corner square bounds L.

CAVEAT, and it is the whole reason this is arithmetic and not a sim sweep. The
simulator UNDER-ROTATES 40-50% during manoeuvres and runs this class of
manoeuvre far less than hardware does, so it is biased AGAINST exactly this
proposal. A sim A/B that says "no" would not be evidence. Geometry has no such
bias.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/sim/diag_corner_band_change_feasibility.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CorridorDimensions, RobotSpecs, TrackDimensions, TrafficSignSpecs

from scripts.common.tables import print_table


def _radius(v: float) -> float:
    """The turn radius the chassis actually achieves at speed ``v``."""
    r = RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M + RobotSpecs.MIN_TURN_RADIUS_SLOPE_S * abs(v)
    return min(r, RobotSpecs.MIN_TURN_RADIUS_CAP_M)


def _band_change_m(margin_m: float) -> tuple[float, float, float]:
    """Centre-to-centre lateral shift for the worst legal sign pair.

    Returns ``(inner_centre, outer_centre, shift)``. Passing a sign that stands
    on the near division line means clearing it on the WALL side; passing one on
    the far line means clearing it on the far side. Half the sign, half the
    chassis and the router's margin stack on both.
    """
    near, far = CorridorDimensions.DIVISION_OUTER, CorridorDimensions.DIVISION_INNER
    stack = TrafficSignSpecs.WIDTH / 2.0 + RobotSpecs.WIDTH / 2.0 + margin_m
    inner_centre = near - stack
    outer_centre = far + stack
    return inner_centre, outer_centre, outer_centre - inner_centre


def _s_curve(shift: float, radius: float) -> tuple[float, float] | None:
    """Longitudinal run and arc angle for a two-arc lane change, or None if impossible.

    Two opposite arcs of the same radius give ``lateral = 2R(1 - cos phi)`` and
    ``longitudinal = 2R sin phi``. Lateral is therefore bounded by ``2R`` no
    matter how much room there is, which is the whole point: this is a hard
    ceiling, not a tuning limit.
    """
    if shift > 2.0 * radius:
        return None
    phi = math.acos(1.0 - shift / (2.0 * radius))
    return 2.0 * radius * math.sin(phi), math.degrees(phi)


def main() -> int:
    """Print the forward-only ceiling, then what a reverse segment adds."""
    margin = 0.10  # sign_router.sign_clearance_margin_m
    inner, outer, shift = _band_change_m(margin)
    corner = TrackDimensions.CORNER_MAX - TrackDimensions.CORNER_MIN

    print("GEOMETRY")
    print(f"  chassis            {RobotSpecs.LENGTH:.3f} x {RobotSpecs.WIDTH:.3f} m")
    print(f"  corner free square {corner:.2f} x {corner:.2f} m")
    print(f"  division lines     {CorridorDimensions.DIVISION_OUTER} / {CorridorDimensions.DIVISION_INNER}")
    print(
        f"  clearance stack    sign/2 + chassis/2 + margin = "
        f"{TrafficSignSpecs.WIDTH / 2:.3f} + {RobotSpecs.WIDTH / 2:.3f} + {margin:.3f}"
    )
    print(f"  centre must be at  <= {inner:.3f} (pass near sign wall-side) or >= {outer:.3f} (pass far sign outside)")
    print(f"  WORST BAND CHANGE  {shift:.3f} m of lateral shift")
    print()

    print("FORWARD-ONLY S-CURVE -- lateral is bounded by 2R at ANY length")
    rows = []
    for v in (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.58):
        r = _radius(v)
        fits = _s_curve(shift, r)
        rows.append(
            [
                f"{v:.2f}",
                f"{r:.3f}",
                f"{2 * r:.3f}",
                "YES" if fits else "IMPOSSIBLE",
                f"{fits[0]:.3f}" if fits else "-",
                f"{fits[1]:.0f}" if fits else "-",
            ]
        )
    print_table(rows, ["speed m/s", "R(v)", "max lateral 2R", "reaches?", "long. run m", "arc deg"])
    print()

    print("REVERSE SEGMENT -- heading bought by backing L metres at R(v)")
    print(f"  the corner square bounds L; a {RobotSpecs.LENGTH:.2f} m chassis fully inside")
    print(f"  {corner:.2f} m leaves about {corner - RobotSpecs.LENGTH:.2f} m of straight-line room,")
    print("  and a reverse ARC uses the diagonal, so L is generous rather than tight.")
    rows = []
    for v in (0.05, 0.10, 0.15, 0.20):
        r = _radius(v)
        rows.append(
            [
                f"{v:.2f}",
                f"{r:.3f}",
                *[f"{math.degrees(length / r):.0f}" for length in (0.15, 0.25, 0.35, 0.50)],
            ]
        )
    print_table(
        rows,
        ["speed m/s", "R(v)", "L=0.15", "L=0.25", "L=0.35", "L=0.50"],
    )
    print("  (columns are degrees of heading gained, per reverse segment)")
    print()
    print(
        "READ IT LIKE THIS: if the forward table says IMPOSSIBLE at every speed whose\n"
        "radius the corner can contain, the band change cannot be tuned into existence\n"
        "and a reverse segment is not an optimisation but the only primitive that reaches.\n"
        "The simulator under-rotates 40-50% during manoeuvres, so it is biased AGAINST\n"
        "this proposal; a sim A/B that says no would not be evidence."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
