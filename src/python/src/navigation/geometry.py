"""Chassis-footprint geometry shared across navigation modules.

Pure functions of ``RobotSpecs`` — no tuning, no track layout. Extracted from
``sign_router.py`` (the only caller until now) so the reasoning behind each
value has one home instead of being buried in a routing module that doesn't
otherwise own chassis geometry.
"""

from __future__ import annotations

import math

from shared.config.constants import RobotSpecs


def chassis_half_diagonal_m() -> float:
    """Half the chassis diagonal — the clearance radius while mid-turn.

    Sized on the chassis half-DIAGONAL, not half-width. Half-width only bounds
    a robot travelling parallel to the surface it is clamped against; a robot
    still turning presents its corner instead, which reaches 0.18 m rather
    than 0.10 m. Sign deformations bite hardest right at a corner — exactly
    where the robot is mid-turn — so a half-width clamp let the corner clip
    the inner block while the waypoint itself was still nominally legal.
    Measured over the obstacles fixtures, widening this removed one
    inner-block and one sign collision (9/16 -> 7/16); going further to 0.24
    over-constrains the deformation and regresses to 10/16.
    """
    return math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)


def behind_tolerance_m() -> float:
    """How far behind the robot's own origin something may sit and still count as alongside.

    Half the chassis length, so an object level with the rear bumper still
    counts (it's alongside, not cleared) but one genuinely receding behind
    stops competing with whatever is coming up next.
    """
    return RobotSpecs.LENGTH / 2


def turn_radius_at_speed_m(speed_mps: float) -> float:
    """The chassis's minimum turn radius at ``speed_mps``, from the measured curve.

    ``R = intercept + slope * |v|``, capped: the radius is a SPEED CURVE on
    this chassis (0.239 m at 0.10 m/s, 0.462 m uncapped at 0.22 m/s), not a
    constant -- see ``adr:0086-simulator-realism``. The same curve the
    simulator's kinematics floor on, read from the robot profile so the two
    cannot drift apart.
    """
    return min(
        RobotSpecs.MIN_TURN_RADIUS_CAP_M,
        RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M + RobotSpecs.MIN_TURN_RADIUS_SLOPE_S * abs(speed_mps),
    )


def arc_fit_speed_mps(run_up_m: float, lateral_m: float) -> float | None:
    """Speed at which one arc of the chassis's own radius buys ``lateral_m`` within ``run_up_m``.

    A single arc of radius ``R`` displaces the chassis sideways by
    ``s^2 / (2R)`` over ``s`` metres of travel (small-angle; the same
    expression ``diag_bag_pass_side_speed.py`` separates failed crossings
    with). Inverting: the arc fits when ``R <= s^2 / (2 * lateral)``, and on
    this chassis ``R`` is a speed curve, so that is a SPEED:
    ``v = (s^2 / (2 * lateral) - intercept) / slope``.

    Returns ``None`` when there is nothing to buy (``lateral_m <= 0``): no cap
    applies. Returns ``0.0`` when the run-up is spent or the fitting radius
    sits below the curve's intercept -- the arc cannot be delivered at any
    speed, and the caller's floor decides what to do with that.
    """
    if lateral_m <= 0.0:
        return None
    if run_up_m <= 0.0:
        return 0.0
    radius_fit = run_up_m * run_up_m / (2.0 * lateral_m)
    return max(0.0, (radius_fit - RobotSpecs.MIN_TURN_RADIUS_INTERCEPT_M) / RobotSpecs.MIN_TURN_RADIUS_SLOPE_S)
