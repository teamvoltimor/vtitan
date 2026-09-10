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
