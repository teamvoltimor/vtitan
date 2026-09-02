"""How far a commanded kinematic step actually gets before a solid surface stops it.

Bisects translation and rotation separately against :class:`TrackModel`
contact geometry, for :class:`~src.simulation.simulated_hardware_gateway.SimulatedHardwareGateway`.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import TYPE_CHECKING

from src.navigation.utils import wrap_angle as _wrap_angle

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.simulation.kinematics import AckermannState
    from src.simulation.track_model import ContactSurface, TrackModel

_MIN_STEP_SCALE = 1e-3
"""Smallest usable fraction of a commanded step.

Below this the move is submillimetre and the chassis is, for scoring purposes,
against the surface rather than sliding along it."""

_STEP_BISECTIONS = 8
"""Bisections used to find the largest fitting fraction of a step.

Eight halvings resolve a 7.5 mm tick to ~0.03 mm, well under the 30 mm LIDAR
noise the navigator is steering on, so more would be measuring nothing."""


def _slide_along(
    track: TrackModel,
    solid_surfaces: frozenset[ContactSurface],
    state: AckermannState,
    candidate: AckermannState,
    largest: Callable[[Callable[[float], bool]], float],
    delta: tuple[float, float],
    scaled_move: float,
) -> AckermannState | None:
    """The blocked step with its into-surface component dropped, or ``None``.

    ``None`` also when sliding gains nothing over ``scaled_move`` -- the
    distance the caller's scale-along-the-same-vector fallback would cover --
    so the caller never has to compare the two itself.

    Keeps whichever single axis survives contact. Per-AXIS rather than against a
    surface normal because ``TrackModel`` reports which surface was touched but
    not its orientation -- exact on this mat, where every wall, inner-block
    face, sign and parking fin is an axis-aligned box, and wrong on a track with
    angled walls.

    Driving squarely into a wall still yields nothing: the into-surface axis is
    blocked and the along-surface one is zero. Head-on contact therefore makes
    no progress and reversing out remains a real escape.
    """
    dx, dy = delta

    def free_xy(mx: float, my: float) -> bool:
        return track.contact_surface(state.x + mx, state.y + my, state.yaw) not in solid_surfaces

    along_x = dx * largest(lambda m: free_xy(dx * m, 0.0))
    along_y = dy * largest(lambda m: free_xy(0.0, dy * m))
    best_x, best_y = (along_x, 0.0) if abs(along_x) >= abs(along_y) else (0.0, along_y)
    commanded = math.hypot(dx, dy)
    travelled = math.hypot(best_x, best_y)
    if not commanded or travelled <= scaled_move:
        return None
    return replace(
        candidate,
        x=state.x + best_x,
        y=state.y + best_y,
        yaw=state.yaw,
        v=candidate.v * (travelled / commanded),
    )


def allowed_step(
    track: TrackModel,
    solid_surfaces: frozenset[ContactSurface],
    state: AckermannState,
    candidate: AckermannState,
    *,
    slide: bool = False,
) -> AckermannState | None:
    """The furthest along the commanded step the chassis may actually go.

    Returns ``candidate`` itself when the whole step is clear, a scaled
    pose when a solid surface cuts it short, or ``None`` when no part of
    it fits and the body cannot move at all.

    Rotation and translation are limited separately, and that separation
    is the whole point. A wall bounds how far the chassis may TURN, not
    whether it may advance: a real car against a wall keeps driving with
    its corner scraping while the steering gradually pulls it clear.
    Scaling both together instead leaves the chassis stuck at its maximum
    yaw forever, because from there every step asks for more rotation --
    each tick the turn needs about 2 mm more clearance than the same
    tick's forward motion earns, so no fraction of it ever fits.

    So: keep the full translation and take whatever fraction of the turn
    fits alongside it. Advancing at the limiting angle earns a fraction of
    a millimetre of clearance per tick, which lets a little more of the
    turn through on the next one, and the chassis peels away. Only if the
    translation itself is blocked -- driving squarely into a wall -- is it
    cut back, and then to nothing, so head-on contact still makes no
    progress and reversing out is still a real escape.

    ``slide`` changes only the blocked-translation branch. Without it the move
    is scaled along the vector it already had, so a chassis leaning into a
    surface loses the along-surface component too and barely advances --
    measured at 20 deg of incidence, a 7.5 mm tick travels 0.125 mm where a
    rubbing chassis would gain 7.05 mm, 56x less. With it, the translation is
    decomposed and the surviving component kept, which is what contact with
    friction-free sliding actually does.

    The decomposition is per-AXIS rather than against a surface normal, because
    ``TrackModel`` reports which surface was touched but not its orientation.
    That is exact here and only here: every wall, inner-block face, sign and
    parking fin on this mat is an axis-aligned box (the fins are rotated 90 deg,
    which leaves them axis-aligned), so the tangent of any contacted surface IS
    one of the two axes. It would need a real normal on a track with angled
    walls.

    Args:
        track: Track geometry to test contact against.
        solid_surfaces: Which contact surfaces physically stop the chassis.
        state: The chassis pose the step starts from.
        candidate: The pose the kinematics produced for this tick.
        slide: Whether a blocked translation may slide along the surface
            instead of being scaled to nothing.

    Returns:
        The pose to adopt, or ``None`` if even the smallest step collides.
    """
    if track.contact_surface(candidate.x, candidate.y, candidate.yaw) not in solid_surfaces:
        return candidate

    dx, dy = candidate.x - state.x, candidate.y - state.y
    dyaw = _wrap_angle(candidate.yaw - state.yaw)

    def free(move: float, turn: float) -> bool:
        return (
            track.contact_surface(
                state.x + dx * move,
                state.y + dy * move,
                state.yaw + dyaw * turn,
            )
            not in solid_surfaces
        )

    def largest(fits: Callable[[float], bool]) -> float:
        """Greatest fraction in [0, 1] that fits, by bisection."""
        if not fits(_MIN_STEP_SCALE):
            return 0.0
        lo, hi = _MIN_STEP_SCALE, 1.0
        for _ in range(_STEP_BISECTIONS):
            mid = (lo + hi) / 2
            if fits(mid):
                lo = mid
            else:
                hi = mid
        return lo

    # Full translation, as much of the turn as fits alongside it.
    if free(1.0, 0.0):
        turn = largest(lambda t: free(1.0, t))
        return replace(candidate, yaw=state.yaw + dyaw * turn)

    # The translation itself is blocked: hold the heading and advance as
    # far as fits, which is nothing when driving squarely into a wall.
    move = largest(lambda m: free(m, 0.0))
    slid = (
        _slide_along(track, solid_surfaces, state, candidate, largest, (dx, dy), math.hypot(dx * move, dy * move))
        if slide
        else None
    )
    if slid is not None:
        return slid
    if move == 0.0:
        return None
    return replace(
        candidate,
        x=state.x + dx * move,
        y=state.y + dy * move,
        yaw=state.yaw,
        v=candidate.v * move,
    )
