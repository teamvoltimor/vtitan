"""How far a commanded kinematic step actually gets before a solid surface stops it.

Bisects translation and rotation separately against :class:`TrackModel`
contact geometry, for :class:`~src.simulation.simulated_hardware_gateway.SimulatedHardwareGateway`.
"""

from __future__ import annotations

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


def allowed_step(
    track: TrackModel,
    solid_surfaces: frozenset[ContactSurface],
    state: AckermannState,
    candidate: AckermannState,
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

    Args:
        track: Track geometry to test contact against.
        solid_surfaces: Which contact surfaces physically stop the chassis.
        state: The chassis pose the step starts from.
        candidate: The pose the kinematics produced for this tick.

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
    if move == 0.0:
        return None
    return replace(
        candidate,
        x=state.x + dx * move,
        y=state.y + dy * move,
        yaw=state.yaw,
        v=candidate.v * move,
    )
