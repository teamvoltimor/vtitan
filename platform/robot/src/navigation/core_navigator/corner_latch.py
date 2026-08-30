"""Hold the corner-turn preview open until the turn has actually been driven.

:func:`~src.navigation.track_geometry.path_turn_ahead` is a *preview*: it
measures the heading change the path makes within
``pursuit.CORNER_PREVIEW_DISTANCE_M`` ahead of the current waypoint. That makes
it a leading signal on approach, which is exactly what
:meth:`~src.navigation.control.controllers.waypoint_controller.WaypointController.select_lookahead`
wants -- and it means the signal DECAYS to zero as the chassis enters the arc,
because the preview window slides past the corner the chassis is now inside.

So the signal is smallest precisely when the corner is being driven, and
thresholding it un-arms the short lookahead mid-turn. Measured on hardware
2026-08-30 (``run_20260830_014612``, first corner, west -> south at 6.1 s), with
``CORNER_TURN_THRESHOLD_RAD`` at 0.35::

    rel_t   turn    look    aerr     xtrack
    -3.99   1.373   0.160   +0.573   0.179     preview armed, short lookahead
    -2.40   0.590   0.160   -0.612   0.076
    -2.00   0.197   0.320   -1.215   0.011     <- un-armed 2 s BEFORE the corner
    -0.60   0.000   0.320   -1.233   0.027
    +1.21   0.000   0.320   -1.053   0.119     <- crosstrack diverging

The other two demands cannot cover the gap. ``crosstrack_error`` is measured
against the planned path and stays at 0.002-0.027 m right through the corner --
the chassis is ON the path, it is POINTING 70 degrees off it -- and
``sign_ahead`` is an Obstacles-only boolean. ``select_lookahead``'s own
docstring records the earlier form of this failure ("sat at 0.9 rad of heading
error for three seconds commanding 0.23 of full lock, because it was still
on-path"); previewing the turn fixed it on approach, and this fixes the same
thing once the robot is inside the corner.

Latching on ELAPSED TIME or on a waypoint count would both be proxies. The
honest release condition is the one the preview itself stated: it said the path
turns by ``previewed`` radians, so hold until the chassis has actually turned
that much. Nothing else needs to be known about the corner, and a turn that is
tighter or slower than planned holds the lookahead for exactly as long as it
really takes rather than for a guessed budget.
"""

from __future__ import annotations

import math

from src.navigation.utils import wrap_angle

_COMPLETION_FRACTION = 0.8
"""Fraction of the previewed heading change that counts as "turn driven".

Not 1.0: ``path_turn_ahead`` measures the path's own heading change over the
preview window, while the release test measures the CHASSIS heading change, and
the two do not have to agree exactly -- the robot may cut the corner slightly,
or the preview may still have been growing when the latch armed. Requiring the
full figure risks a latch that never releases on a corner the robot rounds
slightly tight. Requiring most of it releases within a few ticks of the real
exit.
"""

_MAX_LATCH_YAW_RAD = 2.0 * math.pi
"""Backstop: release after a full revolution of accumulated heading change.

The release test is monotone in |yaw - yaw_at_arm| only until that wraps. A
robot spinning on the spot (a failed escape, say) would otherwise re-satisfy
the test periodically rather than never, but could also hold the latch for a
long time in between. This bounds the damage to one revolution regardless.
"""


class CornerLatch:
    """Sticky version of the corner-turn preview.

    Feed it the raw ``path_turn_ahead`` reading and the current chassis yaw
    each tick; it returns the value the lookahead selector should act on.
    Stateless callers get the raw reading back unchanged whenever no corner is
    being driven, so a straight behaves exactly as before.
    """

    def __init__(self) -> None:
        self._previewed_rad: float = 0.0
        self._yaw_at_arm: float | None = None
        self._accumulated_rad: float = 0.0
        self._last_yaw: float | None = None

    @property
    def is_latched(self) -> bool:
        """True while a previewed corner is still being driven."""
        return self._yaw_at_arm is not None

    def reset(self) -> None:
        """Forget any corner in progress.

        For a new round or a replanned path, where the previewed turn the latch
        is holding open may no longer be on the route at all.
        """
        self._previewed_rad = 0.0
        self._yaw_at_arm = None
        self._accumulated_rad = 0.0
        self._last_yaw = None

    def update(self, turn_ahead_rad: float, robot_yaw: float, arm_threshold_rad: float) -> float:
        """Return the turn-ahead value to act on this tick.

        Args:
            turn_ahead_rad: Raw :func:`path_turn_ahead` reading.
            robot_yaw: Current chassis heading (world frame, radians).
            arm_threshold_rad: ``pursuit.CORNER_TURN_THRESHOLD_RAD`` -- the same
                value ``select_lookahead`` compares against, read from tuning by
                the caller rather than duplicated here.

        Returns:
            ``turn_ahead_rad``, or the previewed value being held if a corner
            armed earlier has not yet been driven.
        """
        if self._last_yaw is not None:
            self._accumulated_rad += abs(wrap_angle(robot_yaw - self._last_yaw))
        self._last_yaw = robot_yaw

        # Re-arming while already latched refreshes the target rather than
        # restarting it: a corner whose preview is still growing should hold to
        # the LARGEST turn it ever promised, not the last one seen before the
        # signal decayed.
        if turn_ahead_rad >= arm_threshold_rad:
            if self._yaw_at_arm is None:
                self._yaw_at_arm = robot_yaw
                self._accumulated_rad = 0.0
            self._previewed_rad = max(self._previewed_rad, turn_ahead_rad)
            return turn_ahead_rad

        if self._yaw_at_arm is None:
            return turn_ahead_rad

        turned = abs(wrap_angle(robot_yaw - self._yaw_at_arm))
        if turned >= _COMPLETION_FRACTION * self._previewed_rad or self._accumulated_rad >= _MAX_LATCH_YAW_RAD:
            held = self._previewed_rad
            self.reset()
            # The tick that completes the turn still reports the held value:
            # releasing to a decayed reading on the same tick would drop the
            # lookahead back out mid-exit, which is the behaviour this exists
            # to prevent.
            return max(turn_ahead_rad, held)

        return max(turn_ahead_rad, self._previewed_rad)
