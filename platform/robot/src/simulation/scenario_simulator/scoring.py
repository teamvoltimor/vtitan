"""Pass-side and obstacle-contact scoring for :class:`ScenarioSimulator`.

Extracted as a mixin so the Obstacles-Challenge judging (wrong-side pass
detection and legal-pillar-nudge downgrade) lives in its own module while
staying instance methods on ``ScenarioSimulator`` — callers import the class,
not these methods, so the split is invisible to them. The scoring state these
methods read and write is owned by the ``ScenarioSimulator`` subclass; it is
declared here only so type-checking can see it.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.domain.enums import Axis
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_router import SignSpec, pass_side_lateral_axis
from src.navigation.planning.waypoints import corridor_for_position
from src.simulation.track_model import ContactSurface, TrackModel

if TYPE_CHECKING:
    from src.simulation.kinematics import AckermannState
    from src.simulation.scenario_simulator.simulator import _StartConditions

_PASS_SIDE_ENGAGE_M = 0.60
"""Range within which the chassis counts as negotiating a sign.

Wide enough to admit a deliberately wide berth -- the sign lane's own spec is
+27.86 cm on the free side -- and far short of the opposite corridor, which is
metres away, so no sign is ever scored against a pass down the other side of
the track."""

_PASS_SIDE_CLEAR_M = 0.75
"""Range beyond which a negotiated sign counts as cleared, and is scored.

Held clear of ``_PASS_SIDE_ENGAGE_M`` on purpose: with one shared threshold a
chassis hovering at the boundary would score, re-engage and score again. The
gap is the hysteresis that makes the pass a single event."""


class PassSideScorer:
    """Obstacles-Challenge scoring behaviours shared into ``ScenarioSimulator``."""

    # State owned by the ScenarioSimulator subclass; declared here for type-checking.
    _track: TrackModel
    _true_signs: list[SignSpec]
    _max_sign_push: float | None
    _sign_push: dict[int, float]
    _prev_contact_xy: Waypoint
    _pass_side_closest: dict[int, tuple[float, Waypoint]]
    _pass_side_engaged: set[int]
    _pass_side_scored: set[int]
    _pass_side_wrong: list[int]
    # The TRUE start, needed because the pass-side rule is travel-relative.
    _start: _StartConditions

    def _check_pass_side_violation(self, state: AckermannState) -> list[int] | None:
        """Return offending sign indices if the run must stop for a wrong-side pass.

        The Obstacles Challenge forbids clearing a red obstacle on its inner
        side or a green on its outer side. Scored here from the TRUE layout and
        the TRUE pose, the same way ``_classify_collision`` uses true geometry —
        deliberately not from ``SignRouter.wrong_side_violations``, which is
        computed in the believed frame from discovered colours and so conflates
        where the chassis drove with what the robot thinks it saw. That record
        is still kept; it is a useful measure of discovery quality, and the gap
        between it and this is exactly that error. It just must not be what
        ends a run.

        The side is decided at the chassis's CLOSEST APPROACH to each sign, not
        at whatever instant a distance threshold is crossed: closest approach is
        where the choice of side is actually made, and measuring anywhere else
        is the same mistake as reading a pass off a waypoint index instead of
        the polyline. A sign is scored once it has been approached and then
        cleared, so a run is never failed for a sign it is still negotiating.

        Returns ``None`` when no violation has occurred this tick.
        """
        if not self._true_signs:
            return None
        x, y = state.x, state.y
        for index, sign in enumerate(self._true_signs):
            if index in self._pass_side_scored:
                continue
            distance = math.hypot(sign.x - x, sign.y - y)
            if distance <= _PASS_SIDE_ENGAGE_M:
                self._pass_side_engaged.add(index)
                closest = self._pass_side_closest.get(index)
                if closest is None or distance < closest[0]:
                    self._pass_side_closest[index] = (distance, Waypoint(x, y))
                continue
            if index not in self._pass_side_engaged or distance <= _PASS_SIDE_CLEAR_M:
                continue
            self._pass_side_scored.add(index)
            if self._is_wrong_side(sign, self._pass_side_closest[index][1]):
                self._pass_side_wrong.append(index)
        return sorted(self._pass_side_wrong) or None

    def _is_wrong_side(self, sign: SignSpec, chassis: Waypoint) -> bool:
        """Was ``chassis`` on the forbidden side of ``sign``?

        The rule is travel-relative -- the vehicle must pass to its own RIGHT of
        a red pillar and its own LEFT of a green one (rules 9.19) -- so this
        needs the round's direction and takes it from ``self._start``, the TRUE
        one, matching the true layout and true pose the rest of this check uses.

        Was a ``@staticmethod`` reading a direction-agnostic ``red outward,
        green inward`` until 2026-09-03. That is the counterclockwise answer
        given for both directions, so every CLOCKWISE round was graded against
        an inverted rule -- and because ``ROUTING_TABLE`` carried the same
        mistake, the simulator agreed with the router and the corpus reported no
        violation. A scorer must not share its convention with the thing it
        scores; that is what made this invisible for two months.
        """
        rule = pass_side_lateral_axis(
            corridor_for_position(sign.x, sign.y), SignColor(sign.color), self._start.direction
        )
        if rule is None:
            return False
        axis, permitted = rule
        robot_lat = chassis.x if axis == Axis.X else chassis.y
        sign_lat = sign.x if axis == Axis.X else sign.y
        if robot_lat == sign_lat:
            return False
        return (1 if robot_lat > sign_lat else -1) != permitted

    def _score_obstacle_contact(
        self,
        surface: ContactSurface,
        state: AckermannState,
    ) -> ContactSurface:
        """Downgrade a legal pillar nudge to a non-event, keep an illegal shove.

        Touching a pillar does not end an Obstacles round. The pillar may be
        moved, and the run stands as long as any corner of it is still inside
        its 85mm placement circle -- ``TrafficSignSpecs.MAX_LEGAL_DISPLACEMENT_M``
        (59.4mm) is the displacement at which that stops being true. Scoring
        first contact as a crash, which is what the surface alone says, fails
        runs the judges would pass.

        Displacement ACCUMULATES over the ticks in contact rather than being
        read off the instantaneous overlap. Overlap depth is bounded by the
        pillar's own 50mm extent, so a max-overlap model tops out below the
        59.4mm limit and no run could ever fail it -- a scoring rule that
        cannot be violated measures nothing. Physically the pillar is shoved
        ahead of the chassis, so the distance the chassis covers while touching
        it is what moves it.
        """
        # Advance the reference EVERY tick, not only while touching. Updating it
        # only during contact makes ``moved`` the distance since the last touch,
        # so a pillar brushed twice a metre apart accumulates that whole metre
        # of driving as if it had been pushed through it.
        dx = state.x - self._prev_contact_xy.x
        dy = state.y - self._prev_contact_xy.y
        self._prev_contact_xy = Waypoint(state.x, state.y)
        if surface is not ContactSurface.OBSTACLE or self._max_sign_push is None:
            return surface
        for index in self._track.obstacle_displacements(state.x, state.y, state.yaw):
            # Only the component of travel pointing AT the pillar moves it. The
            # magnitude of travel does not: a chassis sliding past a pillar it
            # is brushing covers distance without pushing it anywhere, and
            # counting that as displacement made a 0.4 s graze -- eight ticks at
            # the measured 0.156 m/s -- reach the 59.4mm limit on its own.
            sign = self._track.obstacle_center(index)
            if sign is None:
                continue
            to_sign_x, to_sign_y = sign.x - state.x, sign.y - state.y
            norm = math.hypot(to_sign_x, to_sign_y)
            if norm <= 0.0:
                continue
            push = (dx * to_sign_x + dy * to_sign_y) / norm
            if push > 0.0:
                self._sign_push[index] = self._sign_push.get(index, 0.0) + push
        if any(push > self._max_sign_push for push in self._sign_push.values()):
            return surface
        # Touched, but still inside its circle: not a collision, and not the
        # controller's cue to run an escape either.
        return ContactSurface.NONE
