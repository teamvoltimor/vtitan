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

from shared.config.constants import RobotSpecs
from shared.domain.enums import Axis, Direction, Section
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_router import SignSpec, pass_side_lateral_axis
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.race_tracker import TRAVEL_DIRS
from src.simulation.track_model import ContactSurface, TrackModel, _rect_corners

if TYPE_CHECKING:
    from src.simulation.kinematics import AckermannState
    from src.simulation.scenario_simulator.simulator import _StartConditions

_PASS_SIDE_APPROACH_M = 1.20
"""Range within which a sign's radius line is worth testing at all.

Only an optimisation -- the radius is a line across the corridor, so a chassis
metres away is trivially not crossing it. Wide enough that no crossing is
missed at any speed the robot reaches in one tick."""


_ROUND_ORDER = (Section.SOUTH, Section.EAST, Section.NORTH, Section.WEST)
"""The ring in COUNTERCLOCKWISE order, derived from ``TRAVEL_DIRS``.

Driving counterclockwise the south straight heads +x and arrives at EAST, east
heads +y and arrives at NORTH, and so on. Clockwise is the same tuple read
backwards."""

_OPPOSITE_SPEED_EPS_MPS = 0.02
"""Below this the chassis is not meaningfully travelling either way.

Keeps a stationary or barely-creeping robot -- during an escape's reversal
pause, say -- from latching a direction change it never made."""


def _ring_step(section: Section, direction: Direction, step: int) -> Section:
    """The section ``step`` places along the round from ``section``."""
    forward = 1 if direction is Direction.COUNTERCLOCKWISE else -1
    return _ROUND_ORDER[(_ROUND_ORDER.index(section) + forward * step) % len(_ROUND_ORDER)]


class PassSideScorer:
    """Obstacles-Challenge scoring behaviours shared into ``ScenarioSimulator``."""

    # State owned by the ScenarioSimulator subclass; declared here for type-checking.
    _track: TrackModel
    _true_signs: list[SignSpec]
    _max_sign_push: float | None
    _sign_push: dict[int, float]
    _prev_contact_xy: Waypoint
    # Signs whose radius the chassis has begun to cross but not completed.
    # Kept because a partial crossing is exactly the recoverable state the rules
    # permit, and it is worth being able to see it.
    _pass_side_engaged: set[int]
    _pass_side_scored: set[int]
    _pass_side_wrong: list[int]
    # The TRUE start, needed because the pass-side rule is travel-relative.
    _start: _StartConditions
    # Rule 9.21 state: where the chassis began travelling against the round.
    _opposite_origin: Section | None
    _reverse_run_violation: bool
    # The step at which the CURRENT opposite-travel episode began, cleared
    # whenever the chassis travels in the round direction again. Kept so a
    # diagnostic can ask what happened just BEFORE the illegal run started;
    # the step it ENDS on is already `SimResult.steps`, because the violation
    # breaks the run loop.
    _opposite_origin_step: int | None

    def _check_pass_side_violation(self, state: AckermannState) -> list[int] | None:
        """Return offending sign indices if the run must stop for a wrong-side pass.

        Implements the rule as Appendix A section 5 defines it, NOT a proxy for
        it. The offence is completing a crossing of the sign's RADIUS -- "the
        line that goes from the interior wall to the exterior wall ... and where
        the traffic sign is located" -- while on the forbidden side. Until that
        line is fully crossed the round is explicitly NOT stopped: "a threshold
        exists that can be used by the vehicle to recognize the fault state and
        fix the behaviour".

        This replaces a closest-approach test (engage at 0.60 m, score on
        receding past 0.75 m, side taken from the nearest point). That proxy
        FORBADE RECOVERY BY CONSTRUCTION: a chassis that strayed to the wrong
        side and corrected before reaching the line was scored as offending,
        which is legal driving under the rules and is the one recovery the rules
        go out of their way to permit. No detect-and-correct behaviour can be
        developed against a scorer that fails it.

        "Completely crosses" is a FOOTPRINT test, so it uses the oriented
        chassis rectangle rather than the centre point -- the round survives
        until the last corner is past the line.

        Scored from the TRUE layout and the TRUE pose, the same way
        ``_classify_collision`` uses true geometry -- deliberately not from
        ``SignRouter.wrong_side_violations``, which is computed in the believed
        frame from discovered colours and so conflates where the chassis drove
        with what the robot thinks it saw. That record is still kept as a
        measure of discovery quality. It just must not be what ends a run.

        Returns ``None`` when no violation has occurred this tick.
        """
        if not self._true_signs:
            return None
        corners = _rect_corners(state.x, state.y, state.yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
        for index, sign in enumerate(self._true_signs):
            if index in self._pass_side_scored:
                continue
            if math.hypot(sign.x - state.x, sign.y - state.y) > _PASS_SIDE_APPROACH_M:
                continue
            geometry = self._radius_geometry(sign)
            if geometry is None:
                continue
            depth_axis, ahead, lateral_axis, permitted = geometry
            sign_depth = sign.x if depth_axis == Axis.X else sign.y
            # Signed distance past the radius, along the round's travel
            # direction. The chassis has completely crossed once its LAST
            # corner is beyond the line.
            behind = [
                (c.x if depth_axis == Axis.X else c.y) - sign_depth
                for c in corners
            ]
            if min(d * ahead for d in behind) <= 0.0:
                # Still straddling the line, or not there yet -- the rules let
                # the vehicle fix its side from here, so nothing is decided.
                self._pass_side_engaged.add(index)
                continue
            self._pass_side_scored.add(index)
            robot_lat = state.x if lateral_axis == Axis.X else state.y
            sign_lat = sign.x if lateral_axis == Axis.X else sign.y
            if robot_lat != sign_lat and (1 if robot_lat > sign_lat else -1) != permitted:
                self._pass_side_wrong.append(index)
        return sorted(self._pass_side_wrong) or None

    def _radius_geometry(self, sign: SignSpec) -> tuple[Axis, int, Axis, int] | None:
        """``(depth_axis, ahead, lateral_axis, permitted)`` for this sign's radius.

        ``ahead`` is +1/-1 along ``depth_axis`` pointing the way the round is
        driven, so "past the line" is a single signed comparison. ``permitted``
        is the side of the sign the vehicle is required to be on, which is
        travel-relative and therefore depends on the round's direction.
        """
        corridor = corridor_for_position(sign.x, sign.y)
        rule = pass_side_lateral_axis(corridor, SignColor(sign.color), self._start.direction)
        if rule is None:
            return None
        lateral_axis, permitted = rule
        # The radius runs across the corridor, so the vehicle travels along the
        # OTHER axis; TRAVEL_DIRS gives which way along it.
        heading = TRAVEL_DIRS[(corridor, self._start.direction)]
        if lateral_axis == Axis.Y:
            return Axis.X, (1 if heading.nx > 0 else -1), lateral_axis, permitted
        return Axis.Y, (1 if heading.ny > 0 else -1), lateral_axis, permitted

    def _check_reverse_run_violation(self, state: AckermannState, step: int) -> bool:
        """Has the vehicle driven opposite the round direction past its allowance?

        Rule 9.21: the vehicle may drive against the round direction "for two
        sections only: the section where the direction was changed and the
        neighbouring section". Appendix A cases 4 and 5 make the boundary
        explicit -- going COMPLETELY out of the neighbouring section that way
        "will lead to the immediate stop of the round", while a projection only
        PARTLY into the next one does not.

        Modelled here because the simulator otherwise scores exactly one
        round-end condition (the wrong-side pass) and would grade an illegal
        round as a clean one. Escapes and U-turns reverse the chassis routinely
        -- 11 of 256 runs register a U-turn -- so every escape tuning decision
        has been made against a judge that forgives this. Same class of gap as
        the pass-side scorer, which agreed with the router's own mistake.

        Direction of travel is taken from the VELOCITY, not the heading, so
        driving back-to-front is not itself an offence: rules p38 case 6 allows
        it outright as long as the vehicle is being moved in the round
        direction.

        Simplifications, stated rather than hidden:

        * The origin is the section the chassis is in when it starts moving
          opposite. The rules say that for a change made ON a border it is the
          FORWARD section; here a border straddle resolves to whichever section
          ``corridor_for_position`` reports for the centre.
        * Case 5 (several direction changes) says the allowance is measured
          from the change CLOSEST TO THE FINISH. This keeps the first origin
          until the chassis travels in the round direction again, which is
          equal or stricter, never more permissive.
        """
        if self._start.direction is None:
            return False
        section = corridor_for_position(state.x, state.y)
        if section is None:
            return False
        heading = TRAVEL_DIRS[(section, self._start.direction)]
        # Negative v is reverse gear, which flips the motion vector -- exactly
        # what makes back-to-front travel in the round direction legal here.
        along = state.v * (math.cos(state.yaw) * heading.nx + math.sin(state.yaw) * heading.ny)
        if abs(state.v) < _OPPOSITE_SPEED_EPS_MPS:
            return self._reverse_run_violation
        if along >= 0.0:
            self._opposite_origin = None
            self._opposite_origin_step = None
            return self._reverse_run_violation
        if self._opposite_origin is None:
            self._opposite_origin = section
            self._opposite_origin_step = step
        allowed = {self._opposite_origin, _ring_step(self._opposite_origin, self._start.direction, -1)}
        # "Completely out" is a footprint test, like the pass-side radius: while
        # any corner is still in an allowed section the round stands.
        corners = _rect_corners(state.x, state.y, state.yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
        for corner in corners:
            if corridor_for_position(corner.x, corner.y) in allowed:
                return self._reverse_run_violation
        self._reverse_run_violation = True
        return True

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
