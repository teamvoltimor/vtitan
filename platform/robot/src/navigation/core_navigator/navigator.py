"""Core Navigator decoupled from ROS2.

This class implements the navigation loop purely in Python using the
HardwareGateway interface. This achieves Dependency Inversion (SOLID),
making the navigation logic trivial to test and execute in a simulator.
"""

from __future__ import annotations

import logging
import math
import statistics
from collections import deque
from typing import TYPE_CHECKING

from shared.config.constants import (
    CompetitionSpecs,
    CorridorDimensions,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
)
from shared.domain.enums import Direction, NavigatorPhase, RiskLevel
from shared.domain.models import NavigatorDebugSnapshot, Pose, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.clearances import (
    ClearanceAggregate,
    clearances_from_scan,
    threat_direction,
)
from src.navigation.control.controllers import (
    CollisionAvoidanceController,
    EscapeManeuver,
    StuckDetector,
    WaypointController,
    bumper_gap_ahead,
    mask_mapped_obstacles,
)
from src.navigation.core_navigator.escape_recovery import EscapeRecovery
from src.navigation.corridor_estimator import classify_width
from src.navigation.geometry import chassis_half_diagonal_m
from src.navigation.planning.sign_lane import SignLaneParams, apply_sign_lanes
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.core_navigator.corner_latch import CornerLatch
from src.navigation.track_geometry import cross_track_error, path_turn_ahead
from src.navigation.utils import wrap_angle

if TYPE_CHECKING:
    import numpy as np
    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.enums import Section

    from src.navigation.maneuvers.parking import ParkController
    from src.navigation.planning.sign_router import SignRouter
    from src.navigation.ports import HardwareGateway
    from src.navigation.race_tracker import LapDetector

logger = logging.getLogger(__name__)


def _outgoing_bearing(waypoints: list[Waypoint], index: int) -> float:
    """Direction the path points at ``index``, toward its next waypoint.

    Wraps to waypoint 0 past the end -- the planned path is one canonical lap
    of a closed loop (see ``replace_path``), not an open segment.
    """
    wp0 = waypoints[index]
    wp1 = waypoints[(index + 1) % len(waypoints)]
    return wp0.bearing_to(wp1)


class CoreNavigator(EscapeRecovery):
    """Orchestrates navigation using a HardwareGateway interface."""

    def __init__(
        self,
        gateway: HardwareGateway,
        waypoints: list[Waypoint],
        num_laps: int = CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        tuning: NavigationTuning | None = None,
        sign_router: SignRouter | None = None,
        lap_detector: LapDetector | None = None,
        park_controller: ParkController | None = None,
        direction: Direction | None = None,
    ) -> None:
        self._gateway = gateway
        self._waypoints = list(waypoints)
        # The path as PLANNED, before any sign-lane transform. Kept separately
        # so each lane rebuild starts from the centreline instead of stacking
        # onto the previous lane -- see _refresh_sign_lanes.
        self._lane_base_waypoints = list(waypoints)
        self._lane_fingerprint: tuple[tuple[float, float, str], ...] | None = None
        self._num_laps = num_laps
        self._tuning = get_tuning(tuning)
        self._sign_router = sign_router
        self._lap_detector = lap_detector
        # Best current guess of travel direction, same source LapDetector and
        # the planned path already trust. ``None`` only in the sliver before
        # LIDAR inference settles; see ``set_travel_direction``.
        self._direction = direction

        self._waypoint_index = 0
        self._laps_completed = 0
        self._suppress_next_wrap = False
        self._corner_latch = CornerLatch()
        self._corridor_width_belief: dict[Section, float] = {}
        # Replan fade state -- see replace_path and REPLAN_BLEND_TICKS.
        self._blend_from: list[Waypoint] = []
        self._blend_to: list[Waypoint] = []
        self._blend_total = 0
        self._blend_left = 0
        # The speed ladder this run drives on, resolved ONCE here rather than at
        # each of the nine sites that read a tier.
        #
        # Open tolerates far more speed than Obstacles -- its binding constraint
        # is the 180 s round limit, not sign clearance, and Obstacles was
        # measured degrading monotonically with speed (in-time 38/256 at 0.156
        # m/s down to 9/256 at 0.60). `for_open_challenge` returns the base
        # ladder unchanged when the motor profile defines no OPEN_* tiers, so a
        # drivetrain without headroom to spare needs no special case.
        #
        # Keyed on sign_router presence, the same Open/Obstacles discriminator
        # STALE_TARGET_RESCUE and RETRACE_ESCAPE already use (it is None for the
        # Open Challenge by construction). Resolving once also means a tier read
        # mid-run cannot disagree with one read at startup.
        self._speed = (
            self._tuning.speed.for_obstacles_challenge()
            if sign_router is not None
            else self._tuning.speed.for_open_challenge()
        )
        # The clearance zones this run drives on, resolved ONCE for the same
        # reasons and on the same discriminator as the speed ladder above.
        #
        # Only Obstacles has an override to apply (`for_obstacles_challenge`
        # returns self when it is unset), so the Open branch reads the base
        # zones directly rather than through a `for_open_challenge` that could
        # only ever be the identity.
        self._clearance = (
            self._tuning.clearance.for_obstacles_challenge()
            if sign_router is not None
            else self._tuning.clearance
        )
        self._waypoint_threshold = self._tuning.waypoints.MAIN_LOOP_REACHED_DISTANCE_M
        self._current_corridor: Section | None = None
        self._park_controller = park_controller
        self._parking_engaged = False
        # Must clear the chassis's minimum turning radius with real margin: engaging any
        # closer than that hands ParkController a staging target already inside its own
        # turning circle, which no forward-only steering law can reach (see
        # docs/internal/2026-07-11-navigation-logic-review.md §2.3). Reuses ARC_RADIUS, same
        # as ParkController's own staging stand-off, rather than a disconnected literal.
        #
        # That margin is now much larger than it needs to be. This was sized against a
        # ~0.329 m R_min, computed as WHEELBASE/tan(MAX_STEERING_ANGLE) with the steering
        # limit still modelled at 30 deg. Both inputs were wrong: the real lock is ~70 deg,
        # and counter-phase steering pivots about the chassis centre, so the reference
        # length is WHEELBASE/2. True R_min is ~0.034 m -- an order of magnitude smaller,
        # meaning ARC_RADIUS (0.45 m) is no longer near this constraint and the engage
        # distance could be tightened on its own merits rather than on this one.
        self._park_engage_dist = self._tuning.waypoints.ARC_RADIUS

        # Escape-maneuver latching: an escape runs for its full duration_frames
        # instead of a single 50 ms tick, and repeated escapes escalate (reverse
        # longer, switch side) rather than repeating an identical failed pulse.
        self._active_maneuver: EscapeManeuver | None = None
        self._maneuver_frames_left = 0
        self._escape_count = 0  # escapes begun since the last normal drive tick with real progress
        # Base side for escapes, not a running toggle: which side a given
        # attempt actually uses is derived in _escape_steer_sign_for_attempt.
        self._escape_steer_sign = 1.0
        self._escape_sequence_start_xy: tuple[float, float] | None = None
        # Where the chassis has physically been, newest last. The basis for a
        # retrace-reverse: ground the robot occupied a moment ago is known
        # free without any rear-facing sensor. See _retrace_steer.
        self._pose_trail: deque[Pose] = deque(maxlen=self._tuning.escape.POSE_TRAIL_LEN)
        self._retracing = False

        # Controllers. Keyed on the same sign_router discriminator as the speed
        # ladder and clearance zones above, and resolved ONCE for the same
        # reason: a lookahead read mid-run cannot disagree with one read at
        # startup. Only Open has an override to apply, so Obstacles reads the
        # base parameters and its resolution path is untouched.
        self._waypoint_controller = WaypointController.from_tuning(
            self._tuning, for_open=sign_router is None
        )
        self._apply_path_wall_budget()

        # Built from the RESOLVED zones, not `self._tuning.clearance`. This
        # controller owns `assess_risk`, which is what actually fires the
        # reversing escape at `contact_dist` -- resolving everywhere except
        # here would leave the override inert on the path it was added for.
        self._collision_controller = CollisionAvoidanceController.from_tuning(
            self._tuning, clearance=self._clearance
        )

        self._stuck_detector = StuckDetector.from_tuning(self._tuning)

        # Full internal state of the most recent step(), for telemetry -- see
        # NavigatorDebugSnapshot's own docstring for why this exists.
        self._debug = NavigatorDebugSnapshot()

    @property
    def debug_snapshot(self) -> NavigatorDebugSnapshot:
        """Full internal state of the most recent ``step()`` call."""
        return self._debug

    @property
    def current_corridor(self) -> Section | None:
        """Current track corridor derived from robot position. None before first step."""
        return self._current_corridor

    @property
    def sign_router(self) -> SignRouter | None:
        """The traffic-sign router, or None outside the Obstacles Challenge."""
        return self._sign_router

    def _cache_corridor_widths(self) -> None:
        """Recover each corridor's believed width from the planned path itself.

        The navigator is handed waypoints, never the width belief that produced
        them -- but the belief is recoverable: a planned centreline sits half a
        corridor in from the mat edge, so twice a corridor's waypoint-to-edge
        distance is the width it was planned for (~0.60 believed narrow, ~1.00
        believed wide). Reading it back here avoids plumbing the estimator
        through every construction site of this class, and it stays correct by
        construction after a replan because it is derived from the same path the
        robot is actually driving.

        Recomputed only when the path is replaced, not per tick: the mapping is
        a property of the path.
        """
        offsets: dict[Section, list[float]] = {}
        for wp in self._waypoints:
            section = corridor_for_position(wp.x, wp.y)
            if section is None:
                continue
            edge = min(wp.x, TrackDimensions.MAX_COORD - wp.x, wp.y, TrackDimensions.MAX_COORD - wp.y)
            offsets.setdefault(section, []).append(edge)
        # MEDIAN, not min: a corridor's waypoints include the corner arcs at each
        # end, which cut inward and read as a narrower corridor than the straight
        # actually is. The median is the straight-section offset, which is the
        # one that equals half the believed width.
        self._corridor_width_belief = {
            section: 2.0 * statistics.median(edges) for section, edges in offsets.items() if edges
        }

    def _in_narrow_corridor(self) -> bool:
        """Whether the corridor being driven was planned as NARROW.

        Unknown corridor reads as narrow -- the cap this gates is a caution, so
        the absence of evidence should not switch it off.
        """
        if self._current_corridor is None:
            return True
        believed = self._corridor_width_belief.get(self._current_corridor)
        if believed is None:
            return True
        return classify_width(believed) == CorridorDimensions.NARROW

    def _apply_path_wall_budget(self) -> None:
        """Tell the pursuit controller how much drift this path can absorb.

        Measured off the mat's outer walls rather than the corridor-width
        belief, deliberately. WRO moves the inner walls between rounds but the
        mat's own edges are fixed, so the distance from a waypoint to the
        nearest edge is knowable without believing anything -- and it is the
        outer wall the robot keeps hitting, because an under-estimated corridor
        width biases the planned path toward it (narrow belief -> path at
        ~0.30 where the true centre is 0.50). Deriving the budget from the
        belief instead would make it wrong in exactly the rounds it matters.

        Subtracts the chassis half-width, since the budget is how far the
        *body* may stray, not its centreline.
        """
        if not self._waypoints:
            return
        clearance = min(
            min(wp.x, TrackDimensions.MAX_COORD - wp.x, wp.y, TrackDimensions.MAX_COORD - wp.y)
            for wp in self._waypoints
        )
        self._cache_corridor_widths()
        budget = clearance - RobotSpecs.WIDTH / 2 - self._tuning.pursuit.WALL_MARGIN_SAFETY_M
        self._waypoint_controller.set_crosstrack_budget(
            max(budget, self._tuning.pursuit.MIN_LOOKAHEAD_TRANSITION_M),
        )

    def replace_path(
        self,
        waypoints: list[Waypoint],
        robot_xy: tuple[float, float],
        robot_yaw: float | None = None,
    ) -> None:
        """Swap in a new planned path mid-run, resuming at the nearest point.

        Needed when the layout the path was planned against turns out to be
        wrong — the robot estimates corridor widths from LIDAR as it drives
        (see :mod:`src.navigation.corridor_estimator`), so the path has to be
        rebuilt when an estimate changes rather than being fixed at startup.

        The waypoint index cannot carry over: the new path has its own indexing
        and the old index would point somewhere arbitrary on it. Re-seeking to
        the nearest waypoint keeps progress instead of restarting the lap, and
        matters because the paths differ by centimetres, not corridors — the
        nearest point on the new path is essentially where the robot already
        was on the old one.

        Nearest-by-position alone can go wrong right after a blind round's
        direction inference commits: near a corner, several waypoints sit at
        almost the same distance while pointing in very different directions,
        and the robot's heading at that instant does not always match the
        path's local direction there yet (it has been steering, within the
        blind corridor-follower's own limits, during the creep leading up to
        the commit). Picking purely by position can then hand
        :class:`~src.navigation.control.controllers.WaypointController` a
        point past the turn, demanding a correction far larger than finishing
        the corner needs — measured on real hardware as a ~193 deg swing
        where ~90 deg would do. Passing ``robot_yaw`` re-ranks the
        near-tied-by-distance candidates (see
        ``NavigationTuning.waypoints.REPLAN_HEADING_TIE_MARGIN_M``) by
        heading agreement instead.

        Args:
            waypoints: The replacement path (single canonical lap).
            robot_xy: Current position, used to resume at the nearest waypoint.
            robot_yaw: Current heading (radians), if known. When given, breaks
                near-ties in the position search by heading agreement instead
                of taking the strict nearest. Omit where the robot has been
                tracking a path very similar to the new one (e.g. a small
                corridor-width belief update), where nearest-by-position alone
                is already safe.
        """
        previous_index = self._waypoint_index
        # Fade the new geometry in rather than teleporting the steering target
        # onto it. The path is what crosstrack and the steer target are measured
        # against, so an instant swap steps both -- measured on hardware
        # 2026-08-30 as ~0.30 m of crosstrack in one 50 ms tick, against the
        # ~0.03 m the chassis can actually travel in that time, which threw
        # heading error past CRAWL and pinned the robot at creep speed while it
        # recovered onto a line that had moved under it. See REPLAN_BLEND_TICKS.
        #
        # Only the RATE changes here; the destination is identical. The blend is
        # index-wise, guarded on equal waypoint counts so a geometry that ever
        # breaks that assumption falls back to the original instant swap instead
        # of interpolating between indices that do not correspond.
        blend_ticks = self._tuning.waypoints.REPLAN_BLEND_TICKS
        if blend_ticks > 0 and self._waypoints and len(self._waypoints) == len(waypoints):
            self._blend_from = list(self._waypoints)
            self._blend_to = list(waypoints)
            self._blend_total = blend_ticks
            self._blend_left = blend_ticks
        else:
            self._blend_left = 0

        self._waypoints = list(waypoints)
        # A replanned path is a new centreline, so the lanes have to be laid
        # over it again -- and the fingerprint cleared, or the unchanged sign
        # layout would read as "already applied" and leave the new path bare.
        self._lane_base_waypoints = list(waypoints)
        self._lane_fingerprint = None
        # A corner held open by the latch was previewed on the OLD centreline
        # and need not exist on this one, so holding it would keep the short
        # lookahead armed against a turn the robot is no longer going to make.
        self._corner_latch.reset()
        self._apply_path_wall_budget()
        robot_x, robot_y = robot_xy
        distances = [wp.distance_to_xy(robot_x, robot_y) for wp in waypoints]
        nearest_index = min(range(len(waypoints)), key=lambda i: distances[i])

        if robot_yaw is not None:
            margin = distances[nearest_index] + self._tuning.waypoints.REPLAN_HEADING_TIE_MARGIN_M
            candidates = [i for i, d in enumerate(distances) if d <= margin]
            nearest_index = min(
                candidates,
                key=lambda i: abs(wrap_angle(_outgoing_bearing(waypoints, i) - robot_yaw)),
            )

        self._waypoint_index = nearest_index

        # A large forward jump is never earned progress — the robot cannot skip
        # most of a lap between two ticks. It means the re-seek landed on the
        # far side of the start/finish seam: the path was rebuilt running the
        # other way, leaving the robot just *behind* the new waypoint 0, which
        # on a closed loop is also the tail of the list. The tail it is about to
        # drive belongs to a lap it never ran, so the wrap at the end of it is
        # the entry into lap 1, not the completion of it.
        #
        # A correct direction inference never lands here: the robot creeps
        # forward along its corridor, so its heading already matches the true
        # travel direction and the rebuilt path runs the way it is pointing.
        # This is a guard against a *wrong* inference, which on the mat is the
        # case the robot cannot rule out for itself.
        if self._waypoint_index - previous_index > len(waypoints) // 2:
            self._suppress_next_wrap = True

    def _advance_replan_blend(self) -> None:
        """Step the replanned path one tick further in, if a fade is running.

        Interpolates every waypoint from the path the robot was tracking toward
        the one the new belief asks for. The endpoint is exactly the new path --
        the final tick assigns it rather than a 99%-of-the-way interpolation, so
        no residue survives the fade and a later replan starts from a clean
        centreline.

        ``_lane_base_waypoints`` moves with it and the lane fingerprint is
        cleared each blended tick, because the base the lanes are laid over is
        genuinely changing; leaving the fingerprint alone would let the first
        blended tick's lanes stand for the whole fade. Open Challenge pays
        nothing for that -- ``_refresh_sign_lanes`` returns immediately with no
        router.
        """
        if self._blend_left <= 0:
            return

        self._blend_left -= 1
        if self._blend_left == 0:
            self._waypoints = list(self._blend_to)
            self._lane_base_waypoints = list(self._blend_to)
            self._lane_fingerprint = None
            self._blend_from = []
            self._blend_to = []
            return

        alpha = 1.0 - (self._blend_left / self._blend_total)
        blended = [
            Waypoint(
                start.x + (end.x - start.x) * alpha,
                start.y + (end.y - start.y) * alpha,
            )
            for start, end in zip(self._blend_from, self._blend_to, strict=True)
        ]
        self._waypoints = blended
        self._lane_base_waypoints = list(blended)
        self._lane_fingerprint = None

    def _refresh_sign_lanes(self) -> None:
        """Rebuild the planned path onto its pass-side lanes when the sign layout changes.

        No-op unless a ``SignRouter`` exists and ``SIGN_LANE_PLANNER`` is set,
        so the Open Challenge's path is never rewritten -- it has no router at
        all, and the early return here is what makes that structural rather
        than a matter of the flag's value.

        Lanes are always recomputed from ``_lane_base_waypoints`` (the path as
        planned) rather than from ``_waypoints``: re-laning an already-laned
        path would stack one offset on the next every time discovery refined a
        sign by a centimetre.

        ``_waypoint_index`` is deliberately NOT re-seeked the way
        ``replace_path`` does. This transform is 1:1 and order-preserving --
        waypoint *i* of the lane path is waypoint *i* of the base path moved
        sideways by at most ``lateral_offset`` -- so the index still denotes
        the same point on the same lap, and re-seeking could only move it.
        """
        router = self._sign_router
        if router is None or not self._tuning.sign_router.SIGN_LANE_PLANNER:
            return
        fingerprint = router.lane_fingerprint
        if fingerprint == self._lane_fingerprint:
            return
        self._lane_fingerprint = fingerprint

        sr = self._tuning.sign_router
        previous = self._waypoints
        self._waypoints = apply_sign_lanes(
            self._lane_base_waypoints,
            router.lane_specs,
            SignLaneParams(
                lateral_offset=(chassis_half_diagonal_m() + TrafficSignSpecs.WIDTH / 2 + sr.SIGN_CLEARANCE_MARGIN_M)
                * sr.SIGN_LANE_OFFSET_FRAC,
                ramp_m=sr.SIGN_LANE_RAMP_M,
                hold_m=sr.SIGN_LANE_HOLD_M,
                skip_unsatisfiable=sr.SIGN_LANE_SKIP_UNSATISFIABLE,
                split_overlap=sr.SIGN_LANE_SPLIT_OVERLAP,
                corner_entry_m=sr.SIGN_LANE_CORNER_ENTRY_M,
            ),
            # The ROUTER's direction, not the navigator's: the lane must be
            # built on the same one the pass-side decision was made under.
            router.direction,
        )
        self._hold_committed_path(previous, sr.SIGN_LANE_COMMIT_AHEAD_M)
        self._apply_path_wall_budget()
        logger.info("Sign lanes replanned for %d sign(s)", len(fingerprint))

    def _sign_evade_steer(self, robot_x: float, robot_y: float, robot_yaw: float) -> float | None:
        """Steering to swing the chassis clear of a routed sign it is about to clip.

        PREDICTS the contact from geometry rather than waiting for the LIDAR to
        call it CRITICAL. That distinction is the whole mechanism: a return
        only reads CRITICAL at contact range, by which point the chassis is
        essentially already touching and no steering command can help -- which
        is exactly why reversing worked there and steering measured flat.
        Here the trigger is the sign's own along-track distance and lateral
        clearance, both of which are known metres in advance because the
        router is already tracking the sign's position.

        Returns ``None`` unless a routed sign is genuinely ahead, within
        ``SIGN_CONTACT_DIST_M``, and predicted to pass closer than the chassis
        and sign half-widths allow -- so a sign the robot is already clearing
        cleanly is never answered with a swerve.

        The direction comes from the sign's own bearing, not the router's
        pass-side rule. By this point the rule has failed; which side the robot
        ends up on is a scoring question, contact is a run-ending one. Choosing
        the side the robot is ALREADY on also makes this a smaller correction
        than forcing it back across.
        """
        router = self._sign_router
        if router is None:
            return None
        cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
        trigger = self._tuning.sign_router.SIGN_CONTACT_DIST_M
        # Half-widths, plus the chassis's own: how close the centres may pass.
        needed = RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2
        worst: tuple[float, float] | None = None
        for wp in router.routed_sign_positions:
            dx, dy = wp.x - robot_x, wp.y - robot_y
            ahead = dx * cos_yaw + dy * sin_yaw
            if not 0.0 < ahead <= trigger:
                continue
            lateral = -dx * sin_yaw + dy * cos_yaw
            if abs(lateral) >= needed:
                continue  # already going to clear it
            if worst is None or ahead < worst[0]:
                worst = (ahead, lateral)
        if worst is None:
            return None
        _, lateral = worst
        # Positive lateral puts the sign to the LEFT, so steer right. A sign
        # dead ahead (lateral 0) still has to be resolved to a side; take the
        # one the ordinary steering is already favouring.
        if lateral == 0.0:
            return None
        return -math.copysign(self._tuning.sign_router.sign_contact_steer_norm(), lateral)

    def _hold_committed_path(self, previous: list[Waypoint], commit_ahead_m: float) -> None:
        """Keep a lane rebuild from moving the path the chassis is already on.

        A lane ramps onto its offset over the approach, which assumes the
        rebuild happens before the robot reaches that stretch. Sighted runs
        satisfy that trivially -- the layout is known at t=0 and the path is
        built once. A discovering run does not: a sign first observed 1.5 m
        into a corridor triggers a rebuild whose ramp lies BEHIND the chassis,
        so the robot is instantly off a path it has no runway to rejoin.
        Measured on subset64 blind, 10% of rebuilds moved the path away from
        the chassis by up to 0.301 m -- essentially a whole lane offset.

        So the near field is pinned to what it already was. New information
        still bends the path, just ahead of the robot rather than underneath
        it. The near field catches up naturally on the next lap, when the same
        signs are already in the map and the rebuild happens far in advance.

        No-op at ``commit_ahead_m`` 0.0, and no-op for a sighted run either
        way, since nothing rebuilds after the first tick there.
        """
        if commit_ahead_m <= 0.0 or not previous or len(previous) != len(self._waypoints):
            return
        pose = self._gateway.get_current_pose()
        if pose is None:
            return
        cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
        held = list(self._waypoints)
        for i, (old, new) in enumerate(zip(previous, self._waypoints, strict=True)):
            if old == new:
                continue
            # Along-track distance in the chassis frame: negative is behind.
            ahead = (new.x - pose.x) * cos_yaw + (new.y - pose.y) * sin_yaw
            if ahead < commit_ahead_m:
                held[i] = old
        self._waypoints = held

    def replace_sign_router(self, sign_router: SignRouter | None) -> None:
        """Swap in a sign router built for a new race.

        Exists for the same reason as ``replace_park_controller``: the state
        machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING purely
        from the button, and a real (blind) run's active challenge is only
        known once the jumper is resolved -- which can differ from the
        previous race. A process built once as Open (``sign_router=None``)
        must be able to pick up a switch to Obstacles, and vice versa,
        without a restart. The previous router's own committed/discovered
        state does not carry over, same as ``ParkController`` above -- the
        caller builds a fresh one from the current section/direction/tuning.
        """
        self._sign_router = sign_router
        # Whatever lanes the previous router's layout produced belong to that
        # race. Drop back to the planned centreline and let the next tick
        # re-lane from the new router, if there is one.
        self._waypoints = list(self._lane_base_waypoints)
        self._lane_fingerprint = None

    def replace_park_controller(self, park_controller: ParkController | None) -> None:
        """Swap in a fresh ParkController ahead of a new race.

        ParkController's phase only moves forward (STAGE -> ENTER -> DONE),
        so a finished or timed-out one from the previous race can't be
        rewound in place -- the caller builds a new one from the same
        section/direction/metadata it used the first time and hands it here.
        """
        self._park_controller = park_controller
        self._parking_engaged = False

    def reset(self) -> None:
        """Clear per-race state so a new race starts as if this were the first.

        Needed because the state machine can cycle FINISHED -> BOOT_CHECK ->
        READY -> RACING purely from the physical button (two long presses and
        a short one), with no process restart -- so nothing else re-creates
        this object between races. Without this, ``_laps_completed`` alone
        would stay at its previous value and the very first tick of the new
        race would immediately read as already finished.
        """
        self._waypoint_index = 0
        self._laps_completed = 0
        self._suppress_next_wrap = False
        self._corner_latch.reset()
        # A fade left running across a reset would keep dragging the new race's
        # path back toward the previous one's geometry.
        self._blend_left = 0
        self._blend_from = []
        self._blend_to = []
        self._parking_engaged = False
        self._active_maneuver = None
        self._maneuver_frames_left = 0
        self._escape_count = 0
        self._escape_steer_sign = 1.0
        self._escape_sequence_start_xy = None
        self._stuck_detector.reset()
        self._waypoint_controller.reset()
        if self._sign_router is not None:
            self._sign_router.reset_for_new_lap()

    def replace_lap_detector(self, lap_detector: LapDetector) -> None:
        """Swap in a lap detector built for a different travel direction.

        The finish line's normal is the travel direction, so a detector built
        for the wrong one counts crossings with the sign inverted. Blind rounds
        infer the direction from LIDAR after they have started driving (see
        :mod:`src.navigation.direction_estimator`), so the detector that was
        provisional at startup has to be replaced once the answer arrives.

        Only valid before any lap has been counted, which is guaranteed here:
        the direction settles inside the first metre of travel, long before the
        finish line is re-crossed.
        """
        self._lap_detector = lap_detector

    def set_travel_direction(self, direction: Direction) -> None:
        """Adopt the (re-)inferred travel direction.

        Called alongside ``replace_lap_detector`` whenever inference settles or
        revises its answer. Consumed by the collision-avoidance escape maneuver
        as the fallback side when a LIDAR-only clearance comparison cannot
        decide one (see ``CollisionAvoidanceController._k_turn_steer_sign``).
        """
        self._direction = direction

    @property
    def laps_completed(self) -> int:
        """Number of laps confirmed completed so far."""
        return self._laps_completed

    def _base_debug(
        self, robot_x: float | None, robot_y: float | None, robot_yaw: float | None
    ) -> NavigatorDebugSnapshot:
        """Fields available on every phase once pose is known.

        These form the common prefix every ``step()`` branch's snapshot builds on.
        """
        return NavigatorDebugSnapshot(
            pose_x=robot_x,
            pose_y=robot_y,
            pose_yaw=robot_yaw,
            direction=self._direction,
            current_corridor=self._current_corridor,
            waypoint_index=self._waypoint_index,
            laps_completed=self._laps_completed,
            num_laps=self._num_laps,
            parking_engaged=self._parking_engaged if self._park_controller is not None else None,
        )

    def step(self) -> None:
        """Execute one control step.

        Pulls current state from the gateway, calculates commands,
        and pushes them back to the gateway.
        """
        pose = self._gateway.get_current_pose()
        if not pose:
            # No localisation available (startup or sensor dropout): stop rather
            # than coast on the last published command.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._debug = NavigatorDebugSnapshot(
                phase=NavigatorPhase.NO_POSE,
                commanded_speed_mps=0.0,
                commanded_steering_norm=0.0,
            )
            return

        robot_x, robot_y = pose.x, pose.y
        robot_yaw = pose.yaw

        self._current_corridor = corridor_for_position(robot_x, robot_y)

        # Breadcrumbs for a retrace-reverse. Recorded on every tick including
        # mid-maneuver, so the trail is a true record of where the chassis has
        # physically been -- which is the entire basis for reversing along it
        # without rear sensing. See _retrace_steer.
        if (
            not self._pose_trail
            or self._pose_trail[-1].to_waypoint().distance_to(Waypoint(robot_x, robot_y))
            >= self._tuning.escape.POSE_TRAIL_MIN_STEP_M
        ):
            self._pose_trail.append(Pose(robot_x, robot_y, robot_yaw))

        # Continue an in-progress escape maneuver until its latched duration
        # elapses, so escapes are real motions rather than single-tick pulses that
        # never clear the wall.
        if self._active_maneuver is not None:
            self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.ACTIVE_MANEUVER)
            return

        # Update stuck detector — runs while actively driving OR maneuvering
        # into the parking gap, but not once the robot has reached its final
        # deliberate stop (open-challenge hold, or parking done): otherwise a
        # robot correctly holding position at zero velocity would eventually
        # read as "stuck" and reverse itself back out of a completed park.
        # Also suspended while ParkController is mid reverse-and-reorient recovery
        # (see ParkController.is_repositioning) — that maneuver is itself a deliberate,
        # low-net-displacement reverse burst, and the generic escape it would otherwise
        # trigger is blind to the inner keep-out block ParkController is navigating around.
        # Reset (not just skip) during repositioning: otherwise the history queue still
        # spans across the gap, and the first check afterward compares a newest position
        # against an oldest one from well before the reposition started — reintroducing
        # the same false "stuck" trigger one tick later instead of preventing it.
        pc = self._park_controller
        if pc is not None and pc.is_repositioning:
            self._stuck_detector.reset()
        elif not self._is_holding():
            self._stuck_detector.update(Waypoint(robot_x, robot_y))
            if self._stuck_detector.is_stuck:
                self._handle_stuck_escape(robot_x, robot_y, robot_yaw)
                return

        # Lap completion: defer the parking handoff until the robot is actually
        # in the parking corridor and within reach of the staging point. Until
        # then keep navigating so the handoff never fires mid-corridor.
        if self._laps_completed >= self._num_laps and self._handle_finish(
            robot_x,
            robot_y,
            robot_yaw,
        ):
            return

        # Waypoint-wrap detection: signal LapDetector and reset index.
        if self._waypoint_index >= len(self._waypoints):
            self._waypoint_index = 0
            self._stuck_detector.reset()
            if self._suppress_next_wrap:
                # Seeded past the seam by replace_path, not driven — see there.
                self._suppress_next_wrap = False
            elif self._lap_detector is not None:
                self._lap_detector.notify_waypoint_wrapped()
            else:
                # Fallback: no geometric guard — count directly.
                self._laps_completed += 1
                logger.info("Lap %d complete (waypoint-only fallback)", self._laps_completed)
                if self._sign_router is not None:
                    self._sign_router.reset_for_new_lap()
                self._debug = self._base_debug(robot_x, robot_y, robot_yaw)
                self._debug.phase = NavigatorPhase.WAYPOINT_WRAP_FALLBACK
                return

        # Geometric lap counting (requires LapDetector).
        if (
            self._lap_detector is not None
            and self._current_corridor is not None
            and self._lap_detector.update(Waypoint(robot_x, robot_y), self._current_corridor)
        ):
            self._laps_completed += 1
            logger.info("Lap %d complete (geometric + waypoint confirmed)", self._laps_completed)
            if self._sign_router is not None:
                self._sign_router.reset_for_new_lap()

        # Re-plan the path onto its pass-side lanes if the routed sign layout
        # has changed since the last rebuild. Sighted rounds settle this on the
        # first tick (the layout comes from metadata and never moves); blind
        # ones re-enter it as discovery confirms and refines signs. Deliberately
        # at the top of the driving tick rather than inside the router: the
        # rebuilt path has to be in place before crosstrack, the lookahead gate
        # and the target search all read it, and the router only runs after
        # those. Blind discovery lands one tick later as a result, which is
        # 50 ms against a corridor-length lane transition.
        # Advance the replan fade BEFORE the lanes are laid and before anything
        # reads the path this tick, so crosstrack, the lookahead search and the
        # index advance all see one consistent centreline.
        self._advance_replan_blend()

        self._refresh_sign_lanes()

        raw_wp = self._waypoints[self._waypoint_index]

        # Advance past any waypoint the robot has already gone by — not just
        # one it happens to pass within waypoint_threshold of. A robot that
        # starts somewhere other than exactly on the planned centerline (e.g.
        # anywhere in an official starting zone) can curve past a waypoint
        # without ever entering that radius; left un-advanced, the lookahead
        # search below keeps re-targeting that same, increasingly stale point
        # long after the robot has passed it. Once far enough away, that
        # stale point can itself satisfy the search's lookahead-distance test
        # and get selected as the steering target even though it's now behind
        # the robot, spiraling the heading around to chase a point it already
        # passed instead of the path ahead.
        #
        # The comparison wraps around the seam, so the *last* waypoint gets the
        # same pass-by rescue as every other one. It used to stop at
        # ``index + 1 < len``, which left entering a MAIN_LOOP_REACHED_DISTANCE_M
        # circle as the only way past the final point — and a robot running
        # wider than that radius never gets past it, never wraps, and so never
        # completes a lap no matter how many times it drives the loop. Measured
        # on the 2026-08-06 counterclockwise round: crosstrack ran 0.26-0.51 m
        # against a 0.20 m radius, the index froze on the last waypoint at
        # t=90s, and the robot circled the mat for a further 7 minutes with the
        # lap count stuck at zero. Advancing to ``len(waypoints)`` here is the
        # same state reaching the last waypoint produces, and the wrap branch
        # at the top of the next tick is what turns it into a counted lap.
        # Obstacles-only extension: also advance past a waypoint that reads as
        # BEHIND the chassis in its own local frame (the same ahead/behind
        # test select_target_point uses), not just one where the next
        # waypoint tests strictly closer. A robot cutting a corner sharply
        # enough -- more steering authority than the polyline's spacing
        # assumed when it was generated -- can leave BOTH the current and the
        # next waypoint reading as farther away every tick (neither test
        # closes), even though local-frame ahead/behind already correctly
        # shows the chassis has swept past them. Left unrescued the index
        # freezes, select_target_point's own forward search starts from that
        # stale point and returns a distant "ahead" candidate the actual
        # (chord-cutting) trajectory never converges toward, and the chassis
        # clips the wall still chasing it -- root-caused on go_obstacles_0042
        # under the `wideonly` (85 deg steering) hardware profile, subset64.
        # Gated on sign_router presence (None for Open Challenge -- see
        # STALE_TARGET_RESCUE's docstring) and its own toggle, so Open
        # Challenge's waypoint-advance pipeline is untouched byte-for-byte.
        # ADVANCE_PAST_PASSED_WAYPOINT lifts the Obstacles-only gate above, on
        # hardware evidence that Open hits the identical failure the paragraph
        # describes. run_20260831_224600, first corner: the index froze on
        # waypoint 8 at (0.35, 1.00) for FOUR SECONDS while the chassis swept
        # past it, and pure pursuit -- correctly, for a target now behind and to
        # the right -- turned to go back for it. Heading ran 172 -> 151 -> 130
        # -> 107 -> 66 -> 14 deg: about 160 deg of rotation where the corner
        # needed 90, ending perpendicular to the corridor with forward clearance
        # collapsing 0.21 -> 0.11 m into the wall. The index then jumped 8 -> 11,
        # which is the re-seek finding a later point once the overshoot was
        # unrecoverable.
        #
        # The distance test cannot catch this by construction: overshoot a
        # waypoint and BOTH it and its successor recede every tick, so
        # `next_closer` never closes and the only remaining exit is entering a
        # 0.20 m circle the chassis has already left behind.
        #
        # Separate flag rather than reusing STALE_TARGET_RESCUE: that one is
        # namespaced under sign_router, was measured on Obstacles (2-5%, and
        # refuted there), and carries the sign_router presence check that makes
        # it structurally unavailable to Open. Same geometry, different
        # challenge, different evidence -- so a different switch.
        rescue_behind = (self._sign_router is not None and self._tuning.sign_router.STALE_TARGET_RESCUE) or (
            self._tuning.waypoints.ADVANCE_PAST_PASSED_WAYPOINT
        )
        cos_yaw, sin_yaw = (math.cos(robot_yaw), math.sin(robot_yaw)) if rescue_behind else (0.0, 0.0)
        count = len(self._waypoints)
        for _ in range(count):
            next_index = self._waypoint_index + 1
            next_wp = self._waypoints[next_index % count]
            next_closer = next_wp.distance_to(Waypoint(robot_x, robot_y)) < raw_wp.distance_to(
                Waypoint(robot_x, robot_y)
            )
            raw_behind = rescue_behind and ((raw_wp.x - robot_x) * cos_yaw + (raw_wp.y - robot_y) * sin_yaw <= 0)
            if not next_closer and not raw_behind:
                break
            self._waypoint_index = next_index
            if next_index >= count:
                # Seam crossed. Leave raw_wp on the final waypoint and let the
                # wrap branch count the lap next tick — walking on into the new
                # lap here would skip waypoints the wrap is about to rewind to.
                break
            raw_wp = next_wp

        # Get LIDAR scan from gateway
        scan = self._gateway.get_lidar_scan()
        front = (
            self._collision_controller.front_sector(scan.ranges_m, scan.angles_rad)
            if scan and self._tuning.clearance.FORWARD_NO_DATA_IS_DEGRADED
            else None
        )
        if scan and front is not None and not front.measured:
            # A scan arrived but NOTHING in the forward cone was a valid reading.
            # That is the degraded-sensor case below, not open road -- and it is
            # the more dangerous of the two, because no scan at all is obvious
            # while an all-invalid cone reports NO_DATA_RANGE_M and reads as 10 m
            # of clear track. Measured on hardware 2026-08-31: wedged against a
            # wall, this held 9.97 m and the robot drove into it at 0.24 m/s for
            # the rest of the round. Same answer as the no-LIDAR branch, because
            # it is the same statement about the sensor.
            forward_clearance = self._tuning.clearance.SLOW_DIST
            risk = RiskLevel.OBSTACLE
        elif scan:
            # Converted to a BUMPER gap once, here, rather than at each of the
            # comparisons below: the no-LIDAR fallback assigns a threshold value
            # to this same variable, so the two branches have to leave it in one
            # frame or the degraded path means something different from the
            # measured one.
            forward_clearance = bumper_gap_ahead(
                self._collision_controller.compute_forward_clearance(
                    scan.ranges_m,
                    scan.angles_rad,
                )
            )
            risk = self._collision_controller.assess_risk(scan.ranges_m, scan.angles_rad)
        else:
            # No LIDAR: a degraded sensor is not open road. Drive cautiously
            # (slow zone + non-SAFE risk) instead of blasting forward blind.
            forward_clearance = self._tuning.clearance.SLOW_DIST
            risk = RiskLevel.OBSTACLE

        # Two risk readings, deliberately: the RAW scan governs how fast the
        # robot may go, the MAPPED-OBSTACLE-MASKED scan governs whether the
        # escape maneuver fires.
        #
        # A sign the router is routing around is passed at ``lateral_offset``
        # centre-to-centre by design, which is inside CONTACT_DIST — so on the
        # raw scan the escape maneuver fires at every sign pass and reverses the
        # robot out of the very gap the planner aimed for, deciding the run
        # before the router's aim can matter. Withholding those returns from the
        # escape trigger alone keeps the split honest: the robot still slows
        # down for a sign (raw ``risk`` caps speed below), it just no longer
        # panics at one the planner is already handling. Walls and unmapped
        # returns are untouched in both readings.
        escape_ranges: np.ndarray | tuple[float, ...] | None = scan.ranges_m if scan else None
        escape_risk = risk
        if scan and self._sign_router is not None:
            escape_ranges = mask_mapped_obstacles(
                scan.ranges_m,
                scan.angles_rad,
                pose,
                self._sign_router.routed_sign_positions_by_corridor,
                self._tuning.sign_router.ESCAPE_MASK_RADIUS_M,
            )
            escape_risk = self._collision_controller.assess_risk(escape_ranges, scan.angles_rad)

        # Identity of the ray that verdict came from, recorded here rather than
        # reconstructed later: by the time a maneuver latches, the robot has
        # already been driven somewhere else and the geometry is gone.
        escape_trigger = (
            self._collision_controller.nearest_path_ray(escape_ranges, scan.angles_rad)
            if scan and escape_ranges is not None
            else None
        )

        # Check waypoint reached — against the *raw* planned point, not the
        # sign-deformed one: deformation only biases steering near a sign, it
        # must never stall path progression. A sign can pull the steering
        # target sideways by up to lateral_offset, so the robot's real
        # trajectory may never pass within waypoint_threshold of the deformed
        # point — checking that point would freeze waypoint_index indefinitely
        # while the sign stays engaged, corrupting every later tick's lookahead
        # search with a stale target.
        dist_to_wp = raw_wp.distance_to_xy(robot_x, robot_y)
        if dist_to_wp < self._waypoint_threshold:
            self._waypoint_index += 1
            self._debug = self._base_debug(robot_x, robot_y, robot_yaw)
            self._debug.phase = NavigatorPhase.WAYPOINT_REACHED
            self._debug.forward_clearance_m = forward_clearance
            self._debug.min_lidar_range_m = min(scan.ranges_m) if scan and scan.ranges_m else None
            self._debug.risk = risk
            self._debug.escape_risk = escape_risk
            return

        # Steer at a lookahead point, not directly at the (often much closer)
        # next waypoint — otherwise the lookahead distance is computed but
        # discarded, producing weave on straights and corner cutting (PP-1).
        #
        # Lookahead selection itself is gated on crosstrack error (how far off
        # the planned path the robot actually is), not forward LIDAR clearance
        # -- see WaypointController.select_lookahead's docstring for why that
        # mismatch produced slow, lazy cornering and weak centering on real
        # hardware once the steering law became curvature-based (2026-08-03).
        crosstrack = cross_track_error(self._waypoints, robot_x, robot_y)
        # Crosstrack alone arms the short lookahead only after a corner has
        # been missed; the path's own upcoming turn arms it on entry. See
        # select_lookahead's docstring for the hardware measurement behind it.
        turn_ahead = path_turn_ahead(
            self._waypoints,
            self._waypoint_index,
            self._tuning.pursuit.CORNER_PREVIEW_DISTANCE_M,
        )
        # The preview decays to zero once the chassis is INSIDE the arc, which
        # un-arms the short lookahead mid-corner -- and crosstrack cannot cover
        # for it there, because the robot is on the path and merely pointing
        # the wrong way. Hold the preview open until the turn it promised has
        # actually been driven. See CornerLatch for the hardware trace.
        turn_ahead = self._corner_latch.update(
            turn_ahead, robot_yaw, self._tuning.pursuit.CORNER_TURN_THRESHOLD_RAD
        )
        # A third preview signal alongside crosstrack/turn_ahead: crosstrack
        # is measured against the raw path, so it never rises during a sign
        # pass (the deformation biases the SEARCH's output, not the path the
        # search itself is judged against) -- see select_lookahead's
        # docstring. routed_sign_positions is read-only (no engage/pass
        # bookkeeping side effects, unlike _active_sign_candidates), so this
        # is safe to query before deform_waypoint runs later this tick.
        # Also restricted to signs actually AHEAD of the chassis along its own
        # heading -- routed_sign_positions has no direction filter, so
        # without this a not-yet-passed sign still alongside or just behind
        # the chassis (common right after a corridor label flips) forces the
        # short lookahead just as readily as a genuinely upcoming one,
        # disturbing tracking well past the sign pass this exists to fix.
        # Mirrors SignRouter._active_sign_candidates' own along-track check.
        sign_ahead = False
        if self._tuning.sign_router.SIGN_AWARE_LOOKAHEAD and self._sign_router is not None:
            cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
            sign_ahead = any(
                math.hypot(wp.x - robot_x, wp.y - robot_y) < self._tuning.sign_router.ACTIVATION_DIST_M
                and (wp.x - robot_x) * cos_yaw + (wp.y - robot_y) * sin_yaw > 0
                for wp in self._sign_router.routed_sign_positions
            )
        lookahead_distance = self._waypoint_controller.select_lookahead(crosstrack, turn_ahead, sign_ahead)
        # Full waypoint list, not a slice from _waypoint_index -- select_target_point
        # wraps the search around the lap itself now (see its docstring); slicing here
        # would cut that wraparound off right back out again.
        # select_target_point stays tuple-based -- scripts/bag/diag_bag_path_replay.py
        # replays it directly against a tuple-based path end to end, so retyping it
        # would ripple into that script's own internals rather than stopping at a
        # boundary -- so convert only at this call.
        steer_target = self._waypoint_controller.select_target_point(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            waypoints=[(wp.x, wp.y) for wp in self._waypoints],
            waypoint_index=self._waypoint_index,
            lookahead_distance=lookahead_distance,
        )

        # Apply sign routing to whichever point steering will actually chase —
        # deforming a raw-path *candidate* before the lookahead search picked
        # from it meant the search itself, not the sign, decided whether the
        # nudge ever reached steering (it almost never did: waypoints are
        # spaced well under the 0.20-0.40m lookahead, so the search kept
        # skipping past a single deformed candidate to a further, undeformed
        # one). Deforming the search's own output guarantees the bias is
        # exactly what gets steered toward, at full tapered strength whenever
        # that point is close to the sign.
        #
        # SIGN_LANE_PLANNER does not make this redundant, and the two do not
        # fight: the override REPLACES the target's lateral coordinate with an
        # absolute value derived from the sign, so with a lane in place it
        # re-commands the same line the lane already describes instead of
        # adding a second offset to it. The lane carries the chassis onto that
        # line over the corridor's straight; this holds it there through the
        # pass. SIGN_LANE_SUPPRESS_DEFORM exists to measure that claim rather
        # than assume it -- see its docstring for the numbers. The router is
        # CALLED either way regardless: it owns engage/pass bookkeeping, the
        # blind discovery ingest and routed_sign_positions (which the escape
        # mask reads), none of which the lane transform replaces.
        sign_deform_magnitude: float | None = None
        active_sign_count: int | None = None
        suppress_deform = (
            self._tuning.sign_router.SIGN_LANE_PLANNER and self._tuning.sign_router.SIGN_LANE_SUPPRESS_DEFORM
        )
        if self._sign_router is not None and self._current_corridor is not None:
            observations = self._gateway.get_vision_detections()
            raw_target = steer_target
            deformed = self._sign_router.deform_waypoint(
                waypoint=steer_target,
                robot_pos=(robot_x, robot_y),
                robot_yaw=robot_yaw,
                corridor=self._current_corridor,
                observations=observations,
            )
            if not suppress_deform:
                steer_target = deformed
            sign_deform_magnitude = math.hypot(deformed[0] - raw_target[0], deformed[1] - raw_target[1])
            active_sign_count = self._sign_router.active_sign_count

        # Get steering from waypoint controller
        steering_normalized, _, angle_error = self._waypoint_controller.compute_steering(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            target_waypoint=steer_target,
            crosstrack_error=crosstrack,
            tuning=self._tuning,
        )

        # Determine speed
        if forward_clearance < self._clearance.CONTACT_DIST:
            speed = self._speed.creep_mps()
        elif forward_clearance < self._clearance.SLOW_DIST:
            speed = self._speed.slow_mps()
        elif forward_clearance < self._clearance.MEDIUM_DIST:
            speed = self._speed.medium_mps()
        else:
            speed = self._speed.fast_mps()
        # Captured before the heading limiter, the envelope clamp and the risk
        # cap all fold into `speed`. Reporting the post-min value under this
        # name made the two debug fields satisfy final <= heading_speed by
        # construction, so every attribution read as "clearance bound it" or
        # "neither did" and the heading limiter looked innocent on 100% of
        # ticks while it was in fact the binding constraint on most of them.
        clearance_speed = speed

        # Never take a sharp turn at a speed the steering actuator can't keep
        # up with. The steering servo has a fixed slew rate (MAX_STEERING_RATE)
        # independent of how fast the chassis is moving, so a big required
        # heading correction taken at full speed demands a yaw rate the
        # actuator cannot track -- it saturates, overshoots, and oscillates
        # instead of settling (measured on real hardware 2026-08-03, see
        # docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md).
        # Clearance alone never catches this: a corner can have 0.50m+ of open
        # space ahead while still demanding a 90-180 deg correction.
        #
        # Only the CRAWL threshold reduces speed. Everything below it runs at
        # the ceiling, cornering included.
        #
        # This ladder briefly mirrored the clearance one, dropping through
        # medium and slow as heading error grew. Measured on hardware
        # 2026-08-09 that cost 33% of lap time (CW 134.9 s -> 179.3 s, CCW
        # 161.7 s -> 200.9 s, both past the 180 s round limit) for nothing:
        # ordinary cornering sits at 23-45 deg of heading error, so the two
        # middle rungs were not catching a dangerous case, they were taxing
        # every corner on the track. Corner speed was 0.117 m/s against a
        # 0.156 ceiling while forward clearance was 0.34-0.50 m and risk read
        # safe.
        #
        # The rung that earns its place is CRAWL. Below 57 deg the steering has
        # time to track the demand; past it the servo's fixed slew rate cannot
        # keep up (2026-08-03), and that is a real limit rather than a tax.
        #
        # The three intermediate thresholds this used to read were deleted with
        # it -- see HeadingErrorZones. Re-graduating means adding them back,
        # which should take a measurement that beats the times above.
        abs_error = abs(angle_error)
        if abs_error >= self._tuning.heading.CRAWL:
            heading_speed = self._speed.creep_mps()
        else:
            heading_speed = self._speed.fast_mps()
        speed = min(speed, heading_speed)

        # Bound the selected cruise speed by the configured envelope. MIN_MPS
        # and MAX_MPS read like hard limits on the robot and were enforced
        # nowhere: a profile setting FAST above MAX was simply obeyed.
        #
        # This used to be a no-op in the direction that mattered, because the
        # floor and the creep tier were the same number (both 0.05 m/s): the
        # clamp swallowed the creep tier whole, so lowering it changed nothing
        # and read as evidence that the tier did not matter. A validator on
        # SpeedControlParams now rejects that configuration outright.
        #
        # Deliberately applied HERE, to a zone speed that is always positive,
        # and not to the final command: clamping that up to the floor would turn
        # every legitimate stop (escape hand-off, park complete, blocked at both
        # ends) into a 0.05 m/s crawl the robot cannot be commanded out of.
        speed = min(max(speed, self._speed.min_mps()), self._speed.max_mps())

        # Never blast past a non-forward obstacle (e.g. a sign alongside the
        # robot) just because the path ahead is clear.
        if risk != RiskLevel.SAFE:
            speed = min(speed, self._speed.slow_mps())

        # Give the pursuit controller more time to close a sign-avoidance
        # offset. Neither clearance nor heading-error speed reacts to one:
        # a sign deformation biases the STEERING TARGET sideways without
        # necessarily shrinking forward LIDAR clearance or growing heading
        # error, so the ordinary ladders can leave the chassis at full speed
        # while it's still asymptotically closing a lateral offset -- traced
        # as a consistent ~6.5cm shortfall between the commanded line and the
        # chassis at the moment it draws level with the sign (subset64,
        # go_obstacles_0009/0011/0020/0046). Gated on the deformation the
        # router actually applied THIS tick, not proximity to a sign, so it
        # only fires while a correction is genuinely in flight.
        if (
            self._tuning.sign_router.SIGN_AWARE_SPEED
            and sign_deform_magnitude is not None
            and sign_deform_magnitude > self._tuning.sign_router.SIGN_DEFORM_SPEED_THRESHOLD_M
        ):
            speed = min(speed, self._speed.slow_mps())

        # Treat a discovering run's first lap as reconnaissance. The robot
        # cannot see a corridor's signs until it is inside that corridor (they
        # sit within corridors, and the next one is outside a 102 deg FOV until
        # the corner is turned), so on lap 1 avoidance is planned against
        # information that arrives ~1.5 m out -- while on later laps the same
        # signs are already mapped and the full runway is available. Measured
        # on subset64: 78% of blind failures happen during lap 1, 9% in lap 2,
        # none in lap 3. Slowing only that lap buys runway in TIME where runway
        # in DISTANCE cannot be had.
        #
        # Gated on is_discovering (never a sighted run) and on no lap having
        # been completed, so this costs nothing once the map exists.
        explore_frac = self._tuning.sign_router.EXPLORE_LAP_SPEED_FRAC
        if (
            explore_frac < 1.0
            and self._sign_router is not None
            and self._sign_router.is_discovering
            and self._laps_completed == 0
        ):
            speed = min(speed, self._speed.max_mps() * explore_frac)

        # First-lap corner caution, Open-Challenge-applicable (unlike the
        # sign-router explore-lap cap above, not gated on is_discovering --
        # Open Challenge has no sign router at all). A mixed-width corner's
        # PLANNED arc is safe by construction (verified 2026-08-28: clearance
        # to both outer walls never drops below what the straights already
        # have, see docs/internal/algorithms/open-challenge.md hardware cause
        # #6/§6 -- the arc-radius formula is not the bug), but real hardware
        # wedged at exactly this kind of corner anyway, which points at
        # CONTROL tracking error (understeer/trim, already independently
        # measured) eating the plan's margin, not the plan itself. Only the
        # first lap has never actually been driven, so slow there specifically
        # while approaching a corner (turn_ahead, already computed above for
        # lookahead selection) to buy the tracking loop more margin; costs
        # nothing on lap 2+ once the corner has been taken once for real.
        #
        # UNDER TEST 2026-08-30, and the hardware evidence points the other way.
        # Across four track runs the cap pins lap 1 to slow_mps (median commanded
        # 0.220 against 0.400 on later laps) and lap 1 carries DOUBLE the heading
        # error: |angle_error| p90 1.38 rad against 0.68. Lookahead and turn
        # preview are near-identical across laps, so speed is the one variable
        # that moves. Slowing appears to be making the corner worse, not safer --
        # consistent with a corner being a STEERING problem (a fixed servo slew
        # rate has to produce the same geometric turn over more ticks) rather
        # than a braking one. FIRST_LAP_CORNER_CAUTION exists to A/B exactly
        # that; see its docstring.
        # CORNER_CAUTION_ALL_LAPS lifts the lap-1 gate: the preview is available
        # on every lap and knows about a corner metres before the LIDAR does, so
        # restricting it to lap 1 leaves the predictive signal unused for
        # two-thirds of the race while the reactive clearance ladder does the
        # work alone.
        corner_previewed = self._tuning.waypoints.FIRST_LAP_CORNER_CAUTION and turn_ahead
        lap_applies = self._laps_completed == 0 or self._tuning.waypoints.CORNER_CAUTION_ALL_LAPS
        width_applies = (
            not self._tuning.waypoints.FIRST_LAP_CORNER_CAUTION_NARROW_ONLY or self._in_narrow_corridor()
        )
        if corner_previewed and lap_applies and width_applies:
            speed = min(speed, self._speed.corner_mps())

        # Snapshot everything decided so far -- both the escape-trigger branch
        # below and the normal publish at the end of this method share it, only
        # differing in phase and the final command actually sent.
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
        debug.forward_clearance_m = forward_clearance
        debug.min_lidar_range_m = min(scan.ranges_m) if scan and scan.ranges_m else None
        debug.risk = risk
        debug.escape_risk = escape_risk
        if escape_trigger is not None:
            debug.escape_trigger_angle_rad, debug.escape_trigger_range_m = escape_trigger
        debug.crosstrack_error_m = crosstrack
        debug.lookahead_distance_m = lookahead_distance
        debug.path_turn_ahead_rad = turn_ahead
        debug.steer_target_x = steer_target[0]
        debug.steer_target_y = steer_target[1]
        debug.angle_error_rad = angle_error
        debug.clearance_speed_mps = clearance_speed
        debug.heading_speed_mps = heading_speed
        debug.sign_deform_magnitude_m = sign_deform_magnitude
        debug.active_sign_count = active_sign_count

        # Last-resort geometric guard against clipping a routed sign. The two
        # responses that already exist both assume the planner has the sign
        # handled -- the escape mask suppresses any reaction to it, and without
        # that mask the generic escape reverses and swings, which in a 1.0 m
        # corridor trades sign strikes for wall strikes (measured, mask off:
        # sign 57 -> 41 but wall 0 -> 13). That assumption holds sighted, where
        # the lane is placed a corridor ahead, and fails on a blind first lap,
        # where the sign was only discovered ~1.5 m out.
        #
        # Deliberately NOT gated on LIDAR risk: a return reads CRITICAL only at
        # contact range, too late for any steering command to matter, which is
        # why a risk-gated version of this measured flat. The router already
        # knows where the sign is, so predict the clip instead.
        if self._tuning.sign_router.SIGN_CONTACT_EVADE and self._sign_router is not None:
            evade = self._sign_evade_steer(robot_x, robot_y, robot_yaw)
            if evade is not None:
                steering_normalized = max(-1.0, min(1.0, steering_normalized + evade))
                speed = min(speed, self._speed.creep_mps())

        # Escape maneuvers if critical — judged on the masked scan, so a mapped
        # sign cannot trigger one, and steered by the masked scan too: the
        # threat this escape is running from is by construction not the sign.
        if escape_risk == RiskLevel.CRITICAL and scan and escape_ranges is not None:
            escape_clearances = clearances_from_scan(
                LidarScan(ranges_m=tuple(escape_ranges), angles_rad=scan.angles_rad),
                self._collision_controller,
                front_half_fov_rad=self._collision_controller.threat_half_fov_rad,
                aggregate=ClearanceAggregate.MIN,
            )
            threat_dir = threat_direction(
                escape_clearances, self._collision_controller.threat_no_detection_range_m,
            )
            maneuver = self._collision_controller.compute_escape_maneuver(
                escape_risk,
                threat_dir,
                escape_ranges,
                scan.angles_rad,
                self._direction,
            )
            # Rear clearance is checked against the RAW scan: a sign behind the
            # robot is still something to not reverse into, whoever owns it.
            # Retrace instead of swinging, when asked and when there is enough
            # trail to aim at. Obstacles-only by construction: gated on
            # sign_router presence, which is None for Open Challenge, so its
            # escape behaviour is untouched regardless of the flag.
            # NOT gated on sign_router presence any more, so the Open Challenge
            # can reverse too. The gate was incidental rather than a dependency:
            # `_retrace_steer` reads only `_pose_trail`, and merely happens to
            # take its constants from the sign_router tuning group (those fields
            # belong in `escape` and should move).
            #
            # Open could not reverse AT ALL before this. The rear sector is
            # masked to a ~25 deg slot by mount occlusion, so when that slot
            # returns nothing `_reversing_into_unseen_wall` refuses the reverse
            # -- correctly failing closed -- and the robot falls through to a
            # full-lock pivot. Every hardware run this session logged "Reverse
            # escape refused: rear sector measured nothing".
            #
            # Retracing needs no rear sensor by construction: it backs along
            # ground the chassis occupied moments ago, which is known free
            # because the robot was just there. `RETRACE_ESCAPE` still gates it,
            # so this only makes the capability reachable, not automatic.
            self._retracing = bool(
                maneuver is not None
                and maneuver.speed < 0
                and self._tuning.sign_router.RETRACE_ESCAPE
                and self._retrace_steer(robot_x, robot_y, robot_yaw) is not None
            )
            if maneuver and self._reversing_into_unseen_wall(maneuver, scan):
                # Blocked at both ends: fall through to the capped creep-speed
                # publish below rather than backing into an unseen wall. The
                # stuck detector is the backstop if the robot truly can't move.
                maneuver = None
            if maneuver:
                if self._escape_count == 0:
                    self._escape_sequence_start_xy = (robot_x, robot_y)
                self._escape_count += 1
                self._begin_maneuver(self._maybe_escalate(maneuver))
                self._debug = debug
                # Hand over the snapshot rather than letting it be rebuilt: it
                # carries the risk verdict and the trigger ray that caused this
                # escape, and this is the only tick they can be attributed to.
                self._drive_active_maneuver(
                    robot_x, robot_y, robot_yaw, phase=NavigatorPhase.ESCAPE_TRIGGERED, base=debug,
                )
                return

        # Normal publish — clear the escape escalation, but only once the
        # robot has actually moved since the sequence started. A single
        # normal_drive tick between escape attempts doesn't mean the escape
        # worked -- confirmed on real hardware 2026-08-04 (run_20260804_213147):
        # normal_drive -> escape_triggered alternated for 34+ seconds with the
        # robot pinned in place, and this unconditional reset zeroed
        # escape_count every single cycle, so it never reached
        # ESCALATE_AFTER_ATTEMPTS and _maybe_escalate never fired. Requiring
        # real displacement first means a genuinely stuck sequence keeps
        # accumulating toward escalation instead of resetting on every tick
        # that merely classifies as "not critical" for one frame.
        if (
            self._escape_sequence_start_xy is None
            or math.hypot(
                robot_x - self._escape_sequence_start_xy[0],
                robot_y - self._escape_sequence_start_xy[1],
            )
            >= self._tuning.escape.STUCK_MOVE_THRESHOLD
        ):
            self._escape_count = 0
            self._escape_sequence_start_xy = None
        self._gateway.publish_drive(DriveCommand(speed_mps=speed, steering_norm=steering_normalized))
        debug.phase = NavigatorPhase.NORMAL_DRIVE
        debug.commanded_speed_mps = speed
        debug.commanded_steering_norm = steering_normalized
        debug.escape_count = self._escape_count
        self._debug = debug

    def _is_holding(self) -> bool:
        """True once the robot has reached a deliberate, terminal stop.

        Both terminal states are monotonic (never revert once reached), so it
        is safe to permanently stop running stuck detection once this is True.
        """
        if self._laps_completed < self._num_laps:
            return False
        pc = self._park_controller
        return pc is None or pc.is_done

    def _handle_finish(self, robot_x: float, robot_y: float, robot_yaw: float) -> bool:
        """Handle the post-final-lap phase.

        Returns True if a command was issued (caller should stop this tick);
        False if the robot should keep navigating toward the parking corridor.
        """
        pc = self._park_controller
        if pc is None:
            # Open challenge: no parking maneuver — hold position.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            debug = self._base_debug(robot_x, robot_y, robot_yaw)
            debug.phase = NavigatorPhase.FINISHED_HOLD
            debug.commanded_speed_mps = 0.0
            debug.commanded_steering_norm = 0.0
            self._debug = debug
            return True

        if not self._parking_engaged and self._should_engage_parking(robot_x, robot_y):
            logger.info("Parking engaged in corridor %s", self._current_corridor)
            self._parking_engaged = True

        if not self._parking_engaged:
            return False  # Keep navigating until at the staging point.

        if pc.is_done:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            debug = self._base_debug(robot_x, robot_y, robot_yaw)
            debug.phase = NavigatorPhase.FINISHED_HOLD
            debug.commanded_speed_mps = 0.0
            debug.commanded_steering_norm = 0.0
            self._debug = debug
            return True

        cmd = pc.update(Pose(robot_x, robot_y, robot_yaw))
        linear = cmd.linear
        scan = self._gateway.get_lidar_scan()
        if scan:
            # One call for both parking stop-check clearances (narrow-forward min
            # + full 360 deg sweep min) over the same scan.
            clearances = self._collision_controller.parking_clearances(scan.ranges_m, scan.angles_rad)
            fwd = clearances.forward_m
            side = clearances.sweep_m
            # CONTACT_DIST alone isn't a safe threshold here: the chassis extends up to
            # WIDTH/2 (0.10m) or LENGTH/2 (0.15m) from its centre depending on bearing, so a
            # raw range reading of CONTACT_DIST can already mean the footprint edge, not just
            # the sensor, has reached the obstacle. Pad by the chassis half-width so the gate
            # reacts while there's still real clearance left.
            #
            # NOTE (2026-08-21): this pad was written when rays were modelled as leaving the
            # chassis CENTRE. Since 6c727c87 they leave the LIDAR, 0.1222m forward and flush
            # with the bumper, so a FORWARD range now already excludes the front half of the
            # car and this pad is conservative there rather than necessary. Lateral bearings
            # are unchanged -- a forward shift does not alter perpendicular distance to a side
            # wall -- so the pad is still exactly right for them. Left in place deliberately:
            # re-tuning it belongs with the clearance recalibration, not with a geometry fix.
            #
            # This applies in both phases, including ENTER: a smaller pad there still let
            # the chassis clip a block edge in testing (the WRO-regulation gap is only
            # ~4cm wider than the chassis per side, tighter than this controller's approach
            # precision can reliably guarantee). Full padding means ENTER can stall short of
            # a clean park (see ParkController's own max_frames give-up) rather than thread
            # the gap in every case -- a known, documented limitation, not a silent one.
            # Not colliding takes priority over completing the maneuver.
            side_margin = self._clearance.CONTACT_DIST + RobotSpecs.WIDTH / 2
            if fwd < self._clearance.CONTACT_DIST or side < side_margin:
                linear = 0.0
        self._gateway.publish_drive(DriveCommand(speed_mps=linear, steering_norm=cmd.steering))
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
        debug.phase = NavigatorPhase.PARKING
        debug.park_phase = cmd.phase
        debug.commanded_speed_mps = linear
        debug.commanded_steering_norm = cmd.steering
        self._debug = debug
        return True

    def _should_engage_parking(self, robot_x: float, robot_y: float) -> bool:
        """Engage parking only once in the parking corridor and near staging."""
        pc = self._park_controller
        if pc is None:
            return False
        if self._current_corridor is not None and self._current_corridor != pc.section:
            return False
        sx, sy = pc.staging.x, pc.staging.y
        return math.hypot(sx - robot_x, sy - robot_y) < self._park_engage_dist
