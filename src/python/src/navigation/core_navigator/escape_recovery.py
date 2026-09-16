"""Escape and stuck-recovery subsystem for :class:`CoreNavigator`.

Extracted as a mixin so the recovery logic (retrace-reverse, K-turn escalation,
pivot-out-of-wedge, masked-scan escape arbitration) lives in its own module while
staying instance methods on ``CoreNavigator`` -- tests reach them as
``nav._maybe_escalate(...)`` and monkeypatch ``nav._begin_maneuver`` without
knowing the split exists. The state these methods read and write is owned by the
``CoreNavigator`` subclass; it is declared here only so type-checking can see it.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace
from typing import TYPE_CHECKING

from shared.domain.enums import NavigatorPhase
from shared.domain.models import NavigatorDebugSnapshot, Pose, Waypoint

from src.navigation.control.controllers import (
    EscapeManeuver,
    ManeuverType,
    bumper_gap_ahead,
    bumper_gap_behind,
)
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.utils import trail_clearance_behind

if TYPE_CHECKING:
    from collections import deque
    from collections.abc import Callable

    from shared.config.navigation_tuning import NavigationTuning
    from shared.config.navigation_tuning.escape import EscapeManeuverParams
    from shared.config.navigation_tuning.motion import ClearanceZones

    from src.navigation.control.controllers import (
        CollisionAvoidanceController,
        StuckDetector,
        WaypointController,
    )
    from src.navigation.ports import HardwareGateway

logger = logging.getLogger(__name__)


class EscapeRecovery:
    """Recovery behaviours shared into :class:`CoreNavigator` by inheritance."""

    # State owned by the CoreNavigator subclass; declared here for type-checking.
    _tuning: NavigationTuning
    # The per-challenge clearance zones resolved once in ``CoreNavigator.__init__``.
    # Read this, NOT ``_tuning.clearance``, for any CONTACT_DIST gate: on Obstacles
    # the two differ whenever ``OBSTACLES_CONTACT_DIST`` is set, and these gates are
    # the escape path the override exists to move. See
    # ``adr:0061-contact-zone-per-challenge``.
    _clearance: ClearanceZones
    # The per-challenge escape parameters resolved once in ``CoreNavigator.__init__``,
    # for the same reason and on the same discriminator as ``_clearance`` above.
    # Read this, NOT ``_tuning.escape``, for every escape value -- resolving only the
    # fields that obviously needed it is exactly what made the clearance override
    # diverge from its shared field, so there is one object and it is this one.
    _escape: EscapeManeuverParams
    _gateway: HardwareGateway
    _collision_controller: CollisionAvoidanceController
    _waypoint_controller: WaypointController
    _stuck_detector: StuckDetector
    _pose_trail: deque[Pose]
    _retracing: bool
    _active_maneuver: EscapeManeuver | None
    _maneuver_frames_left: int
    _escape_count: int
    _escape_steer_sign: float
    _escape_sequence_start_xy: tuple[float, float] | None
    _debug: NavigatorDebugSnapshot
    _base_debug: Callable[[float | None, float | None, float | None], NavigatorDebugSnapshot]

    def _retrace_steer(self, robot_x: float, robot_y: float, robot_yaw: float) -> float | None:
        """Steering that reverses the chassis back along ground it just occupied.

        A generic reverse escape backs along an ARC into space the robot has
        never been and largely cannot see; retracing needs no rear sensor by
        construction, because the chassis was physically standing on this ground
        seconds ago, and it cannot swing into a wall. Measured rationale and the
        refuted alternative: ``adr:0050-escape-steering-degrees-and-committed-side``
        and ``adr:0055-escape-maneuver-selection``.

        Reverse pure pursuit: curvature is the NEGATIVE of the forward case,
        since the vehicle rotates the other way for a given steer angle when
        travelling backwards. Returns ``None`` when the trail is too short to
        aim at, leaving the caller on its ordinary reverse.
        """
        target = self._trail_point_behind(robot_x, robot_y)
        if target is None:
            return None
        cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
        dx, dy = target.x - robot_x, target.y - robot_y
        along = dx * cos_yaw + dy * sin_yaw
        lateral = -dx * sin_yaw + dy * cos_yaw
        distance = target.to_waypoint().distance_to(Waypoint(robot_x, robot_y))
        if distance < self._escape.pose_trail_min_step_m or along > 0.0:
            # Target is not actually behind the chassis -- nothing to retrace.
            return None
        return self._tuning.sign_router.retrace_steer_gain_norm(-lateral / distance)

    def _trail_point_behind(self, robot_x: float, robot_y: float) -> Pose | None:
        """The breadcrumb roughly ``RETRACE_DIST_M`` back along the trail."""
        want = self._tuning.sign_router.retrace_dist_m
        travelled = 0.0
        previous = (robot_x, robot_y)
        for point in reversed(self._pose_trail):
            travelled += point.to_waypoint().distance_to(Waypoint(previous[0], previous[1]))
            previous = (point.x, point.y)
            if travelled >= want:
                return point
        return None

    def _reversing_into_unseen_wall(self, maneuver: EscapeManeuver, scan: LidarScan) -> bool:
        """True if executing ``maneuver`` would back into a wall behind the robot.

        Skipped while retracing: that maneuver reverses along ground the
        chassis just occupied, so it is known free without consulting a rear
        sector this hardware barely covers (and may not cover at all on the
        next chassis -- see ``_retrace_steer``).

        A rear sector with no valid rays counts as blocked, not clear. Reading
        the clearance alone fails open there, because ``compute_rear_clearance``
        reports the same 10 m for "nothing behind me" and "I cannot see behind
        me" -- the gate would wave the reverse through exactly when it is
        blindest. Refusing costs little: the caller falls through to a capped
        forward creep, with the stuck detector as the backstop.
        """
        if maneuver.speed >= 0 or self._retracing:
            return False
        rear = self._collision_controller.rear_sector(scan.ranges_m, scan.angles_rad)
        if not rear.measured:
            # No rear vision on this mount, but the pose trail records ground
            # the chassis physically occupied -- the same argument the retrace
            # exemption above already accepts, reached by escapes that are not
            # retraces. Evidence rather than a sensor: it cannot know what
            # moved in since, so it has to cover the WHOLE manoeuvre with
            # CONTACT_DIST to spare before it counts, and an empty trail still
            # refuses. Without this the gate is unreachable on a chassis with
            # no rear slot, and a scenario needing one escape-reverse hits the
            # wall instead (measured on go_open #85, 2026-08-22).
            if self._trail_confirms_reverse(
                reverse_distance=abs(maneuver.speed) * maneuver.duration_frames / self._tuning.control.control_hz
            ):
                return False
            logger.warning("Reverse escape refused: rear sector measured nothing")
            return True
        # As a gap from the REAR bumper. Compared raw until 2026-08-22, which
        # made this gate unreachable: the sensor is at the front, so an obstacle
        # touching the rear bumper reports ~0.272 m against a 0.10 m threshold
        # and the reverse was authorised right up to the moment of impact.
        return bumper_gap_behind(rear.min_range_m) < self._clearance.contact_dist

    def _fit_reverse_to_rear_gap(self, maneuver: EscapeManeuver, scan: LidarScan) -> EscapeManeuver:
        """Shorten a reversing escape to the rear room actually measured.

        ``_reversing_into_unseen_wall`` compares the gap only at the FIRST frame
        and then the manoeuvre runs its full latched duration, which comes from
        FRONT severity, so a reverse can be driven into the pillar behind. A
        CEILING, not a replacement: front severity still proposes, the rear room
        only caps, and a reverse that already fits comes back unchanged. See
        ``adr:0055-escape-maneuver-selection``.

        Two deliberate non-interventions: an unmeasured rear sector is left
        ALONE rather than capped to zero (authorisation is
        ``_reversing_into_unseen_wall``'s job), and a gap already inside
        ``CONTACT_DIST`` is left alone too (a refusal, not a truncation).

        Obstacles-only by configuration (``obstacles_k_turn_fit_rear_gap``):
        Open escapes fire in corners against walls, where a shortened reverse
        under-rotates and re-triggers, and nothing has measured that Open wants
        this.
        """
        if not self._escape.k_turn_fit_rear_gap or maneuver.speed >= 0 or self._retracing:
            # Retracing backs along ground the chassis physically occupied, so
            # its room is vouched for by the trail rather than by the rear
            # sector -- the same exemption ``_reversing_into_unseen_wall`` makes.
            return maneuver
        rear = self._collision_controller.rear_sector(scan.ranges_m, scan.angles_rad)
        if not rear.measured:
            return maneuver
        room = bumper_gap_behind(rear.min_range_m) - self._clearance.contact_dist
        if room <= 0.0:
            return maneuver
        per_frame = abs(maneuver.speed) / self._tuning.control.control_hz
        if per_frame <= 0.0:
            return maneuver
        fits = max(1, int(room / per_frame))
        if fits >= maneuver.duration_frames:
            return maneuver
        logger.info(
            "Reverse escape shortened to fit rear gap: %.3f m rear, %d -> %d frames",
            bumper_gap_behind(rear.min_range_m),
            maneuver.duration_frames,
            fits,
        )
        return replace(maneuver, duration_frames=fits)

    def _trail_confirms_reverse(self, reverse_distance: float) -> bool:
        """Whether the pose trail vouches for a reverse of ``reverse_distance``.

        The trail records ground the chassis physically occupied, so it is the
        one statement about the space behind that needs no rear sensor. It is
        evidence, not a reading: it cannot know what moved in since, so the
        ground must cover the WHOLE manoeuvre with ``CONTACT_DIST`` to spare.
        An empty trail refuses -- which is exactly the told-direction wedge
        behaviour wanted on a chassis with no rear slot. See
        ``_reversing_into_unseen_wall`` and the stuck-escape gate, both of which
        call this rather than trusting an unmeasured rear sector.
        """
        if not self._pose_trail:
            return False
        trail_x, trail_y, trail_yaw = self._pose_trail[-1]
        covered = trail_clearance_behind(self._pose_trail, trail_x, trail_y, trail_yaw)
        return covered is not None and covered >= reverse_distance + self._clearance.contact_dist

    def _begin_maneuver(self, maneuver: EscapeManeuver) -> None:
        """Latch an escape maneuver so it executes for its full duration."""
        self._active_maneuver = maneuver
        self._maneuver_frames_left = max(1, maneuver.duration_frames)
        # An escape maneuver drives steering directly, bypassing pure pursuit.
        # Clear the rate-limit memory so pure pursuit doesn't rate-limit its
        # first post-maneuver command against a stale pre-maneuver angle.
        self._waypoint_controller.reset()

    def _side_correction_blends(self) -> bool:
        """Whether the latched manoeuvre should BIAS the plan rather than replace it.

        Only a FORWARD side correction qualifies. A k-turn or a stuck reverse is
        a real manoeuvre that needs the chassis to itself, and a side correction
        that has switched to reverse (``already_touching``) is one too -- the
        planner has no model for backing off a wall it is already against.

        Ships OFF: adding two steering signals can saturate the wheel. See
        ``adr:0088-refuted-config-knobs``.
        """
        maneuver = self._active_maneuver
        return (
            maneuver is not None
            and self._escape.side_correction_blends
            and maneuver.maneuver_type is ManeuverType.SIDE_CORRECTION
            and maneuver.speed >= 0.0
        )

    def _take_side_correction_bias(self) -> tuple[float, float] | None:
        """Consume one tick of a blending side correction: (steering, speed).

        Counts the latch down here because ``_drive_active_maneuver`` -- which
        normally owns that countdown -- is deliberately not reached on this
        path. Without it the correction would latch forever and bias every
        subsequent tick.
        """
        if not self._side_correction_blends():
            return None
        maneuver = self._active_maneuver
        assert maneuver is not None  # noqa: S101 - narrowed by _side_correction_blends
        self._maneuver_frames_left -= 1
        if self._maneuver_frames_left <= 0:
            self._active_maneuver = None
            self._retracing = False
        return maneuver.steering, maneuver.speed

    def _drive_active_maneuver(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        phase: NavigatorPhase,
        base: NavigatorDebugSnapshot | None = None,
    ) -> None:
        """Publish the active escape command and count down its latched duration.

        ``base`` is the snapshot the caller has already filled in, used instead
        of building a fresh one. Rebuilding unconditionally discards the risk and
        the ray the escape verdict came from, so the ESCAPE_TRIGGERED tick
        published exactly the fields that would explain it as None. See
        ``adr:0056-raw-and-masked-scan``.

        Callers with nothing to carry pass None and get the old behaviour --
        the stuck paths reach here from branches that never computed a risk
        verdict, so for them a fresh snapshot is the honest one.
        """
        maneuver = self._active_maneuver
        if maneuver is None:
            return
        self._maneuver_frames_left -= 1
        if self._maneuver_frames_left <= 0:
            self._active_maneuver = None
            self._retracing = False
            # Re-anchor the escape sequence to where this manoeuvre ENDED.
            #
            # Anchored at the LATCH point, the manoeuvre satisfied the reset's
            # own movement test with its own travel, so every escape certified
            # itself as having worked and escalation could not fire. Measuring
            # from the END asks whether the robot made progress SINCE the escape,
            # rather than during it. See ``adr:0055-escape-maneuver-selection``.
            if self._escape_sequence_start_xy is not None:
                self._escape_sequence_start_xy = (robot_x, robot_y)
        # A retrace is re-aimed every tick, unlike a latched arc: the whole
        # point is to follow a path, and a single steering value fixed at
        # trigger time would describe an arc again after the first few
        # centimetres. Falls back to the latched steering the moment the trail
        # runs out, so this can only ever be as bad as the ordinary reverse.
        steering = maneuver.steering
        if self._retracing:
            retrace = self._retrace_steer(robot_x, robot_y, robot_yaw)
            if retrace is not None:
                steering = retrace
        maneuver = replace(maneuver, steering=steering)
        self._gateway.publish_drive(DriveCommand(speed_mps=maneuver.speed, steering_norm=maneuver.steering))
        debug = self._base_debug(robot_x, robot_y, robot_yaw) if base is None else base
        debug.phase = phase
        debug.active_maneuver_type = maneuver.maneuver_type
        debug.maneuver_steering = maneuver.steering
        debug.maneuver_speed_mps = maneuver.speed
        debug.maneuver_frames_left = self._maneuver_frames_left
        debug.escape_count = self._escape_count
        debug.commanded_speed_mps = maneuver.speed
        debug.commanded_steering_norm = maneuver.steering
        self._debug = debug

    def _escape_steer_sign_for_attempt(self, first_attempt: int = 1, start_sign: float | None = None) -> float:
        """Which side this escape attempt swings toward.

        Derived from ``_escape_count`` rather than flipped in place, so a side
        is held for ``escape_side_commit_attempts`` consecutive attempts before
        the other is tried. Flipping on every attempt (which all three escape
        paths used to do independently) means consecutive attempts rotate the
        chassis in opposite directions and undo each other. A wedge needs
        several attempts pushing the *same* way to accumulate; alternating
        guarantees they cannot. See ``adr:0055-escape-maneuver-selection``.

        ``_escape_steer_sign`` is the base side, not a running toggle -- the
        blocks alternate around it.

        Args:
            first_attempt: The ``_escape_count`` at which this caller's sequence
                begins, so its blocks line up with it. Anchoring every caller at
                1 instead leaves whichever attempt a caller actually starts on
                stranded mid-block, and a block of one is the alternating
                behaviour this exists to stop.
            start_sign: Side for the sequence's first block, defaulting to the
                base. Escalation passes the opposite, since switching sides is
                the point of escalating.
        """
        base = self._escape_steer_sign if start_sign is None else start_sign
        commit = max(1, self._escape.escape_side_commit_attempts)
        block = max(0, self._escape_count - first_attempt) // commit
        return base if block % 2 == 0 else -base

    def _pivot_steer_sign(self, scan: LidarScan | None) -> float:
        """Forward-travel steer sign that swings the nose toward the open side.

        Used by the rear-free stop-and-steer pivot, where the chassis is wedged
        front-and-back with no rear sensor to authorise a reverse. Forward travel
        swings the nose RIGHT for a positive command (``yaw_rate =
        (v/L)*tan(steer)`` with ``v > 0``), the opposite of reverse, so the sign
        must point the nose toward the *wider* side clearance, not mirror the
        reverse K-turn rule.

        A side with no valid return is the clearest possible "open" reading -- a
        wall-pinned chassis reads a close valid return on the jammed side and
        nothing on the free side -- so a missing side is treated as maximally
        open, never as a tie. Falls back to the committed escape side when LIDAR
        says nothing at all, so a pivot still happens rather than stalling.
        """
        if scan is None or scan.ranges_m is None:
            return self._escape_steer_sign_for_attempt()
        left = self._collision_controller.compute_min_clearance(
            scan.ranges_m, scan.angles_rad, center_rad=math.pi / 2, half_fov_rad=math.pi / 4
        )
        right = self._collision_controller.compute_min_clearance(
            scan.ranges_m, scan.angles_rad, center_rad=-math.pi / 2, half_fov_rad=math.pi / 4
        )
        no_data = self._tuning.lidar_sectors.no_data_range_m
        if left >= no_data and right >= no_data:
            return self._escape_steer_sign_for_attempt()
        # More open side wins; forward positive steer = nose right.
        if left > right:
            return -1.0
        if right > left:
            return 1.0
        return self._escape_steer_sign_for_attempt()

    def _stuck_escape_base_sign(self, scan: LidarScan | None) -> float:
        """Base steering side for a FRESH stuck-escape sequence.

        Swings toward whichever side LIDAR measures as clearer, falling back
        to the currently committed side when both are tied or unreadable.
        ``_escape_steer_sign_for_attempt``'s block-alternation still owns
        which side repeated attempts within the sequence take; this only
        fixes what side attempt 1 commits to.

        Measured on the 2026-09-10 Obstacles bags: this base was hardcoded to
        1.0 at every reset and NEVER read from LIDAR, so the stuck K-turn
        opposed the clearer side far more often than the one escape type that
        already read a threat direction. This closes that gap the same way: a
        left/right clearance comparison, seeded once per sequence rather than
        re-read every tick. See ``adr:0050-escape-steering-degrees-and-committed-side``.

        The clearer-side sign is numerically identical whether the escape
        that follows drives forward or reverses: Ackermann reverse flips
        which physical side a given steering SIGN swings the nose toward,
        but "aim the nose at the clearer side" is invariant to that flip --
        see ``_pivot_steer_sign`` (forward) and
        ``CollisionAvoidanceController._k_turn_steer_sign`` (reverse), which
        independently derive the same formula for their own cases. One
        comparison here serves both ``_handle_stuck_escape`` branches below.
        """
        if scan is None or scan.ranges_m is None:
            return self._escape_steer_sign
        left = self._collision_controller.compute_min_clearance(
            scan.ranges_m, scan.angles_rad, center_rad=math.pi / 2, half_fov_rad=math.pi / 4
        )
        right = self._collision_controller.compute_min_clearance(
            scan.ranges_m, scan.angles_rad, center_rad=-math.pi / 2, half_fov_rad=math.pi / 4
        )
        no_data = self._tuning.lidar_sectors.no_data_range_m
        if (left >= no_data and right >= no_data) or left == right:
            return self._escape_steer_sign
        return -1.0 if left > right else 1.0

    def _maybe_escalate(self, maneuver: EscapeManeuver) -> EscapeManeuver:
        """Escalate a repeated escape instead of repeating an identical pulse.

        After a few consecutive escapes that clearly aren't working, reverse for
        longer and swing toward the opposite side, so the robot stops slamming
        the same failing maneuver into the same wall.
        """
        if self._escape_count <= self._escape.escalate_after_attempts:
            return maneuver
        side = self._escape_steer_sign_for_attempt(
            first_attempt=self._escape.escalate_after_attempts + 1,
            start_sign=-self._escape_steer_sign,
        )
        steering = abs(maneuver.steering) * side if maneuver.steering else 0.0
        return replace(
            maneuver,
            steering=steering,
            duration_frames=min(maneuver.duration_frames * 2, self._escape.max_escape_frames(self._tuning.control.control_hz)),
        )

    def _handle_stuck_escape(self, robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Reverse out of a stuck state, but never back into an unseen wall.

        The reverse is latched for several frames (escalating with repeated
        attempts) and switches steering side only after committing to one for
        several attempts (see ``_escape_steer_sign_for_attempt``), so a
        wall-pinned robot actually backs away instead of twitching.

        When reverse itself is blocked (wedged both front and rear), the old
        code just held and reset the stuck detector forever, re-arming the same
        failed command; a forward creep at full steering lock (same side-commit
        and escalation pattern) instead walks the nose clear. See
        ``adr:0055-escape-maneuver-selection``.
        """
        logger.warning("Robot stuck - triggering escape")
        stuck_diag = self._stuck_detector.get_diagnostics()
        # "Not blocked" sentinel for the no-scan-yet case below, reusing
        # lidar_sectors.no_data_range_m rather than a second independent
        # magic 10.0 -- both mean the same thing: no valid reading, so
        # assume clear rather than blocked.
        rear_clear = self._tuning.lidar_sectors.no_data_range_m
        forward_clear = self._tuning.lidar_sectors.no_data_range_m
        rear_blind = False
        forward_blind = False
        scan = self._gateway.get_lidar_scan()
        if scan:
            # A rear sector that measured nothing reports the same 10 m as a
            # genuinely empty one, so the distance alone cannot tell them
            # apart. Tracked separately rather than folded into rear_clear so
            # the two stay distinguishable below (and in the log line).
            rear = self._collision_controller.rear_sector(scan.ranges_m, scan.angles_rad)
            rear_blind = not rear.measured
            # Both ends as BUMPER gaps, so the single CONTACT_DIST below means
            # the same thing in each direction. Compared raw, it did not: the
            # sensor is at the front, so the rear test needed an obstacle 17 cm
            # inside the chassis before it would trip. The NO_DATA sentinel
            # survives the conversion -- 10 m less either datum is still open
            # road -- so the no-scan branch keeps its "assume clear" meaning.
            rear_clear = bumper_gap_behind(rear.min_range_m)
            # The front needs the SAME distinction the rear has above, and for
            # the same reason: an unreadable forward cone reports NO_DATA_RANGE_M
            # and is indistinguishable from open road, so choosing STUCK_FORWARD
            # from it escapes INTO a wall the robot is already touching. See
            # ``adr:0056-raw-and-masked-scan``.
            front = self._collision_controller.front_sector(scan.ranges_m, scan.angles_rad)
            forward_blind = not front.measured
            forward_clear = bumper_gap_ahead(front.min_range_m)
        # Blind behind is a reason to prefer forward, but only when forward is
        # actually open. Treating it as flatly "blocked" would leave a chassis
        # with no rear vision at all frozen in every corner where both ends
        # read blocked; there, an unseen reverse is still the better of two
        # bad options -- but only when the pose trail vouches for it. A blind
        # rear with NO trail is the exact "cannot see behind" case, and the
        # fall-through below would reverse into whatever moved in since; that
        # must hold instead. The same evidence argument as
        # _reversing_into_unseen_wall (ground the chassis occupied), applied to
        # the worst-case stuck reverse so a longer escalation cannot outrun it.
        #
        # Every forward-forcing term below is gated on `rear_blind`, so none of
        # this is the normal path any more: the rear slot was re-measured on
        # 2026-08-31 (~40 deg at +/-160..180, wedges -155..-120 / 120..160) and
        # `rear_blind` is now usually False, leaving only the genuine
        # `rear_clear < CONTACT_DIST` case. Measured over five previously
        # colliding scenarios, "rear sector measured nothing" refusals went
        # 54 -> 0. The degraded reasoning above is kept deliberately -- it is
        # what runs when the rear IS occluded, which is still every bearing
        # inside the wedges.
        stuck_reverse_distance = (
            abs(self._escape.rev_speed) * self._escape.max_escape_frames(self._tuning.control.control_hz) / self._tuning.control.control_hz
        )
        blind_rear_unconfirmed = rear_blind and not self._trail_confirms_reverse(
            reverse_distance=stuck_reverse_distance
        )
        # "Forward is open" must mean MEASURED open, not merely a large number.
        # Without the second term a blind forward cone reads 10 m and every test
        # below passes, which is how the robot came to escape forward into a wall
        # it was touching. Gated so the flag alone decides whether this is live.
        forward_open = forward_clear >= self._clearance.contact_dist and not (
            self._tuning.clearance.forward_no_data_is_degraded and forward_blind
        )
        if (
            rear_clear < self._clearance.contact_dist
            or (rear_blind and forward_open)
            or blind_rear_unconfirmed
        ):
            if forward_open:
                logger.warning(
                    "Stuck escape: rear %s (%.2f m), forward clear (%.2f m) - forcing forward escape",
                    "unseen" if rear_blind else "blocked",
                    rear_clear,
                    forward_clear,
                )
                if self._escape_count == 0:
                    self._escape_sequence_start_xy = (robot_x, robot_y)
                    self._escape_steer_sign = self._stuck_escape_base_sign(scan)
                self._escape_count += 1
                frames = min(
                    self._escape.k_turn_min_frames(self._tuning.control.control_hz)
                    + self._escape.stuck_escalation_per_attempt_frames(self._tuning.control.control_hz) * (self._escape_count - 1),
                    self._escape.max_escape_frames(self._tuning.control.control_hz),
                )
                steering = self._escape.rev_steer_norm() * self._escape_steer_sign_for_attempt()
                self._begin_maneuver(
                    EscapeManeuver(
                        maneuver_type=ManeuverType.STUCK_FORWARD,
                        steering=steering,
                        speed=self._tuning.speed.escape_nudge_mps(),
                        duration_frames=frames,
                    ),
                )
                self._stuck_detector.reset()
                self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
                self._debug.is_stuck = bool(stuck_diag["is_stuck"])
                self._debug.stuck_count = int(stuck_diag["stuck_count"])
                self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
                self._debug.rear_clearance_m = rear_clear
                self._debug.forward_clearance_m = forward_clear
                return
            # Both ends blocked and rear unmeasurable (the current build carries
            # no rear slot -- see rear_sector_no_longer_available_2026_08_22):
            # a frozen hold used to deadlock here, re-arming the same failed
            # command every stuck window until the run timed out (27 s frozen on
            # real hardware 2026-08-04; and the dominant term in Obstacles'
            # 19->132 timeout rise after the LIDAR fix). The safe, rear-free
            # recovery is a LOW-SPEED PIVOT forward -- never a reverse, since the
            # rear gate cannot authorise one without a sensor -- steering toward
            # the more open side so the chassis reorients out of the wedge
            # instead of sitting in it. Forward creep, not zero speed: it walks
            # itself clear using decisive steering the pure-pursuit path would
            # not command for this geometry.
            steer_sign = self._pivot_steer_sign(scan)
            if self._escape_count == 0:
                self._escape_sequence_start_xy = (robot_x, robot_y)
            self._escape_count += 1
            frames = min(
                self._escape.k_turn_min_frames(self._tuning.control.control_hz)
                + self._escape.stuck_escalation_per_attempt_frames(self._tuning.control.control_hz) * (self._escape_count - 1),
                self._escape.max_escape_frames(self._tuning.control.control_hz),
            )
            logger.warning(
                "Stuck escape both-blocked: rear %.2f m, forward %.2f m - stop-and-steer pivot (sign %.1f)",
                rear_clear,
                forward_clear,
                steer_sign,
            )
            self._begin_maneuver(
                EscapeManeuver(
                    maneuver_type=ManeuverType.STUCK_FORWARD,
                    steering=self._escape.rev_steer_norm() * steer_sign,
                    speed=self._tuning.speed.escape_nudge_mps(),
                    duration_frames=frames,
                )
            )
            self._stuck_detector.reset()
            self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
            self._debug.is_stuck = bool(stuck_diag["is_stuck"])
            self._debug.stuck_count = int(stuck_diag["stuck_count"])
            self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
            self._debug.rear_clearance_m = rear_clear
            self._debug.forward_clearance_m = forward_clear
            return

        if self._escape_count == 0:
            self._escape_sequence_start_xy = (robot_x, robot_y)
            self._escape_steer_sign = self._stuck_escape_base_sign(scan)
        self._escape_count += 1
        frames = min(
            self._escape.k_turn_min_frames(self._tuning.control.control_hz)
            + self._escape.stuck_escalation_per_attempt_frames(self._tuning.control.control_hz) * (self._escape_count - 1),
            self._escape.max_escape_frames(self._tuning.control.control_hz),
        )
        steering = self._escape.rev_steer_norm() * self._escape_steer_sign_for_attempt()
        # The reverse leg curves the OPPOSITE way, so its rotation adds to the
        # forward leg's instead of undoing it. Without this the two legs of a
        # k-turn hold the same lock and retrace one another -- the bay's
        # pendulum, measured at 85.6% of leg pairs on the open track. See
        # escape_mirrors_reverse; ships off.
        if self._escape.escape_mirrors_reverse:
            steering = -steering
        self._begin_maneuver(
            EscapeManeuver(
                maneuver_type=ManeuverType.STUCK_REVERSE,
                steering=steering,
                speed=self._escape.rev_speed,
                duration_frames=frames,
            ),
        )
        self._stuck_detector.reset()
        self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
        self._debug.is_stuck = bool(stuck_diag["is_stuck"])
        self._debug.stuck_count = int(stuck_diag["stuck_count"])
        self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
        self._debug.rear_clearance_m = rear_clear
