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
from typing import TYPE_CHECKING, Callable

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

    from shared.config.navigation_tuning import NavigationTuning

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
        never been and, on this chassis, largely cannot see: the rear sector is
        already masked from -160..-115 deg and +115..+175 deg by mount
        occlusion, leaving a ~25 deg slot straight back as the only rear vision
        there is. Two consequences, and both argue for retracing instead:

        * That slot may not exist on the next chassis at all. If it goes, the
          rear sector has no valid rays and ``compute_rear_clearance`` reports
          the same ``NO_DATA_RANGE_M`` (10 m) it reports for open road;
          ``_reversing_into_unseen_wall`` now refuses that case outright, so
          the gate fails closed -- but a refused reverse is a robot that isn't
          escaping, not a robot that escaped safely.
        * The arc is what produces the wall strikes. Measured blind with the
          escape mask off, sign collisions fall 57 -> 41 but wall collisions
          rise 0 -> 13, in a corridor only 1.0 m wide.

        Retracing needs no rear sensor by construction: the chassis was
        physically standing on this ground seconds ago, so it is free unless
        something moved into it, and nothing on this track does. It also cannot
        swing into a wall, because it follows a path already driven rather than
        an arc into the unknown.

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
        if distance < self._tuning.escape.POSE_TRAIL_MIN_STEP_M or along > 0.0:
            # Target is not actually behind the chassis -- nothing to retrace.
            return None
        return self._tuning.sign_router.retrace_steer_gain_norm(-lateral / distance)

    def _trail_point_behind(self, robot_x: float, robot_y: float) -> Pose | None:
        """The breadcrumb roughly ``RETRACE_DIST_M`` back along the trail."""
        want = self._tuning.sign_router.RETRACE_DIST_M
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
                reverse_distance=abs(maneuver.speed) * maneuver.duration_frames / self._tuning.control.CONTROL_HZ
            ):
                return False
            logger.warning("Reverse escape refused: rear sector measured nothing")
            return True
        # As a gap from the REAR bumper. Compared raw until 2026-08-22, which
        # made this gate unreachable: the sensor is at the front, so an obstacle
        # touching the rear bumper reports ~0.272 m against a 0.10 m threshold
        # and the reverse was authorised right up to the moment of impact.
        return bumper_gap_behind(rear.min_range_m) < self._tuning.clearance.CONTACT_DIST

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
        return covered is not None and covered >= reverse_distance + self._tuning.clearance.CONTACT_DIST

    def _begin_maneuver(self, maneuver: EscapeManeuver) -> None:
        """Latch an escape maneuver so it executes for its full duration."""
        self._active_maneuver = maneuver
        self._maneuver_frames_left = max(1, maneuver.duration_frames)
        # An escape maneuver drives steering directly, bypassing pure pursuit.
        # Clear the rate-limit memory so pure pursuit doesn't rate-limit its
        # first post-maneuver command against a stale pre-maneuver angle.
        self._waypoint_controller.reset()

    def _drive_active_maneuver(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        phase: NavigatorPhase,
    ) -> None:
        """Publish the active escape command and count down its latched duration."""
        maneuver = self._active_maneuver
        if maneuver is None:
            return
        self._maneuver_frames_left -= 1
        if self._maneuver_frames_left <= 0:
            self._active_maneuver = None
            self._retracing = False
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
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
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
        is held for ``ESCAPE_SIDE_COMMIT_ATTEMPTS`` consecutive attempts before
        the other is tried. Flipping on every attempt (which all three escape
        paths used to do independently) means consecutive attempts rotate the
        chassis in opposite directions and undo each other: measured on real
        hardware 2026-08-05 (run_20260805_200011) as four escalating escapes
        over 40 s that rocked the yaw between -0.4 and -0.8 rad and translated
        the robot exactly nowhere. Escaping a wedge needs several attempts
        pushing the *same* way to accumulate; alternating guarantees they
        cannot.

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
        commit = max(1, self._tuning.escape.ESCAPE_SIDE_COMMIT_ATTEMPTS)
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
        no_data = self._tuning.lidar_sectors.NO_DATA_RANGE_M
        if left >= no_data and right >= no_data:
            return self._escape_steer_sign_for_attempt()
        # More open side wins; forward positive steer = nose right.
        if left > right:
            return -1.0
        if right > left:
            return 1.0
        return self._escape_steer_sign_for_attempt()

    def _maybe_escalate(self, maneuver: EscapeManeuver) -> EscapeManeuver:
        """Escalate a repeated escape instead of repeating an identical pulse.

        After a few consecutive escapes that clearly aren't working, reverse for
        longer and swing toward the opposite side, so the robot stops slamming
        the same failing maneuver into the same wall.
        """
        if self._escape_count <= self._tuning.escape.ESCALATE_AFTER_ATTEMPTS:
            return maneuver
        side = self._escape_steer_sign_for_attempt(
            first_attempt=self._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1,
            start_sign=-self._escape_steer_sign,
        )
        steering = abs(maneuver.steering) * side if maneuver.steering else 0.0
        return replace(
            maneuver,
            steering=steering,
            duration_frames=min(maneuver.duration_frames * 2, self._tuning.escape.MAX_ESCAPE_FRAMES),
        )

    def _handle_stuck_escape(self, robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Reverse out of a stuck state, but never back into an unseen wall.

        The reverse is latched for several frames (escalating with repeated
        attempts) and switches steering side only after committing to one for
        several attempts (see ``_escape_steer_sign_for_attempt``), so a
        wall-pinned robot actually backs away instead of twitching one
        centimetre every few seconds forever.

        When reverse itself is blocked (wedged both front and rear -- a real
        corner, or a moderate turn normal_drive's own curvature-based
        steering isn't decisive enough to complete at creep speed), this used
        to just hold and reset the stuck detector, over and over, forever:
        confirmed on real hardware 2026-08-04 as a robot frozen at the same
        position for 27s straight, is_stuck firing repeatedly and each time
        just re-arming the same forward command that had already failed for
        the previous window (see docs/known-issues-backlog.md). Holding is
        only actually the safe choice when forward is *also* blocked; when
        it isn't, a forward creep at full steering lock (same side-commit and
        escalation pattern as the reverse case) gives the
        robot a real chance to walk itself clear using more decisive
        steering than normal_drive's own pure-pursuit curvature was willing
        to command for this geometry.
        """
        logger.warning("Robot stuck - triggering escape")
        stuck_diag = self._stuck_detector.get_diagnostics()
        # "Not blocked" sentinel for the no-scan-yet case below, reusing
        # lidar_sectors.NO_DATA_RANGE_M rather than a second independent
        # magic 10.0 -- both mean the same thing: no valid reading, so
        # assume clear rather than blocked.
        rear_clear = self._tuning.lidar_sectors.NO_DATA_RANGE_M
        forward_clear = self._tuning.lidar_sectors.NO_DATA_RANGE_M
        rear_blind = False
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
            forward_clear = bumper_gap_ahead(
                self._collision_controller.compute_forward_clearance(
                    scan.ranges_m,
                    scan.angles_rad,
                )
            )
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
        stuck_reverse_distance = (
            abs(self._tuning.escape.REV_SPEED) * self._tuning.escape.MAX_ESCAPE_FRAMES / self._tuning.control.CONTROL_HZ
        )
        blind_rear_unconfirmed = rear_blind and not self._trail_confirms_reverse(
            reverse_distance=stuck_reverse_distance
        )
        if (
            rear_clear < self._tuning.clearance.CONTACT_DIST
            or (rear_blind and forward_clear >= self._tuning.clearance.CONTACT_DIST)
            or blind_rear_unconfirmed
        ):
            if forward_clear >= self._tuning.clearance.CONTACT_DIST:
                logger.warning(
                    "Stuck escape: rear %s (%.2f m), forward clear (%.2f m) - forcing forward escape",
                    "unseen" if rear_blind else "blocked",
                    rear_clear,
                    forward_clear,
                )
                if self._escape_count == 0:
                    self._escape_sequence_start_xy = (robot_x, robot_y)
                self._escape_count += 1
                frames = min(
                    self._tuning.escape.K_TURN_MIN_FRAMES
                    + self._tuning.escape.STUCK_ESCALATION_FRAMES_PER_ATTEMPT * (self._escape_count - 1),
                    self._tuning.escape.MAX_ESCAPE_FRAMES,
                )
                steering = self._tuning.escape.rev_steer_norm() * self._escape_steer_sign_for_attempt()
                self._begin_maneuver(
                    EscapeManeuver(
                        maneuver_type=ManeuverType.STUCK_FORWARD,
                        steering=steering,
                        speed=self._tuning.speed.creep_mps(),
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
                self._tuning.escape.K_TURN_MIN_FRAMES
                + self._tuning.escape.STUCK_ESCALATION_FRAMES_PER_ATTEMPT * (self._escape_count - 1),
                self._tuning.escape.MAX_ESCAPE_FRAMES,
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
                    steering=self._tuning.escape.rev_steer_norm() * steer_sign,
                    speed=self._tuning.speed.creep_mps(),
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
        self._escape_count += 1
        frames = min(
            self._tuning.escape.K_TURN_MIN_FRAMES
            + self._tuning.escape.STUCK_ESCALATION_FRAMES_PER_ATTEMPT * (self._escape_count - 1),
            self._tuning.escape.MAX_ESCAPE_FRAMES,
        )
        steering = self._tuning.escape.rev_steer_norm() * self._escape_steer_sign_for_attempt()
        self._begin_maneuver(
            EscapeManeuver(
                maneuver_type=ManeuverType.STUCK_REVERSE,
                steering=steering,
                speed=self._tuning.escape.REV_SPEED,
                duration_frames=frames,
            ),
        )
        self._stuck_detector.reset()
        self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
        self._debug.is_stuck = bool(stuck_diag["is_stuck"])
        self._debug.stuck_count = int(stuck_diag["stuck_count"])
        self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
        self._debug.rear_clearance_m = rear_clear
