"""Unit tests for CoreNavigator's escape-maneuver arbitration.

Uses a minimal fake HardwareGateway (implements the HardwareGateway Protocol
structurally) to drive CoreNavigator.step() directly, without ROS2 or the
simulator, so these scenarios can be pinned exactly: a wall dead ahead AND
dead behind must never produce a reversing command -- instead it must
recover with a low-speed forward pivot (stop-and-steer) toward the open side.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import NavigatorPhase, Section
from shared.domain.models import Detection, IMUReading, Pose, SignColor, Waypoint

from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.control.controllers import EscapeManeuver, ManeuverType
from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.utils import wrap_angle
from tests.fixtures import FakeGateway, create_scan_with_sectors
from tests.test_constants import (
    ANGLES_FULL_ROTATION,
    LIDAR_DEFAULT_FAR,
    NUM_RAYS,
    SIDE_SECTOR_INDICES,
)

ANGLES = ANGLES_FULL_ROTATION.tolist()


def seed_straight_pose_trail(nav: CoreNavigator, length_m: float = 1.0, spacing_m: float = 0.01) -> None:
    """Record ``length_m`` of breadcrumbs directly behind the chassis.

    The escape tests drive a robot with a single step and no history, so its
    pose trail is empty -- exactly the state that must refuse a reverse (you
    cannot evidence ground you have not occupied). These tests pin that a
    FRONT-only threat still reverses; that is true on track, where the trail is
    full, so they seed one behind the robot's ACTUAL pose (some of these tests
    anchor off-origin). Without this the reverse gate is testing the wrong state
    (empty-trail refusal) rather than the reversing logic. See
    ``trail_clearance_behind`` and
    adr:0055-escape-maneuver-selection.
    """
    pose = nav._gateway.pose
    count = int(length_m / spacing_m)
    # Chassis faces +x in all these fixtures (yaw=0), so "behind" is -x.
    nav._pose_trail.extend(Pose(pose.x - (count - i) * spacing_m, pose.y, pose.yaw) for i in range(count + 1))


@pytest.fixture()
def waypoints() -> list[Waypoint]:
    return [Waypoint(5.0, 0.0), Waypoint(10.0, 0.0)]  # far ahead — never reached in these tests


class TestCriticalEscapeRearGate:
    def test_wedged_both_ends_never_reverses(self, waypoints, tuning):
        """Front AND rear blocked: the escape must not command a reverse.

        Rear distance (0.09 m) is deliberately beyond the self-detection radius
        (0.08 m, see CA-4) so this is a genuine wall, not a filtered chassis
        reflection — otherwise the rear-gate would never see it as blocked.
        """
        ranges = create_scan_with_sectors(front=0.06, back=REAR_BLOCKED_M)
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav.step()

        assert gateway.commands, "expected a published command"
        assert gateway.commands[-1].speed_mps >= 0, "must not reverse into an unseen rear wall"

    def test_front_blocked_rear_clear_reverses(self, waypoints, tuning):
        """Front blocked, rear clear: the K-turn escape should reverse."""
        ranges = create_scan_with_sectors(front=0.06)
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)
        seed_straight_pose_trail(nav)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0, "front-only threat should trigger the reverse K-turn"

    def test_rear_blind_and_trail_less_creeps_forward_not_reverse(self, waypoints, tuning):
        """The degraded default, pinned on purpose.

        With the rear unmeasurable AND a single step carrying no pose trail,
        there is no evidence behind at all. Reversing there is reversing blind
        into whatever moved in since -- the documented fallback is a capped
        forward creep, and with no trail that is the RIGHT call, not a
        regression. This asserts the fallback rather than letting it be an
        incidental side effect of the empty-trail refusal above.

        The rear is blinded explicitly rather than by assuming the mount cannot
        see behind. ``create_scan_with_sectors`` defaults every ray to 10 m, so
        once the blind wedges narrowed the rear became readable and this scan
        stopped describing a rear-blind robot at all. A self-detection return
        is what an occluded bearing actually reports, so that is what the
        sector is given here. See adr:0056-raw-and-masked-scan.
        """
        # Built by bearing rather than via create_scan_with_sectors(back=...),
        # whose named sectors do not reach the rear arc this gate reads.
        sectors = tuning.lidar_sectors
        occluded = sectors.self_detection_threshold_m / 2.0
        base = create_scan_with_sectors(front=0.06)
        ranges = [
            occluded if abs(wrap_angle(a - math.pi)) <= math.radians(sectors.threat_half_fov_deg) else r
            for r, a in zip(base, ANGLES, strict=False)
        ]
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)
        assert not nav._pose_trail, "test precondition: no history yet"

        nav.step()

        assert gateway.commands, "expected a published command"
        assert gateway.commands[-1].speed_mps >= 0, "must not reverse blind into an unseen rear wall"
        assert gateway.commands[-1].speed_mps <= tuning.speed.slow_mps, (
            "degraded path is a capped creep, not full speed"
        )


class TestMappedObstacleEscapeSplit:
    """A sign the SignRouter is routing around must not trigger the reactive
    escape maneuver; anything the router does not own still must.

    The router passes a sign at ~lateral_offset centre-to-centre by design,
    which is inside CONTACT_DIST — so without this split the escape fires on
    every sign pass and reverses the robot out of the gap the planner aimed
    for. Measured over the 16 obstacles fixtures, that decided the run before
    the router's aim could matter: the lateral_offset knob was byte-identical
    from 0.20 to 0.32 with the split off, and 16/16 -> 14/16 with 2/16
    completing three laps once it was on (docs/sign-avoidance-investigation.md).
    """

    _FRONT_RANGE = 0.06
    """Inside CONTACT_DIST (0.10), so the raw scan reads CRITICAL."""

    _BASE_X, _BASE_Y = 1.5, 0.5
    """Robot anchor, squarely inside the SOUTH corridor's interior (well clear
    of the 1.0/2.0 inner-square boundary corridor_for_position partitions on)
    rather than the world origin. The escape mask now gates on the ROBOT's
    own corridor matching a routed sign's (see mask_mapped_obstacles), and
    the origin sits in the corner tie-break zone where a few centimetres can
    flip the classification -- exactly the coordinates ``sign_xy`` moves
    the sign by in these tests. Anchoring inside a real corridor keeps that
    flip out of scope for tests that aren't about corridor boundaries."""

    @staticmethod
    def _navigator(waypoints, sign_xy, tuning, mask_radius=None, override_tuning=None):
        """A navigator facing a close front return, with a sign mapped at ``sign_xy``.

        The robot sits at ``(_BASE_X, _BASE_Y)`` facing east, so the front
        return lands at ``(_BASE_X + _FRONT_RANGE, _BASE_Y)`` in world
        coordinates. ``sign_xy`` is relative to that same anchor, matching
        the front return's own offset when a caller wants them co-located.
        """
        ranges = create_scan_with_sectors(front=TestMappedObstacleEscapeSplit._FRONT_RANGE)
        base_x, base_y = TestMappedObstacleEscapeSplit._BASE_X, TestMappedObstacleEscapeSplit._BASE_Y
        gateway = FakeGateway(
            Pose(x=base_x, y=base_y, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        if mask_radius is not None:
            tuning = override_tuning(tuning, sign_router={"escape_mask_radius_m": mask_radius})
        router = SignRouter(
            [SignSpec(x=base_x + sign_xy[0], y=base_y + sign_xy[1], color=SignColor.RED)],
            config=SignRouterConfig.from_tuning(tuning.sign_router),
        )
        nav = CoreNavigator(
            gateway=gateway,
            waypoints=waypoints,
            num_laps=1,
            tuning=tuning,
            sign_router=router,
        )
        return gateway, nav

    def test_routed_sign_does_not_trigger_the_escape(self, waypoints, tuning):
        """The front return lands on the mapped sign, so no reverse is commanded."""
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0), tuning=tuning)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps > 0, "a sign the router owns must not trigger a reversing escape"

    def test_unmapped_obstacle_at_the_same_range_still_escapes(self, waypoints, tuning):
        """Same scan, sign mapped elsewhere: the full reactive guard applies.

        This is the half that makes the split a split rather than a blanket
        suppression — the robot must still reverse off a wall or an
        undiscovered obstacle at exactly this range.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(0.0, 0.9), tuning=tuning)
        seed_straight_pose_trail(nav)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0, "an obstacle nothing owns must still trigger the escape"

    def test_zero_mask_radius_restores_the_escape(self, waypoints, tuning, override_tuning):
        """The documented off-switch really is off."""
        gateway, nav = self._navigator(
            waypoints, sign_xy=(self._FRONT_RANGE, 0.0), tuning=tuning, mask_radius=0.0, override_tuning=override_tuning
        )
        seed_straight_pose_trail(nav)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0

    def test_masked_sign_still_slows_the_robot(self, waypoints, tuning):
        """Speed is governed by the RAW scan, so the robot still slows for a
        sign — it just no longer panics at one.

        Specifically the raw ``risk != SAFE`` cap still applies, which is what
        holds this to the slow tier rather than the fast one. Losing it would mean
        taking every sign pass at full speed, which is not what the split is
        for: the split removes the escape maneuver, not the caution.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0), tuning=tuning)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps <= nav._tuning.speed.slow_mps

    def test_passed_sign_gets_its_guard_back(self, waypoints, tuning):
        """Once the router retires a sign it stops owning it, so the reactive
        layer must resume treating that return as a real threat.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0), tuning=tuning)
        nav.sign_router._passed.add(0)
        seed_straight_pose_trail(nav)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0


class TestStuckDetectionDuringParking:
    """Stuck detection must keep running once parking has engaged (CA-5) —
    previously `_handle_finish` returned before the stuck-detector update ever
    ran, so a robot nosed against a parking block stalled forever.
    """

    def test_stuck_while_parking_triggers_reverse_escape(self, waypoints, tuning):
        gateway = FakeGateway(Pose(x=1.5, y=0.05, yaw=-math.pi / 2), lidar=None)
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        # Simulate "already finished, parking engaged, nosed against a block":
        # laps are done and parking is active but the robot hasn't moved.
        nav._laps_completed = 1
        nav._park_controller = _StubParkController()
        nav._parking_engaged = True

        for _ in range(tuning.escape.stuck_timeout_frames(tuning.control.control_hz) + 15):
            nav.step()

        assert any(cmd.speed_mps < 0 for cmd in gateway.commands), (
            "expected a reverse escape once the stuck timeout elapsed while parking"
        )

    def test_holding_after_parking_done_never_reverses(self, waypoints, tuning):
        """Once parked (done), stationary holding must never be read as stuck."""
        gateway = FakeGateway(Pose(x=1.5, y=0.05, yaw=-math.pi / 2), lidar=None)
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav._laps_completed = 1
        nav._park_controller = _StubParkController(done=True)
        nav._parking_engaged = True

        for _ in range(tuning.escape.stuck_timeout_frames(tuning.control.control_hz) + 10):
            nav.step()

        assert all(cmd.speed_mps == 0.0 for cmd in gateway.commands)


# A rear obstacle has to be OUTSIDE the chassis to exist at all: the rear face
# sits RobotSpecs.LIDAR_TO_REAR_BUMPER = 0.2722 m behind the sensor, so the
# small value these fixtures once used was inside the robot. Rear
# self-detection is chassis geometry now rather than a small scalar, so that
# value is filtered as the body -- correctly. 0.30 m is just behind the
# BUMPER, which is what `bumper_gap_behind` compares against CONTACT_DIST, so
# this is still "rear blocked" and the assertions below are unchanged. See
# adr:0056-raw-and-masked-scan.
REAR_BLOCKED_M = 0.30


class TestTheEscapePublishesItsDecisionInputs:
    """A bag recorded what the escape COMMANDED but never what it was ASKED for.

    Several analyses stalled on that gap: whether the K-turn's poor corner
    agreement is a wrong choice or a correct refusal of a shut side; how
    many ticks the pass-side refusal decides (it needed the controller
    monkey-patched in the simulator to answer at all); and which actor cancels
    the escape pendulum, where the chassis was measured obeying its own command
    on almost every reverse leg, so the defect is purely which SIDE each actor
    picks, and the side the router asked for was not on the wire. See
    adr:0050-escape-steering-degrees-and-committed-side.

    REACHABILITY IS NOT PROVEN HERE and deliberately so. The fields are written
    on the COLLISION escape path (K_TURN / SIDE_CORRECTION), and no unit test in
    this file reaches it: a front-blocked scan takes the `escape_recovery`
    STUCK_* path instead, and every existing K_TURN test constructs the
    manoeuvre by hand rather than driving the navigator into one. Reachability
    is proven end-to-end against the obstacles simulator instead; see the commit
    that added these fields. What IS pinned here is the control below, because
    a field that only ever reads None is indistinguishable from dead
    instrumentation.
    """

    def test_the_open_challenge_publishes_no_preferred_sign(self, waypoints, tuning):
        """The control. With no sign router there is no committed side to ask for.

        Without it a None reading could not be told from the field never being
        written, and every Open Challenge tick takes this path.
        """
        ranges = create_scan_with_sectors(front=0.06)
        gateway = FakeGateway(
            Pose(x=0.0, y=0.0, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        seen = []
        for _ in range(10):
            nav.step()
            seen.append(nav.debug_snapshot.escape_preferred_sign)

        assert all(v is None for v in seen)


class TestStuckEscapeRearBlocked:
    """A robot wedged with reverse blocked used to just hold and
    reset the stuck detector forever, re-arming the same forward command that
    had already failed -- confirmed on real hardware as frozen at one
    position for a long stretch (see docs/known-issues-backlog.md). When
    forward has room, it should get a real forward escape at full steering
    lock instead of an indefinite hold. See
    adr:0055-escape-maneuver-selection.
    """

    def test_forward_room_forces_a_forward_escape_instead_of_holding(self, waypoints, tuning):
        ranges = create_scan_with_sectors(
            back=REAR_BLOCKED_M
        )  # blocked: 0.03 m behind the bumper  # rear blocked, front stays LIDAR_DEFAULT_FAR
        gateway = FakeGateway(
            Pose(x=0.0, y=0.0, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        for _ in range(tuning.escape.stuck_timeout_frames(tuning.control.control_hz) + 5):
            nav.step()

        escape_cmds = [c for c in gateway.commands if c.speed_mps > 0 and abs(c.steering_norm) > 0.5]
        assert escape_cmds, "expected a forward escape at strong steering once stuck with rear blocked"
        assert nav.debug_snapshot.active_maneuver_type == ManeuverType.STUCK_FORWARD.value

    def test_both_ends_blocked_still_holds(self, waypoints, tuning):
        """Genuinely sandwiched (front AND rear blocked): holding is still correct."""
        ranges = create_scan_with_sectors(front=0.06, back=REAR_BLOCKED_M)
        gateway = FakeGateway(
            Pose(x=0.0, y=0.0, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        for _ in range(tuning.escape.stuck_timeout_frames(tuning.control.control_hz) + 5):
            nav.step()

        assert all(c.speed_mps >= 0 for c in gateway.commands), "must not reverse into an unseen rear wall"
        # Both ends blocked with no rear sensor: the safe recovery is a low-speed
        # forward pivot toward the open side, NOT a frozen hold (which deadlocked
        # into timeouts). It must still never reverse, and must actually steer.
        assert any(c.speed_mps > 0 and abs(c.steering_norm) > 0.0 for c in gateway.commands), (
            "should stop-and-steer (forward pivot) when both ends are blocked"
        )


class _StubParkController:
    """Minimal ParkController stand-in: always drives in place, never finishes
    on its own unless constructed with ``done=True``.
    """

    def __init__(self, done: bool = False) -> None:
        self.is_done = done
        self.is_repositioning = False
        self.section = None
        self.staging = Waypoint(0.0, 0.0)

    def update(self, robot_pose):
        from src.navigation.maneuvers.parking import ParkCommand

        if self.is_done:
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")
        return ParkCommand(linear=0.1, steering=0.0, done=False, phase="enter")


class TestMissingSensorsDegradeSafely:
    def test_missing_pose_stops(self, waypoints, tuning):
        gateway = FakeGateway(pose=None, lidar=None)  # type: ignore[arg-type]
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav.step()

        assert gateway.commands == [DriveCommand(speed_mps=0.0, steering_norm=0.0)]

    def test_missing_lidar_does_not_use_full_speed(self, waypoints, tuning):
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), lidar=None)
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav.step()

        assert gateway.commands
        # Resolved ladder, not the base one: this navigator has no sign_router,
        # so it is an Open Challenge navigator and drives the OPEN tiers. Against
        # the base ladder this reads as a violation whenever a motor profile
        # gives Open a faster slow tier (0.275 vs 0.22 on the REV HD Hex), which
        # is the profile working as intended rather than a degraded-sensor bug.
        assert gateway.commands[-1].speed_mps <= tuning.speed.for_open_challenge().slow_mps


class TestEscapeEscalation:
    """_maybe_escalate: after escalate_after_attempts consecutive failed escapes,
    the next escape should reverse longer and swing to the opposite side instead
    of repeating an identical pulse into the same wall
    (adr:0055-escape-maneuver-selection).
    """

    @staticmethod
    def _navigator(waypoints, tuning) -> CoreNavigator:
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), lidar=None)
        return CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

    @staticmethod
    def _maneuver(steering: float = 0.4, duration: int = 6) -> EscapeManeuver:
        return EscapeManeuver(
            maneuver_type=ManeuverType.K_TURN,
            steering=steering,
            speed=-0.2,
            duration_frames=duration,
        )

    def test_within_threshold_returns_maneuver_unchanged(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning)
        maneuver = self._maneuver()

        for count in range(1, nav._tuning.escape.escalate_after_attempts + 1):
            nav._escape_count = count
            assert nav._maybe_escalate(maneuver) is maneuver

    def test_beyond_threshold_flips_side_and_extends_duration(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning)
        maneuver = self._maneuver(steering=0.4, duration=6)
        nav._escape_count = nav._tuning.escape.escalate_after_attempts + 1
        starting_sign = nav._escape_steer_sign

        result = nav._maybe_escalate(maneuver)

        assert result.duration_frames == min(
            6 * 2, nav._tuning.escape.max_escape_frames(nav._tuning.control.control_hz)
        )
        assert math.copysign(1.0, result.steering) == -starting_sign
        assert abs(result.steering) == pytest.approx(abs(maneuver.steering))
        assert result.maneuver_type == maneuver.maneuver_type
        assert result.speed == maneuver.speed

    def test_successive_escalations_commit_to_a_side_before_switching(self, waypoints, tuning):
        """A side is held for several attempts, not flipped on every one.

        Flipping every attempt means consecutive escapes rotate the chassis in
        opposite directions and cancel out -- measured on real hardware as a run
        of escalating escapes over many seconds that rocked the yaw and
        translated the robot nowhere. Escaping a wedge needs several attempts
        pushing the same way to accumulate. See
        adr:0055-escape-maneuver-selection.
        """
        nav = self._navigator(waypoints, tuning)
        maneuver = self._maneuver(steering=0.4, duration=6)
        commit = nav._tuning.escape.escape_side_commit_attempts
        assert commit > 1, "a commit of 1 is the alternate-every-attempt behaviour this pins against"

        signs = []
        for count in range(
            nav._tuning.escape.escalate_after_attempts + 1,
            nav._tuning.escape.escalate_after_attempts + 1 + 2 * commit,
        ):
            nav._escape_count = count
            signs.append(math.copysign(1.0, nav._maybe_escalate(maneuver).steering))

        assert len(set(signs[:commit])) == 1, f"first {commit} attempts must share a side, got {signs}"
        assert len(set(signs[commit:])) == 1, f"next {commit} attempts must share a side, got {signs}"
        assert signs[0] == -signs[commit], f"the two blocks must be opposite sides, got {signs}"
        assert signs[0] == -nav._escape_steer_sign, (
            f"escalation must start on the side opposite the one that just failed, got {signs}"
        )

    def test_gate_only_escalation_switches_side_without_doubling(self, waypoints, tuning):
        """With ``escalate_doubles_duration`` off, escalation is a GATE, not a ladder.

        The side switch is the part a wedge never reached; doubling a locked
        reverse doubles the arc it sweeps blind through the rear occlusion band,
        and on the corpus the doubled K-turn is the one that shoves an unmapped
        pillar past the legal displacement in a single manoeuvre. Obstacles
        ships it off. See adr:0055-escape-maneuver-selection.
        """
        nav = self._navigator(waypoints, tuning)
        nav._escape = nav._escape.model_copy(update={"escalate_doubles_duration": False})
        maneuver = self._maneuver(steering=0.4, duration=6)
        nav._escape_count = nav._tuning.escape.escalate_after_attempts + 1

        result = nav._maybe_escalate(maneuver)

        assert result.duration_frames == maneuver.duration_frames
        assert math.copysign(1.0, result.steering) == -nav._escape_steer_sign
        assert abs(result.steering) == pytest.approx(abs(maneuver.steering))

    def test_duration_caps_at_max_escape_frames(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning)
        maneuver = self._maneuver(
            steering=0.4, duration=nav._tuning.escape.max_escape_frames(nav._tuning.control.control_hz)
        )
        nav._escape_count = nav._tuning.escape.escalate_after_attempts + 1

        result = nav._maybe_escalate(maneuver)

        assert result.duration_frames == nav._tuning.escape.max_escape_frames(nav._tuning.control.control_hz)

    def test_straight_reverse_has_no_side_to_flip(self, waypoints, tuning):
        """A zero-steering escape (e.g. a straight stuck-reverse) stays at zero
        when escalated — only its duration should extend.
        """
        nav = self._navigator(waypoints, tuning)
        maneuver = self._maneuver(steering=0.0, duration=6)
        nav._escape_count = nav._tuning.escape.escalate_after_attempts + 1

        result = nav._maybe_escalate(maneuver)

        assert result.steering == 0.0
        assert result.duration_frames == 12


class TestEscapeEscalationIntegration:
    """End-to-end through CoreNavigator.step(): a threat that never clears must
    drive a sequence of full escape maneuvers whose duration/side escalates,
    not the same short pulse repeated forever.
    """

    def test_persistent_front_threat_escalates_after_repeated_escapes(self, waypoints, tuning):
        ranges = create_scan_with_sectors(front=0.06)
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)
        seed_straight_pose_trail(nav)

        maneuvers_begun: list[EscapeManeuver] = []
        was_active = False
        target = tuning.escape.escalate_after_attempts + 2
        for _ in range(500):
            nav.step()
            now_active = nav._active_maneuver is not None
            if now_active and not was_active:
                maneuvers_begun.append(nav._active_maneuver)
                if len(maneuvers_begun) >= target:
                    break
            was_active = now_active

        assert len(maneuvers_begun) == target, "threat never cleared, so every maneuver should re-trigger a new escape"

        pre_escalation = maneuvers_begun[: tuning.escape.escalate_after_attempts]
        escalated = maneuvers_begun[tuning.escape.escalate_after_attempts]
        last_pre_escalation = pre_escalation[-1]

        assert escalated.duration_frames > last_pre_escalation.duration_frames
        if last_pre_escalation.steering:
            assert math.copysign(1.0, escalated.steering) == -math.copysign(1.0, last_pre_escalation.steering)


class TestEscapeEscalationSurvivesInterveningNormalDriveTicks:
    """A brief normal_drive tick between escape attempts must not reset the
    escalation counter unless the robot actually moved.

    Confirmed on real hardware: with the
    robot genuinely pinned in place, SIDE_CORRECTION's brief creep read as
    "not critical" for one tick between escapes, which reset escape_count to
    0 every single cycle -- so it never reached escalate_after_attempts and
    never escalated, for many seconds. The threat toggling on/off each
    decision tick (rather than staying permanently critical, as in
    TestEscapeEscalationIntegration above) is what reproduces the gap that
    test doesn't cover: a *fixed* threat never even reaches the normal_drive
    reset branch, since a new escape re-triggers before the old one clears. See
    adr:0055-escape-maneuver-selection.
    """

    def test_oscillating_threat_without_progress_still_escalates(self, waypoints, tuning):
        close = tuple(create_scan_with_sectors(front=0.06))
        far = tuple(np.full(NUM_RAYS, LIDAR_DEFAULT_FAR).tolist())

        class _OscillatingGateway(FakeGateway):
            def __init__(self, pose):
                super().__init__(pose, LidarScan(ranges_m=close, angles_rad=tuple(ANGLES)))
                self._tick = 0

            def get_lidar_scan(self):
                self._tick += 1
                ranges = close if self._tick % 2 else far
                return LidarScan(ranges_m=ranges, angles_rad=tuple(ANGLES))

        gateway = _OscillatingGateway(Pose(x=0.0, y=0.0, yaw=0.0))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        maneuvers_begun: list[EscapeManeuver] = []
        was_active = False
        target = tuning.escape.escalate_after_attempts + 2
        for _ in range(1000):
            nav.step()
            now_active = nav._active_maneuver is not None
            if now_active and not was_active:
                maneuvers_begun.append(nav._active_maneuver)
                if len(maneuvers_begun) >= target:
                    break
            was_active = now_active

        assert len(maneuvers_begun) == target, "expected escapes to keep re-triggering with the threat oscillating"

        pre_escalation = maneuvers_begun[: tuning.escape.escalate_after_attempts]
        escalated = maneuvers_begun[tuning.escape.escalate_after_attempts]
        assert escalated.duration_frames > pre_escalation[-1].duration_frames, (
            "escape_count must survive the intervening normal_drive tick since the robot never moved"
        )


class TestReverseFitsTheRearGap:
    """The K-turn's reverse DISTANCE must not exceed the rear room measured.

    ``k_turn_min_s``/``k_turn_max_s`` are chosen from the severity of what is in
    FRONT: at ``rev_speed`` the critical escape commits a reverse
    without reading a single number about what is BEHIND. ``_reversing_into_
    unseen_wall`` cannot catch it -- it only checks the gap at the FIRST frame
    against ``CONTACT_DIST``, so a gap larger than the first frame's authorises
    the whole reverse and
    the chassis is driven into the pillar it is escaping. Measured over the
    escape episodes on the bags, the reverse did not fit in a third of them.
    See adr:0055-escape-maneuver-selection.
    """

    _ROOM_M = 0.05
    """Rear room left beyond CONTACT_DIST: five frames of reverse at the
    shipped 0.20 m/s and 20 Hz, against a 22-frame critical K-turn."""

    @staticmethod
    def _scan_with_rear_at(tuning, rear_range_m: float) -> LidarScan:
        """A front-blocked scan whose whole rear arc reads ``rear_range_m``.

        Built by bearing rather than via ``create_scan_with_sectors(back=...)``,
        whose named sectors do not reach the rear arc ``rear_sector`` reads --
        the same reason the rear-blind test above builds its own.
        """
        sectors = tuning.lidar_sectors
        base = create_scan_with_sectors(front=0.06)
        ranges = [
            rear_range_m if abs(wrap_angle(a - math.pi)) <= math.radians(sectors.threat_half_fov_deg) else r
            for r, a in zip(base, ANGLES, strict=False)
        ]
        return LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES))

    @staticmethod
    def _navigator(waypoints, tuning, scan, *, obstacles: bool):
        """A navigator on the Obstacles or the Open path, facing ``scan``.

        The discriminator is ``sign_router`` presence, exactly as
        ``CoreNavigator`` resolves the per-challenge escape parameters. The
        router is given a sign far away so it cannot mask anything here.
        """
        gateway = FakeGateway(Pose(x=1.5, y=0.5, yaw=0.0), scan)
        router = (
            SignRouter(
                [SignSpec(x=50.0, y=50.0, color=SignColor.RED)],
                config=SignRouterConfig.from_tuning(tuning.sign_router),
            )
            if obstacles
            else None
        )
        return CoreNavigator(
            gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning, sign_router=router
        )

    def _rear_range_for_room(self, tuning, room_m: float, *, obstacles: bool = True) -> float:
        """Sensor range whose rear BUMPER gap leaves exactly ``room_m`` to spare.

        Against the CONTACT_DIST the navigator itself resolves for this
        challenge, not the shared field: Obstacles ships OBSTACLES_CONTACT_DIST
        (0.04 against 0.10), and sizing the scan off the wrong one moves the
        room these tests are pinning.
        """
        zones = tuning.clearance.for_obstacles_challenge() if obstacles else tuning.clearance
        return RobotSpecs.LIDAR_TO_REAR_BUMPER + zones.contact_dist + room_m

    def _critical_k_turn(self, tuning) -> EscapeManeuver:
        return EscapeManeuver(
            maneuver_type=ManeuverType.K_TURN,
            steering=tuning.escape.rev_steer_norm(),
            speed=tuning.escape.rev_speed,
            duration_frames=tuning.escape.k_turn_max_frames(tuning.control.control_hz),
        )

    def test_obstacles_shortens_a_reverse_that_does_not_fit(self, waypoints, tuning):
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, self._ROOM_M))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._critical_k_turn(tuning)

        fitted = nav._fit_reverse_to_rear_gap(maneuver, scan)

        per_frame = abs(maneuver.speed) / tuning.control.control_hz
        assert fitted.duration_frames < maneuver.duration_frames, "21.6 cm of reverse into 5 cm of room"
        assert fitted.duration_frames * per_frame <= self._ROOM_M
        # A ceiling, not a rewrite: nothing else about the manoeuvre moves.
        assert fitted.steering == maneuver.steering
        assert fitted.speed == maneuver.speed

    def test_open_is_untouched(self, waypoints, tuning):
        """Open ships the shared value (False), so the same geometry is unchanged.

        Its escapes fire in corners against WALLS, where a shortened reverse
        under-rotates and re-triggers; nothing measured says Open wants this.
        """
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, self._ROOM_M, obstacles=False))
        nav = self._navigator(waypoints, tuning, scan, obstacles=False)
        maneuver = self._critical_k_turn(tuning)

        assert nav._fit_reverse_to_rear_gap(maneuver, scan) == maneuver

    def test_a_reverse_that_already_fits_is_untouched(self, waypoints, tuning):
        """Most of them, and shortening those would be the regression. See
        adr:0055-escape-maneuver-selection."""
        maneuver = self._critical_k_turn(tuning)
        needed = abs(maneuver.speed) * maneuver.duration_frames / tuning.control.control_hz
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, needed + 0.10))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)

        assert nav._fit_reverse_to_rear_gap(maneuver, scan) == maneuver

    def test_an_unmeasured_rear_is_left_alone_not_capped_to_zero(self, waypoints, tuning):
        """Capping on the no-data sentinel would delete the manoeuvre outright.

        Authorising a blind reverse stays ``_reversing_into_unseen_wall``'s job,
        which refuses it unless the pose trail vouches for the ground; this must
        not pre-empt that with a one-frame stub.
        """
        occluded = tuning.lidar_sectors.self_detection_threshold_m / 2.0
        scan = self._scan_with_rear_at(tuning, occluded)
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._critical_k_turn(tuning)

        assert not nav._collision_controller.rear_sector(scan.ranges_m, scan.angles_rad).measured
        assert nav._fit_reverse_to_rear_gap(maneuver, scan) == maneuver

    def test_a_rear_already_inside_contact_is_a_refusal_not_a_truncation(self, waypoints, tuning):
        """Below CONTACT_DIST the reverse must be REFUSED, which is the gate's job.

        Returning a one-frame stub here would convert a clean refusal into a
        twitch, and the refusal is what stops the chassis moving at all.
        """
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, -0.02))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._critical_k_turn(tuning)

        assert nav._fit_reverse_to_rear_gap(maneuver, scan) == maneuver
        assert nav._reversing_into_unseen_wall(maneuver, scan), "the refusal must come from the gate"

    def test_a_forward_maneuver_is_never_shortened(self, waypoints, tuning):
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, self._ROOM_M))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        forward = replace(self._critical_k_turn(tuning), speed=abs(tuning.escape.rev_speed))

        assert nav._fit_reverse_to_rear_gap(forward, scan) == forward

    def test_a_retrace_is_exempt(self, waypoints, tuning):
        """A retrace backs along ground the chassis physically occupied.

        Its room is vouched for by the pose trail rather than by the rear
        sector -- the same exemption ``_reversing_into_unseen_wall`` makes.
        """
        scan = self._scan_with_rear_at(tuning, self._rear_range_for_room(tuning, self._ROOM_M))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        nav._retracing = True
        maneuver = self._critical_k_turn(tuning)

        assert nav._fit_reverse_to_rear_gap(maneuver, scan) == maneuver


class TestEscapeMirrorsReverse:
    """``escape_mirrors_reverse``: the reverse leg curves the OTHER way.

    Every escape manoeuvre is built from the same
    ``rev_steer_norm() * _escape_steer_sign_for_attempt()``, so a reverse holds
    the lock the forward leg used and undoes its rotation. Measured on the
    hardware runs, the large majority of forward/reverse leg pairs held
    the same sign. This is the bay's pendulum outside the bay. See
    adr:0050-escape-steering-degrees-and-committed-side.
    """

    @staticmethod
    def _nav_with(_tuning, waypoints, mirrors: bool):
        overridden = tuning_with_overrides({"escape_mirrors_reverse": mirrors}, group="escape")
        ranges = create_scan_with_sectors(front=0.06)
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES))
        gateway = FakeGateway(Pose(x=1.5, y=0.5, yaw=0.0), scan)
        return CoreNavigator(
            gateway=gateway, waypoints=waypoints, num_laps=1, tuning=overridden, sign_router=None
        )

    def test_ships_off(self, tuning) -> None:
        assert tuning.escape.escape_mirrors_reverse is False

    def test_the_reverse_steers_opposite_to_the_unmirrored_one(self, waypoints, tuning) -> None:
        """Same attempt, same side commitment; only the reverse leg's sign moves."""
        plain = self._nav_with(tuning, waypoints, mirrors=False)
        mirrored = self._nav_with(tuning, waypoints, mirrors=True)

        reverses = []
        for nav in (plain, mirrored):
            nav._handle_stuck_escape(1.5, 0.5, 0.0)
            maneuver = nav._active_maneuver
            assert maneuver is not None, "the stuck escape must have begun a manoeuvre"
            reverses.append(maneuver)

        if reverses[0].maneuver_type is not ManeuverType.STUCK_REVERSE:
            pytest.skip("this geometry did not select a reverse; the sign rule is only defined for one")
        assert reverses[1].maneuver_type is ManeuverType.STUCK_REVERSE
        assert reverses[0].steering == pytest.approx(-reverses[1].steering)
        assert reverses[0].steering != 0.0, "a zero lock would make the comparison vacuous"


class TestDwellGate:
    """The escape's side switch gated on TIME IN PLACE rather than attempt count.

    The attempt counter cannot express this. It resets on 3 cm of travel, and
    the failure it is meant to catch produces that 3 cm continuously: measured
    over the 2026-09-15 session, a wedged round covered 22.3 m of path for
    0.14 m of net displacement, so ``escalate_after_attempts`` fired on 0% of
    its latches while firing on 24% of a clean round's. Dwell measures what
    separates them. See ``adr:0055-escape-maneuver-selection``.
    """

    RADIUS_M = 0.30
    DWELL_S = 12.0

    @staticmethod
    def _navigator(waypoints, tuning, *, dwell_s: float) -> CoreNavigator:
        overridden = tuning_with_overrides(
            {
                "escape_dwell_seconds": dwell_s,
                "escape_dwell_radius_m": TestDwellGate.RADIUS_M,
                "escape_dwell_cooldown_s": 10.0,
                "escape_dwell_max_per_place": 3,
            },
            group="escape",
            base=tuning,
        )
        gateway = FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), lidar=None)
        return CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=overridden)

    @staticmethod
    def _maneuver(steering: float = 0.4) -> EscapeManeuver:
        return EscapeManeuver(
            maneuver_type=ManeuverType.K_TURN, steering=steering, speed=-0.2, duration_frames=6
        )

    @staticmethod
    def _thrash(nav: CoreNavigator, seconds: float, *, amplitude_m: float) -> None:
        """Feed poses that move constantly and go nowhere, the measured wedge shape."""
        for tick in range(int(seconds * nav._tuning.control.control_hz)):
            nav.note_dwell_sample(amplitude_m * (1.0 if tick % 2 else -1.0), 0.0)

    def test_thrashing_in_place_switches_the_side(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning, dwell_s=self.DWELL_S)
        # Twice the 3 cm the counter watches, so the counter would have reset
        # on every single one of these ticks.
        self._thrash(nav, self.DWELL_S + 1.0, amplitude_m=0.06)

        flipped = nav._maybe_dwell_flip(self._maneuver(steering=0.4), 0.06, 0.0)

        assert flipped.steering == pytest.approx(-0.4)
        assert flipped.duration_frames == 6, "the gate switches the side and nothing else"

    def test_shipped_default_is_inert(self, waypoints, tuning):
        """The control. Without it, the test above proves only that a flip happens."""
        nav = self._navigator(waypoints, tuning, dwell_s=0.0)
        self._thrash(nav, self.DWELL_S + 1.0, amplitude_m=0.06)
        maneuver = self._maneuver(steering=0.4)

        assert nav._maybe_dwell_flip(maneuver, 0.06, 0.0) is maneuver

    def test_driving_away_does_not_dwell(self, waypoints, tuning):
        """The other control: the same elapsed time, spent actually going somewhere."""
        nav = self._navigator(waypoints, tuning, dwell_s=self.DWELL_S)
        for tick in range(int((self.DWELL_S + 1.0) * nav._tuning.control.control_hz)):
            nav.note_dwell_sample(0.01 * tick, 0.0)
        maneuver = self._maneuver(steering=0.4)

        assert nav._maybe_dwell_flip(maneuver, 0.01 * tick, 0.0) is maneuver

    def test_capped_per_place(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning, dwell_s=self.DWELL_S)
        cap = nav._tuning.escape.escape_dwell_max_per_place

        fires = 0
        for _ in range(cap + 3):
            # Each round of thrashing also advances past the cooldown, so the
            # cap is the only thing that can stop the gate here.
            self._thrash(nav, self.DWELL_S + 1.0, amplitude_m=0.06)
            if nav._maybe_dwell_flip(self._maneuver(), 0.06, 0.0).steering < 0:
                fires += 1
        assert fires == cap, "a fourth switch in the same place is not the missing ingredient"


class TestLockedKTurnDeclinesIntoTheTail:
    """A locked reverse must not swing the TAIL into something beside it.

    In reverse the nose swings toward the steer side and the tail the other
    way. The wanted-side gate in ``_k_turn_steer_sign`` checks the nose side;
    nothing checked the rear quadrant the tail sweeps, which is where the
    pillar the chassis was passing sits when the wall ahead fires the escape.
    Measured on the corpus: five of six pillar pushes accrued in that swing.
    ``k_turn_tail_clearance_m`` declines the lock and reverses straight, the
    K-turn's existing answer to a shut wanted side. Obstacles only.
    """

    _POSE = Pose(x=1.5, y=0.5, yaw=0.0)

    @staticmethod
    def _front_blocked() -> LidarScan:
        return LidarScan(ranges_m=tuple(create_scan_with_sectors(front=0.06)), angles_rad=tuple(ANGLES))

    @staticmethod
    def _navigator(waypoints, tuning, scan, *, obstacles: bool) -> CoreNavigator:
        gateway = FakeGateway(Pose(x=1.5, y=0.5, yaw=0.0), scan)
        router = (
            SignRouter(
                [SignSpec(x=50.0, y=50.0, color=SignColor.RED)],
                config=SignRouterConfig.from_tuning(tuning.sign_router),
            )
            if obstacles
            else None
        )
        return CoreNavigator(
            gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning, sign_router=router
        )

    @staticmethod
    def _locked_k_turn(tuning, sign: float) -> EscapeManeuver:
        return EscapeManeuver(
            maneuver_type=ManeuverType.K_TURN,
            steering=sign * tuning.escape.rev_steer_norm(),
            speed=tuning.escape.rev_speed,
            duration_frames=tuning.escape.k_turn_max_frames(tuning.control.control_hz),
        )

    def _pillar(self, along_m: float, left_m: float) -> list[tuple[Waypoint, Section]]:
        """A mapped sign at chassis-frame (``along_m``, ``left_m``) for a yaw-0 pose."""
        return [(Waypoint(self._POSE.x + along_m, self._POSE.y + left_m), Section.NORTH)]

    def test_obstacles_declines_the_lock_when_a_mapped_pillar_sits_beside_the_tail(self, waypoints, tuning):
        scan = self._front_blocked()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)  # nose swings right, tail swings LEFT

        declined = nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(-0.07, 0.19))

        assert declined.steering == 0.0, "rear-left pillar beside the flank is inside the strip the tail sweeps"
        assert declined.speed == maneuver.speed
        assert declined.duration_frames == maneuver.duration_frames

    def test_a_wall_ahead_of_the_rear_bumper_line_is_not_the_tail_strip(self, waypoints, tuning):
        scan = self._front_blocked()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)

        # Same lateral offset, but level with the FRONT axle: the nose swings
        # away from it and the tail never reaches it.
        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(+0.10, 0.19)) == maneuver

    def test_a_pillar_on_the_nose_side_does_not_decline(self, waypoints, tuning):
        scan = self._front_blocked()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)

        # Same pillar mirrored to the rear-RIGHT: the tail swings away from it.
        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(-0.07, -0.19)) == maneuver

    def test_a_pillar_beyond_the_limit_does_not_decline(self, waypoints, tuning):
        scan = self._front_blocked()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)
        reach = tuning.escape.for_obstacles_challenge().k_turn_tail_clearance_m
        beyond = RobotSpecs.WIDTH / 2 + reach + TrafficSignSpecs.WIDTH / 2 + 0.05

        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(-0.07, beyond)) == maneuver

    def test_a_raw_return_beside_the_tail_declines_without_a_map(self, waypoints, tuning):
        # 0.25 m at +140 deg from the sensor lands at (-0.07, 0.16) from the
        # chassis centre: beside the rear-left flank, outside the body, inside
        # the strip the tail sweeps.
        base = create_scan_with_sectors(front=0.06)
        ranges = [
            0.25 if abs(a - math.radians(140.0)) < math.radians(3.0) else r
            for r, a in zip(base, ANGLES, strict=False)
        ]
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)

        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, []).steering == 0.0

    def test_the_lock_stands_when_the_straight_reverse_has_no_room(self, waypoints, tuning):
        # Rear arc reading a gap under one minimum K-turn: the straight leg
        # would be cut to a few frames by the rear-gap fit and deliver nothing,
        # so the lock is the lesser harm and is kept.
        sectors = tuning.lidar_sectors
        zones = tuning.clearance.for_obstacles_challenge()
        rear_range = RobotSpecs.LIDAR_TO_REAR_BUMPER + zones.contact_dist + 0.02
        base = create_scan_with_sectors(front=0.06)
        ranges = [
            rear_range if abs(wrap_angle(a - math.pi)) <= math.radians(sectors.threat_half_fov_deg) else r
            for r, a in zip(base, ANGLES, strict=False)
        ]
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES))
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        maneuver = self._locked_k_turn(tuning, +1.0)

        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(-0.07, 0.19)) == maneuver

    def test_open_challenge_is_untouched(self, waypoints, tuning):
        scan = self._front_blocked()
        nav = self._navigator(waypoints, tuning, scan, obstacles=False)
        maneuver = self._locked_k_turn(tuning, +1.0)

        assert nav._decline_lock_into_tail(maneuver, scan, 1.5, 0.5, 0.0, self._pillar(-0.07, 0.19)) == maneuver


class TestSetupReverseBuysRoomBeforeReapproach:
    """After a FRONT-threat K-turn, back straight until the gap fits a turn.

    Measured over 105 hardware escape episodes: the escape gains a median 9.8
    cm and the planner then hands back the same target 97% of the time, so 62%
    re-fire within two seconds. ``setup_reverse_room_m`` chains up to
    ``setup_reverse_legs`` straight legs after the K-turn until the forward
    bumper gap reaches it. REFUTED on the corpus (13 to 15: the room is bought
    and the planner re-approaches into the pillar anyway) and shipped OFF;
    these tests pin the mechanism for the knob, enabled explicitly. See
    adr:0055-escape-maneuver-selection.
    """

    _ROOM_M = 0.35

    @staticmethod
    def _navigator(waypoints, tuning, scan, *, obstacles: bool) -> CoreNavigator:
        gateway = FakeGateway(Pose(x=1.5, y=0.5, yaw=0.0), scan)
        router = (
            SignRouter(
                [SignSpec(x=50.0, y=50.0, color=SignColor.RED)],
                config=SignRouterConfig.from_tuning(tuning.sign_router),
            )
            if obstacles
            else None
        )
        return CoreNavigator(
            gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning, sign_router=router
        )

    @staticmethod
    def _open_rear_scan() -> LidarScan:
        return LidarScan(ranges_m=tuple(create_scan_with_sectors(front=0.20, back=3.0)), angles_rad=tuple(ANGLES))

    def test_chains_straight_legs_until_the_room_is_there(self, waypoints, tuning):
        scan = self._open_rear_scan()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        nav._escape = nav._escape.model_copy(update={"setup_reverse_room_m": self._ROOM_M})
        room = self._ROOM_M
        nav._setup_legs_left = nav._escape.setup_reverse_legs

        first = nav._setup_reverse_leg(scan, forward_clearance=room - 0.05)

        assert first is not None
        assert first.steering == 0.0, "a setup leg is straight: it exists to buy room, not to swing"
        assert first.speed == nav._escape.rev_speed
        assert first.duration_frames == nav._escape.k_turn_min_frames(tuning.control.control_hz)
        assert nav._setup_legs_left == nav._escape.setup_reverse_legs - 1

        # Room reached: no more legs, and the arming is spent.
        assert nav._setup_reverse_leg(scan, forward_clearance=room + 0.01) is None
        assert nav._setup_legs_left == 0

    def test_the_leg_budget_caps_the_chain(self, waypoints, tuning):
        scan = self._open_rear_scan()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        nav._escape = nav._escape.model_copy(update={"setup_reverse_room_m": self._ROOM_M})
        room = self._ROOM_M
        nav._setup_legs_left = nav._escape.setup_reverse_legs

        legs = 0
        while nav._setup_reverse_leg(scan, forward_clearance=room - 0.05) is not None:
            legs += 1
        assert legs == nav._escape.setup_reverse_legs

    def test_not_armed_means_no_leg(self, waypoints, tuning):
        scan = self._open_rear_scan()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        assert nav._setup_legs_left == 0
        assert nav._setup_reverse_leg(scan, forward_clearance=0.05) is None

    def test_shipped_off_means_no_leg_even_when_armed(self, waypoints, tuning):
        scan = self._open_rear_scan()
        nav = self._navigator(waypoints, tuning, scan, obstacles=True)
        assert nav._escape.setup_reverse_room_m == 0.0, "refuted on the corpus, ships off"
        nav._setup_legs_left = 3
        assert nav._setup_reverse_leg(scan, forward_clearance=0.05) is None

    def test_open_challenge_never_backs_up_for_room(self, waypoints, tuning):
        scan = self._open_rear_scan()
        nav = self._navigator(waypoints, tuning, scan, obstacles=False)
        nav._setup_legs_left = 3
        assert nav._setup_reverse_leg(scan, forward_clearance=0.05) is None


class TestPostEscapeCreep:
    """A front-threat reverse escape arms a creep-speed cap for the re-approach.

    Measured on the corpus at re-approach: the lane target sits 0.17-0.25 m
    ahead with 0.06-0.18 m of lateral offset. At the 0.35 m capped turn radius
    the chassis can shift 0.06 m in that depth; at creep (R = 0.24 m) 0.11 m.
    The arithmetic held and the corpus refuted it (13 to 21), so it ships OFF;
    these tests pin the mechanism for the knob, enabled explicitly. See
    adr:0055-escape-maneuver-selection.
    """

    @staticmethod
    def _navigator(waypoints, tuning, *, obstacles: bool) -> CoreNavigator:
        scan = LidarScan(ranges_m=tuple(create_scan_with_sectors(front=3.0, back=3.0)), angles_rad=tuple(ANGLES))
        gateway = FakeGateway(Pose(x=1.5, y=0.5, yaw=0.0), scan)
        router = (
            SignRouter(
                [SignSpec(x=50.0, y=50.0, color=SignColor.RED)],
                config=SignRouterConfig.from_tuning(tuning.sign_router),
            )
            if obstacles
            else None
        )
        return CoreNavigator(
            gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning, sign_router=router
        )

    def _run_out(self, nav: CoreNavigator, maneuver: EscapeManeuver) -> None:
        nav._begin_maneuver(maneuver)
        for _ in range(maneuver.duration_frames):
            nav._drive_active_maneuver(1.5, 0.5, 0.0, phase=NavigatorPhase.ACTIVE_MANEUVER)
        assert nav._active_maneuver is None

    def test_a_reverse_k_turn_arms_the_creep_on_obstacles(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning, obstacles=True)
        assert nav._escape.post_escape_creep_s == 0.0, "refuted on the corpus (13 -> 21), ships off"
        seconds = 3.0
        nav._escape = nav._escape.model_copy(update={"post_escape_creep_s": seconds})

        self._run_out(nav, EscapeManeuver(ManeuverType.K_TURN, steering=0.4, speed=-0.2, duration_frames=4))

        assert nav._post_escape_creep_ticks == nav._escape.frames(seconds, tuning.control.control_hz)

    def test_a_forward_side_correction_does_not_arm_it(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning, obstacles=True)

        self._run_out(nav, EscapeManeuver(ManeuverType.SIDE_CORRECTION, steering=0.2, speed=0.1, duration_frames=4))

        assert nav._post_escape_creep_ticks == 0

    def test_open_challenge_stays_off(self, waypoints, tuning):
        nav = self._navigator(waypoints, tuning, obstacles=False)

        self._run_out(nav, EscapeManeuver(ManeuverType.K_TURN, steering=0.4, speed=-0.2, duration_frames=4))

        assert nav._post_escape_creep_ticks == 0
