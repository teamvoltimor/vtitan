"""Unit tests for CoreNavigator's escape-maneuver arbitration.

Uses a minimal fake HardwareGateway (implements the HardwareGateway Protocol
structurally) to drive CoreNavigator.step() directly, without ROS2 or the
simulator, so these scenarios can be pinned exactly: a wall dead ahead AND
dead behind must never produce a reversing command.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from shared.config.constants import ColorNames
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.control.controllers import EscapeManeuver, ManeuverType
from src.navigation.core_navigator import CoreNavigator
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, SignSpec
from src.navigation.ports import DriveCommand, LidarScan
from tests.test_constants import (
    ANGLES_FULL_ROTATION,
    LIDAR_DEFAULT_FAR,
    NUM_RAYS,
    SIDE_SECTOR_INDICES,
)

ANGLES = ANGLES_FULL_ROTATION.tolist()


def _scan_with_sectors(**close_sectors: float) -> list[float]:
    """A far-range scan with the given named sectors (front/back/left/right) closed."""
    ranges = np.full(NUM_RAYS, LIDAR_DEFAULT_FAR)
    centers = {"front": 0.0, "left": math.pi / 2, "right": -math.pi / 2, "back": math.pi}
    for name, dist in close_sectors.items():
        center = centers[name]
        idx = int(np.argmin(np.abs(np.asarray(ANGLES) - center)))
        ranges[idx - SIDE_SECTOR_INDICES : idx + SIDE_SECTOR_INDICES] = dist
    return ranges.tolist()


class _FakeGateway:
    def __init__(self, pose: Pose, lidar: LidarScan | None) -> None:
        self._pose = pose
        self._lidar = lidar
        self.commands: list[DriveCommand] = []

    def publish_drive(self, command: DriveCommand) -> None:
        self.commands.append(command)

    def get_current_pose(self) -> Pose | None:
        return self._pose

    def get_lidar_scan(self) -> LidarScan | None:
        return self._lidar

    def get_imu_reading(self) -> IMUReading | None:
        return IMUReading(yaw=self._pose.yaw, pitch=0.0, roll=0.0)

    def get_vision_detections(self) -> list[Detection]:
        return []


@pytest.fixture()
def waypoints() -> list[tuple[float, float]]:
    return [(5.0, 0.0), (10.0, 0.0)]  # far ahead — never reached in these tests


class TestCriticalEscapeRearGate:
    def test_wedged_both_ends_never_reverses(self, waypoints):
        """Front AND rear blocked: the escape must not command a reverse.

        Rear distance (0.09 m) is deliberately beyond the self-detection radius
        (0.08 m, see CA-4) so this is a genuine wall, not a filtered chassis
        reflection — otherwise the rear-gate would never see it as blocked.
        """
        ranges = _scan_with_sectors(front=0.06, back=0.09)
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=NavigationTuning())

        nav.step()

        assert gateway.commands, "expected a published command"
        assert gateway.commands[-1].speed_mps >= 0, "must not reverse into an unseen rear wall"

    def test_front_blocked_rear_clear_reverses(self, waypoints):
        """Front blocked, rear clear: the K-turn escape should reverse."""
        ranges = _scan_with_sectors(front=0.06)
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=NavigationTuning())

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0, "front-only threat should trigger the reverse K-turn"


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

    @staticmethod
    def _navigator(waypoints, sign_xy, mask_radius=None):
        """A navigator facing a close front return, with a sign mapped at ``sign_xy``.

        The robot sits at the origin facing east, so the front return lands at
        ``(_FRONT_RANGE, 0)`` in world coordinates.
        """
        ranges = _scan_with_sectors(front=TestMappedObstacleEscapeSplit._FRONT_RANGE)
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        tuning = NavigationTuning()
        if mask_radius is not None:
            tuning = dataclasses.replace(
                tuning,
                sign_router=tuning.sign_router.model_copy(update={"ESCAPE_MASK_RADIUS_M": mask_radius}),
            )
        router = SignRouter(
            [SignSpec(x=sign_xy[0], y=sign_xy[1], color=ColorNames.RED)],
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

    def test_routed_sign_does_not_trigger_the_escape(self, waypoints):
        """The front return lands on the mapped sign, so no reverse is commanded."""
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0))

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps > 0, "a sign the router owns must not trigger a reversing escape"

    def test_unmapped_obstacle_at_the_same_range_still_escapes(self, waypoints):
        """Same scan, sign mapped elsewhere: the full reactive guard applies.

        This is the half that makes the split a split rather than a blanket
        suppression — the robot must still reverse off a wall or an
        undiscovered obstacle at exactly this range.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(0.0, 0.9))

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0, "an obstacle nothing owns must still trigger the escape"

    def test_zero_mask_radius_restores_the_escape(self, waypoints):
        """The documented off-switch really is off."""
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0), mask_radius=0.0)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0

    def test_masked_sign_still_slows_the_robot(self, waypoints):
        """Speed is governed by the RAW scan, so the robot still slows for a
        sign — it just no longer panics at one.

        Specifically the raw ``risk != SAFE`` cap still applies, which is what
        holds this to SLOW_SPEED rather than FAST_SPEED. Losing it would mean
        taking every sign pass at full speed, which is not what the split is
        for: the split removes the escape maneuver, not the caution.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0))

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps <= nav._tuning.speed.SLOW_SPEED

    def test_passed_sign_gets_its_guard_back(self, waypoints):
        """Once the router retires a sign it stops owning it, so the reactive
        layer must resume treating that return as a real threat.
        """
        gateway, nav = self._navigator(waypoints, sign_xy=(self._FRONT_RANGE, 0.0))
        nav.sign_router._passed.add(0)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps < 0


class TestStuckDetectionDuringParking:
    """Stuck detection must keep running once parking has engaged (CA-5) —
    previously `_handle_finish` returned before the stuck-detector update ever
    ran, so a robot nosed against a parking block stalled forever.
    """

    def test_stuck_while_parking_triggers_reverse_escape(self, waypoints):
        gateway = _FakeGateway(Pose(x=1.5, y=0.05, yaw=-math.pi / 2), lidar=None)
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        # Simulate "already finished, parking engaged, nosed against a block":
        # laps are done and parking is active but the robot hasn't moved.
        nav._laps_completed = 1
        nav._park_controller = _StubParkController()
        nav._parking_engaged = True

        for _ in range(tuning.escape.STUCK_TIMEOUT_FRAMES + 15):
            nav.step()

        assert any(cmd.speed_mps < 0 for cmd in gateway.commands), (
            "expected a reverse escape once the stuck timeout elapsed while parking"
        )

    def test_holding_after_parking_done_never_reverses(self, waypoints):
        """Once parked (done), stationary holding must never be read as stuck."""
        gateway = _FakeGateway(Pose(x=1.5, y=0.05, yaw=-math.pi / 2), lidar=None)
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav._laps_completed = 1
        nav._park_controller = _StubParkController(done=True)
        nav._parking_engaged = True

        for _ in range(tuning.escape.STUCK_TIMEOUT_FRAMES + 10):
            nav.step()

        assert all(cmd.speed_mps == 0.0 for cmd in gateway.commands)


class TestStuckEscapeRearBlocked:
    """2026-08-04: a robot wedged with reverse blocked used to just hold and
    reset the stuck detector forever, re-arming the same forward command that
    had already failed -- confirmed on real hardware as frozen at one
    position for 27s straight (see docs/known-issues-backlog.md). When
    forward has room, it should get a real forward escape at full steering
    lock instead of an indefinite hold.
    """

    def test_forward_room_forces_a_forward_escape_instead_of_holding(self, waypoints):
        ranges = _scan_with_sectors(back=0.09)  # rear blocked, front stays LIDAR_DEFAULT_FAR
        gateway = _FakeGateway(
            Pose(x=0.0, y=0.0, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        for _ in range(tuning.escape.STUCK_TIMEOUT_FRAMES + 5):
            nav.step()

        escape_cmds = [c for c in gateway.commands if c.speed_mps > 0 and abs(c.steering_norm) > 0.5]
        assert escape_cmds, "expected a forward escape at strong steering once stuck with rear blocked"
        assert nav.debug_snapshot.active_maneuver_type == ManeuverType.STUCK_FORWARD.value

    def test_both_ends_blocked_still_holds(self, waypoints):
        """Genuinely sandwiched (front AND rear blocked): holding is still correct."""
        ranges = _scan_with_sectors(front=0.06, back=0.09)
        gateway = _FakeGateway(
            Pose(x=0.0, y=0.0, yaw=0.0),
            LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)),
        )
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        for _ in range(tuning.escape.STUCK_TIMEOUT_FRAMES + 5):
            nav.step()

        assert all(c.speed_mps >= 0 for c in gateway.commands), "must not reverse into an unseen rear wall"
        assert not any(abs(c.steering_norm) > 0.5 and c.speed_mps > 0 for c in gateway.commands), (
            "must not force a forward escape when forward is also blocked"
        )


class _StubParkController:
    """Minimal ParkController stand-in: always drives in place, never finishes
    on its own unless constructed with ``done=True``.
    """

    def __init__(self, done: bool = False) -> None:
        self.is_done = done
        self.is_repositioning = False
        self.section = None
        self.staging = (0.0, 0.0)

    def update(self, robot_pos, robot_yaw):
        from src.navigation.maneuvers.parking import ParkCommand

        if self.is_done:
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")
        return ParkCommand(linear=0.1, steering=0.0, done=False, phase="enter")


class TestMissingSensorsDegradeSafely:
    def test_missing_pose_stops(self, waypoints):
        gateway = _FakeGateway(pose=None, lidar=None)  # type: ignore[arg-type]
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=NavigationTuning())

        nav.step()

        assert gateway.commands == [DriveCommand(speed_mps=0.0, steering_norm=0.0)]

    def test_missing_lidar_does_not_use_full_speed(self, waypoints):
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), lidar=None)
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        nav.step()

        assert gateway.commands
        assert gateway.commands[-1].speed_mps <= tuning.speed.SLOW_SPEED


class TestEscapeEscalation:
    """_maybe_escalate: after ESCALATE_AFTER_ATTEMPTS consecutive failed escapes,
    the next escape should reverse longer and swing to the opposite side instead
    of repeating an identical pulse into the same wall (open recommendation from
    the 2026-07-03 navigation review, docs/internal/2026-07-03-navigation-review-findings.md).
    """

    @staticmethod
    def _navigator(waypoints) -> CoreNavigator:
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), lidar=None)
        return CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=NavigationTuning())

    @staticmethod
    def _maneuver(steering: float = 0.4, duration: int = 6) -> EscapeManeuver:
        return EscapeManeuver(
            maneuver_type=ManeuverType.K_TURN,
            steering=steering,
            speed=-0.2,
            duration_frames=duration,
        )

    def test_within_threshold_returns_maneuver_unchanged(self, waypoints):
        nav = self._navigator(waypoints)
        maneuver = self._maneuver()

        for count in range(1, nav._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1):
            nav._escape_count = count
            assert nav._maybe_escalate(maneuver) is maneuver

    def test_beyond_threshold_flips_side_and_extends_duration(self, waypoints):
        nav = self._navigator(waypoints)
        maneuver = self._maneuver(steering=0.4, duration=6)
        nav._escape_count = nav._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1
        starting_sign = nav._escape_steer_sign

        result = nav._maybe_escalate(maneuver)

        assert result.duration_frames == min(6 * 2, nav._tuning.escape.MAX_ESCAPE_FRAMES)
        assert math.copysign(1.0, result.steering) == -starting_sign
        assert abs(result.steering) == pytest.approx(abs(maneuver.steering))
        assert result.maneuver_type == maneuver.maneuver_type
        assert result.speed == maneuver.speed

    def test_successive_escalations_alternate_sides(self, waypoints):
        nav = self._navigator(waypoints)
        maneuver = self._maneuver(steering=0.4, duration=6)
        nav._escape_count = nav._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1

        first = nav._maybe_escalate(maneuver)
        second = nav._maybe_escalate(maneuver)

        assert math.copysign(1.0, first.steering) == -math.copysign(1.0, second.steering)

    def test_duration_caps_at_max_escape_frames(self, waypoints):
        nav = self._navigator(waypoints)
        maneuver = self._maneuver(steering=0.4, duration=nav._tuning.escape.MAX_ESCAPE_FRAMES)
        nav._escape_count = nav._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1

        result = nav._maybe_escalate(maneuver)

        assert result.duration_frames == nav._tuning.escape.MAX_ESCAPE_FRAMES

    def test_straight_reverse_has_no_side_to_flip(self, waypoints):
        """A zero-steering escape (e.g. a straight stuck-reverse) stays at zero
        when escalated — only its duration should extend.
        """
        nav = self._navigator(waypoints)
        maneuver = self._maneuver(steering=0.0, duration=6)
        nav._escape_count = nav._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1

        result = nav._maybe_escalate(maneuver)

        assert result.steering == 0.0
        assert result.duration_frames == 12


class TestEscapeEscalationIntegration:
    """End-to-end through CoreNavigator.step(): a threat that never clears must
    drive a sequence of full escape maneuvers whose duration/side escalates,
    not the same short pulse repeated forever.
    """

    def test_persistent_front_threat_escalates_after_repeated_escapes(self, waypoints):
        ranges = _scan_with_sectors(front=0.06)
        gateway = _FakeGateway(Pose(x=0.0, y=0.0, yaw=0.0), LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES)))
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        maneuvers_begun: list[EscapeManeuver] = []
        was_active = False
        target = tuning.escape.ESCALATE_AFTER_ATTEMPTS + 2
        for _ in range(500):
            nav.step()
            now_active = nav._active_maneuver is not None
            if now_active and not was_active:
                maneuvers_begun.append(nav._active_maneuver)
                if len(maneuvers_begun) >= target:
                    break
            was_active = now_active

        assert len(maneuvers_begun) == target, "threat never cleared, so every maneuver should re-trigger a new escape"

        pre_escalation = maneuvers_begun[: tuning.escape.ESCALATE_AFTER_ATTEMPTS]
        escalated = maneuvers_begun[tuning.escape.ESCALATE_AFTER_ATTEMPTS]
        last_pre_escalation = pre_escalation[-1]

        assert escalated.duration_frames > last_pre_escalation.duration_frames
        if last_pre_escalation.steering:
            assert math.copysign(1.0, escalated.steering) == -math.copysign(1.0, last_pre_escalation.steering)


class TestEscapeEscalationSurvivesInterveningNormalDriveTicks:
    """A brief normal_drive tick between escape attempts must not reset the
    escalation counter unless the robot actually moved.

    Confirmed on real hardware 2026-08-04 (run_20260804_213147): with the
    robot genuinely pinned in place, SIDE_CORRECTION's brief creep read as
    "not critical" for one tick between escapes, which reset escape_count to
    0 every single cycle -- so it never reached ESCALATE_AFTER_ATTEMPTS and
    never escalated, for 34+ seconds. The threat toggling on/off each
    decision tick (rather than staying permanently critical, as in
    TestEscapeEscalationIntegration above) is what reproduces the gap that
    test doesn't cover: a *fixed* threat never even reaches the normal_drive
    reset branch, since a new escape re-triggers before the old one clears.
    """

    def test_oscillating_threat_without_progress_still_escalates(self, waypoints):
        close = tuple(_scan_with_sectors(front=0.06))
        far = tuple(np.full(NUM_RAYS, LIDAR_DEFAULT_FAR).tolist())

        class _OscillatingGateway(_FakeGateway):
            def __init__(self, pose):
                super().__init__(pose, LidarScan(ranges_m=close, angles_rad=tuple(ANGLES)))
                self._tick = 0

            def get_lidar_scan(self):
                self._tick += 1
                ranges = close if self._tick % 2 else far
                return LidarScan(ranges_m=ranges, angles_rad=tuple(ANGLES))

        gateway = _OscillatingGateway(Pose(x=0.0, y=0.0, yaw=0.0))
        tuning = NavigationTuning()
        nav = CoreNavigator(gateway=gateway, waypoints=waypoints, num_laps=1, tuning=tuning)

        maneuvers_begun: list[EscapeManeuver] = []
        was_active = False
        target = tuning.escape.ESCALATE_AFTER_ATTEMPTS + 2
        for _ in range(1000):
            nav.step()
            now_active = nav._active_maneuver is not None
            if now_active and not was_active:
                maneuvers_begun.append(nav._active_maneuver)
                if len(maneuvers_begun) >= target:
                    break
            was_active = now_active

        assert len(maneuvers_begun) == target, "expected escapes to keep re-triggering with the threat oscillating"

        pre_escalation = maneuvers_begun[: tuning.escape.ESCALATE_AFTER_ATTEMPTS]
        escalated = maneuvers_begun[tuning.escape.ESCALATE_AFTER_ATTEMPTS]
        assert escalated.duration_frames > pre_escalation[-1].duration_frames, (
            "escape_count must survive the intervening normal_drive tick since the robot never moved"
        )
