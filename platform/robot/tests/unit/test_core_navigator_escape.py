"""Unit tests for CoreNavigator's escape-maneuver arbitration.

Uses a minimal fake HardwareGateway (implements the HardwareGateway Protocol
structurally) to drive CoreNavigator.step() directly, without ROS2 or the
simulator, so these scenarios can be pinned exactly: a wall dead ahead AND
dead behind must never produce a reversing command.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.control.controllers import EscapeManeuver, ManeuverType
from src.navigation.core_navigator import CoreNavigator
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
