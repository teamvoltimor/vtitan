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
