"""Unit tests for CollisionAvoidanceController LIDAR sectoring.

Regression coverage for the angle-aware sector logic: a wall *behind* the
robot must report as "back" (never "front"), so the escape logic never
reverses into an unseen wall. Sectors are derived from per-ray bearings
(0 rad = forward, +-pi = rear) and must agree across all helpers regardless
of the scan's index ordering.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.domain.enums import RiskLevel

from src.navigation.control.controllers.collision_avoidance_controller import (
    CollisionAvoidanceController,
)
from tests.test_constants import (
    ANGLES_FULL_ROTATION,
    FORWARD_SECTOR_INDICES,
    LIDAR_CLOSE_THREAT,
    LIDAR_DEFAULT_FAR,
    MIN_FORWARD_CLEARANCE,
    MIN_REAR_CLEARANCE,
    NUM_RAYS,
    REAR_SECTOR_INDICES,
    YAW_EAST,
)

ANGLES = ANGLES_FULL_ROTATION


def _scan(default: float = LIDAR_DEFAULT_FAR) -> np.ndarray:
    return np.full(NUM_RAYS, default)


def _index_for(angle_rad: float) -> int:
    return int(np.argmin(np.abs(ANGLES - angle_rad)))


@pytest.fixture()
def controller() -> CollisionAvoidanceController:
    return CollisionAvoidanceController()


class TestThreatDirection:
    def test_wall_behind_reports_back_not_front(self, controller):
        ranges = _scan()
        # Close returns near +-pi (rear cone), including index 0 (= -pi).
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "back"

    def test_wall_behind_back_even_without_angles(self, controller):
        # When angles are omitted, index 0 is assumed to be -pi, so a close
        # ray at index 0 is still the rear, not the front.
        ranges = _scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges) == "back"

    def test_wall_ahead_reports_front(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "front"

    def test_obstacle_left_reports_left(self, controller):
        ranges = _scan()
        i = _index_for(math.pi / 2)  # +pi/2 = left
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "left"

    def test_all_clear_reports_none(self, controller):
        assert controller.detect_threat_direction(_scan(), ANGLES) == "none"


class TestClearance:
    def test_forward_clearance_ignores_rear_wall(self, controller):
        ranges = _scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        # Path ahead is clear even though a wall sits behind.
        assert controller.compute_forward_clearance(ranges, ANGLES) > MIN_FORWARD_CLEARANCE

    def test_rear_clearance_sees_rear_wall(self, controller):
        ranges = _scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES) == pytest.approx(LIDAR_CLOSE_THREAT)

    def test_rear_clearance_clear_when_only_front_blocked(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES) > MIN_REAR_CLEARANCE


class TestEscapeDoesNotReverseIntoRearWall:
    def test_rear_threat_yields_no_kturn(self, controller):
        # A rear wall classifies as "back"; the escape table only reverses for
        # a "front" threat, so a rear wall never triggers a reverse K-turn.
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "back")
        assert maneuver is None


class TestKTurnSteersTowardClearerSide:
    """Ackermann reverse flips yaw response: +steering swings the nose right
    while reversing, -steering swings it left. The K-turn must pick its sign
    from which side is actually clearer, not a fixed direction, or it swings
    into the tighter wall every other attempt.
    """

    def test_steers_left_when_left_is_clearer(self, controller):
        ranges = _scan()
        i = _index_for(-math.pi / 2)  # tighten the right side
        ranges[i - 6 : i + 6] = 0.15
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES)
        assert maneuver.steering < 0  # swing toward the clearer left side

    def test_steers_right_when_right_is_clearer(self, controller):
        ranges = _scan()
        i = _index_for(math.pi / 2)  # tighten the left side
        ranges[i - 6 : i + 6] = 0.15
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES)
        assert maneuver.steering > 0  # swing toward the clearer right side

    def test_defaults_to_right_without_lidar_data(self, controller):
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front")
        assert maneuver.steering > 0


class TestSelfDetectionFilter:
    """Chassis/cable reflections at <= 0.08 m on side/rear sectors must not
    permanently read as a wall — that would block every reverse escape.
    """

    def test_rear_self_reflection_does_not_block_reverse(self, controller):
        ranges = _scan()
        i = _index_for(math.pi)
        ranges[i - 4 : i + 4] = 0.05  # inside the self-detection radius
        assert controller.compute_rear_clearance(ranges, ANGLES) > 5.0

    def test_rear_real_wall_beyond_self_radius_still_detected(self, controller):
        ranges = _scan()
        i = _index_for(math.pi)
        ranges[i - 4 : i + 4] = 0.15  # beyond self-detection radius: a real wall
        assert controller.compute_rear_clearance(ranges, ANGLES) == pytest.approx(0.15)

    def test_side_self_reflection_does_not_report_as_threat(self, controller):
        ranges = _scan()
        i = _index_for(math.pi / 2)
        ranges[i - 4 : i + 4] = 0.05
        assert controller.detect_threat_direction(ranges, ANGLES) == "none"

    def test_forward_near_contact_is_not_filtered(self, controller):
        # The forward bearing must never be self-detection filtered: a genuine
        # near-contact obstacle has to still register.
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.05
        assert controller.detect_threat_direction(ranges, ANGLES) == "front"


class TestForwardPathRisk:
    """Risk is judged over the forward driving lane, not the full 360 sweep.

    Corridor side walls sit within the slow-zone distance but are not obstacles
    the robot is about to hit; flagging them pins the speed to a crawl for a whole
    lap (the ~0.16 m/s corridor creep). Only obstacles in the robot's path count.
    """

    def test_side_walls_are_not_risk(self, controller):
        ranges = _scan()
        # Close walls to the left and right (+-pi/2), outside the driving lane.
        for center in (math.pi / 2, -math.pi / 2):
            i = _index_for(center)
            ranges[i - 6 : i + 6] = 0.20
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.SAFE

    def test_forward_obstacle_within_contact_is_critical(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.08  # < contact_dist (0.10)
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.CRITICAL

    def test_forward_obstacle_in_slow_zone_is_obstacle(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.20  # contact_dist < 0.20 < slow_dist (0.25)
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.OBSTACLE
