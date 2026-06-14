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

NUM_RAYS = 360
# ROS LaserScan convention: index 0 = angle_min = -pi (directly behind).
ANGLES = np.linspace(-math.pi, math.pi, NUM_RAYS)


def _scan(default: float = 10.0) -> np.ndarray:
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
        ranges[:8] = 0.3
        ranges[-8:] = 0.3

        assert controller.detect_threat_direction(ranges, ANGLES) == "back"

    def test_wall_behind_back_even_without_angles(self, controller):
        # When angles are omitted, index 0 is assumed to be -pi, so a close
        # ray at index 0 is still the rear, not the front.
        ranges = _scan()
        ranges[:8] = 0.3
        ranges[-8:] = 0.3

        assert controller.detect_threat_direction(ranges) == "back"

    def test_wall_ahead_reports_front(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.3

        assert controller.detect_threat_direction(ranges, ANGLES) == "front"

    def test_obstacle_left_reports_left(self, controller):
        ranges = _scan()
        i = _index_for(math.pi / 2)  # +pi/2 = left
        ranges[i - 4 : i + 4] = 0.3

        assert controller.detect_threat_direction(ranges, ANGLES) == "left"

    def test_all_clear_reports_none(self, controller):
        assert controller.detect_threat_direction(_scan(), ANGLES) == "none"


class TestClearance:
    def test_forward_clearance_ignores_rear_wall(self, controller):
        ranges = _scan()
        ranges[:8] = 0.3
        ranges[-8:] = 0.3
        # Path ahead is clear even though a wall sits behind.
        assert controller.compute_forward_clearance(ranges, ANGLES) > 5.0

    def test_rear_clearance_sees_rear_wall(self, controller):
        ranges = _scan()
        ranges[:8] = 0.3
        ranges[-8:] = 0.3
        assert controller.compute_rear_clearance(ranges, ANGLES) == pytest.approx(0.3)

    def test_rear_clearance_clear_when_only_front_blocked(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.3
        assert controller.compute_rear_clearance(ranges, ANGLES) > 5.0


class TestEscapeDoesNotReverseIntoRearWall:
    def test_rear_threat_yields_no_kturn(self, controller):
        # A rear wall classifies as "back"; the escape table only reverses for
        # a "front" threat, so a rear wall never triggers a reverse K-turn.
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "back")
        assert maneuver is None
