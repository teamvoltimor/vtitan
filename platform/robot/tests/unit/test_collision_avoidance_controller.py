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
    mask_mapped_obstacles,
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


class TestNoReturnRaysExcluded:
    """A single no-return (+inf, beyond LIDAR max range) ray inside a sector
    must not poison that sector's mean/min/max to infinity -- a real 360 deg
    scan routinely has scattered no-return rays, and every sector consumer
    (the OLED's displayed clearance, detect_threat_direction, K-turn side
    selection) reads mean_range_m/min_range_m as a real distance.
    """

    def test_forward_clearance_ignores_a_stray_no_return_ray(self, controller):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i] = np.inf  # one no-return ray inside the forward sector
        clearance = controller.compute_forward_clearance(ranges, ANGLES)
        assert math.isfinite(clearance)
        assert clearance == pytest.approx(LIDAR_DEFAULT_FAR)

    def test_forward_clearance_all_no_return_falls_back_to_default(self, controller):
        ranges = np.full(NUM_RAYS, np.inf)
        assert controller.compute_forward_clearance(ranges, ANGLES) == 10.0


class TestForwardPathRisk:
    """Risk is judged over the forward driving lane, not the full 360 sweep.

    Corridor side walls sit within the slow-zone distance but are not obstacles
    the robot is about to hit; flagging them pins the speed to a crawl for a whole
    lap (the ~0.16 m/s corridor creep). Only obstacles in the robot's path count.
    """

    def test_side_walls_are_not_risk(self, controller):
        ranges = _scan()
        # Close walls to the left and right (+-pi/2), outside the driving lane.
        # Must clear path_half_width (chassis half-width + margin, 0.20m for the
        # 0.20m-wide chassis) with room to spare, or this stops testing "outside
        # the lane" and starts testing the boundary instead.
        for center in (math.pi / 2, -math.pi / 2):
            i = _index_for(center)
            ranges[i - 6 : i + 6] = 0.35
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


class TestMaskMappedObstacles:
    """Returns attributable to a planner-owned obstacle are withheld from the
    escape trigger; everything else keeps the full reactive guard.

    The split is by provenance, not distance — so these tests pin that a
    mapped position at a given range is masked while an identical return at a
    position nothing owns is not.
    """

    _MASK_RADIUS = 0.12

    def test_return_on_a_mapped_position_is_masked(self):
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.08  # inside contact_dist

        masked = mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.isinf(masked[i]), "a return landing on a mapped sign must be withheld"

    def test_unmapped_return_at_the_same_range_is_untouched(self):
        """The guard is removed for the mapped obstacle only, not for the range."""
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.08

        # Mapped sign is off to the side; the forward return belongs to nothing.
        masked = mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.0, 0.9)], self._MASK_RADIUS)

        assert masked[i] == pytest.approx(0.08)

    def test_masking_downgrades_risk_from_critical_to_safe(self, controller):
        """The whole point: the same scan is CRITICAL raw and SAFE once masked."""
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.08

        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.CRITICAL
        masked = mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)
        assert controller.assess_risk(masked, ANGLES) == RiskLevel.SAFE

    def test_mapped_positions_are_world_frame_not_robot_frame(self):
        """Ray endpoints are placed using the robot's pose, so a sign's world
        position masks the correct ray whatever the robot's heading is.

        Pinning this because a robot-frame reading would appear to work at
        yaw=0 — the pose the other tests here use — and silently mask the wrong
        bearing everywhere else on the track.
        """
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i - 4 : i + 4] = 0.08
        # Robot at (1.0, 2.0) facing north: the forward return lands at
        # (1.0, 2.08), NOT at (0.08, 0).
        pose = (1.0, 2.0, math.pi / 2)

        masked_world = mask_mapped_obstacles(ranges, ANGLES, pose, [(1.0, 2.08)], self._MASK_RADIUS)
        masked_body = mask_mapped_obstacles(ranges, ANGLES, pose, [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.isinf(masked_world[i])
        assert masked_body[i] == pytest.approx(0.08)

    def test_zero_radius_and_empty_map_are_no_ops(self):
        """Both disable the split — the escape trigger sees the raw scan.

        ``escape_mask_radius_m = 0`` is the documented off-switch and the
        comparison every measurement of the split is read against, so it has to
        be exactly the old behaviour rather than approximately it.
        """
        ranges = _scan()
        ranges[_index_for(0.0)] = 0.08

        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.08, 0.0)], 0.0), ranges)
        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [], self._MASK_RADIUS), ranges)

    def test_no_return_rays_stay_infinite_and_never_become_nan(self):
        """``inf`` ranges have no endpoint to attribute.

        ``inf * cos(theta)`` is ``+-inf`` and ``inf - inf`` is ``nan``, so an
        unguarded distance test would turn every no-return ray into ``nan`` —
        which compares False everywhere and would quietly corrupt the sector
        helpers' min/mean rather than failing loudly.
        """
        ranges = _scan()
        ranges[:] = np.inf

        masked = mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.all(np.isinf(masked))
        assert not np.any(np.isnan(masked))

    def test_input_scan_is_not_mutated(self):
        """The raw scan still governs speed and the rear gate, so masking must
        return a copy rather than editing the caller's array in place.
        """
        ranges = _scan()
        i = _index_for(0.0)
        ranges[i] = 0.08

        mask_mapped_obstacles(ranges, ANGLES, (0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert ranges[i] == pytest.approx(0.08)
