"""Unit tests for collision detection helpers (no ROS2 dependency)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.enums import RiskLevel

from src.navigation.collision import (
    assess_collision_risk,
    clamp_lidar_scan,
    measure_distance_in_direction,
    update_fwd_critical_count,
)


def _uniform_scan(distance: float, num_rays: int = 360) -> tuple[np.ndarray, np.ndarray]:
    """Uniform circular scan — all rays at the same distance."""
    angles = np.linspace(-math.pi, math.pi, num_rays, endpoint=False)
    ranges = np.full(num_rays, distance)
    return ranges, angles


def _directional_scan(
    forward_dist: float,
    side_dist: float = 1.0,
    num_rays: int = 360,
) -> tuple[np.ndarray, np.ndarray]:
    """Scan with a short reading ahead and open sides."""
    angles = np.linspace(-math.pi, math.pi, num_rays, endpoint=False)
    ranges = np.full(num_rays, side_dist)
    # Forward sector: rays within ±0.25 rad of 0
    forward_mask = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < 0.25
    ranges[forward_mask] = forward_dist
    return ranges, angles


# measure_distance_in_direction
class TestMeasureDistance:
    def test_uniform_scan_returns_distance(self) -> None:
        ranges, angles = _uniform_scan(1.5)
        d = measure_distance_in_direction(ranges, angles, target_angle=0.0)
        assert d == pytest.approx(1.5, abs=0.01)

    def test_no_rays_in_sector_returns_inf(self) -> None:
        ranges, angles = _uniform_scan(1.0, num_rays=4)
        # Very tight tolerance — no rays at exactly pi/3
        d = measure_distance_in_direction(ranges, angles, target_angle=math.pi / 3, tolerance=0.01)
        assert d == float("inf")

    def test_minimum_range_in_sector_selected(self) -> None:
        ranges, angles = _uniform_scan(2.0)
        # Inject a close reading near forward
        ranges[180] = 0.3  # angle near 0
        d = measure_distance_in_direction(ranges, angles, target_angle=0.0)
        assert d == pytest.approx(0.3, abs=0.01)

    def test_self_detection_filter(self) -> None:
        ranges, angles = _uniform_scan(0.05)  # all readings at self-detection threshold
        d = measure_distance_in_direction(ranges, angles, target_angle=math.pi / 2, filter_self_detection=True)
        assert d == float("inf")


# clamp_lidar_scan
class TestClampLidarScan:
    def test_inf_replaced_by_max_range(self) -> None:
        raw = np.array([float("inf"), 1.0, 2.0])
        clamped = clamp_lidar_scan(raw, max_range=5.0)
        assert clamped[0] == pytest.approx(5.0)
        assert clamped[1] == pytest.approx(1.0)

    def test_below_min_replaced_by_min_range(self) -> None:
        from shared.config.constants import RobotSpecs

        raw = np.array([0.01, 0.04, 1.0])
        clamped = clamp_lidar_scan(raw, max_range=12.0)
        assert clamped[0] == pytest.approx(RobotSpecs.LIDAR_MIN_RANGE)
        assert clamped[1] == pytest.approx(RobotSpecs.LIDAR_MIN_RANGE)
        assert clamped[2] == pytest.approx(1.0)

    def test_valid_ranges_unchanged(self) -> None:
        raw = np.array([0.5, 1.0, 2.5])
        clamped = clamp_lidar_scan(raw, max_range=12.0)
        np.testing.assert_array_almost_equal(clamped, raw)

    def test_original_array_not_mutated(self) -> None:
        raw = np.array([float("inf"), 1.0])
        original = raw.copy()
        clamp_lidar_scan(raw, max_range=5.0)
        np.testing.assert_array_equal(raw, original)


# update_fwd_critical_count
class TestUpdateFwdCriticalCount:
    def test_increments_when_below_threshold(self) -> None:
        ranges, angles = _directional_scan(forward_dist=0.05)
        count = update_fwd_critical_count(ranges, angles, critical_distance=0.10, current_count=2)
        assert count == 3

    def test_resets_when_clear(self) -> None:
        ranges, angles = _directional_scan(forward_dist=1.0)
        count = update_fwd_critical_count(ranges, angles, critical_distance=0.10, current_count=5)
        assert count == 0


# assess_collision_risk
class TestAssessCollisionRisk:
    def test_safe_when_clear(self) -> None:
        ranges, angles = _uniform_scan(1.5)
        risk, dists = assess_collision_risk(
            ranges,
            angles,
            critical_distance=0.10,
            fwd_critical_count=0,
            fwd_critical_threshold=2,
            is_open_challenge=True,
        )
        assert risk == RiskLevel.SAFE
        assert dists["forward"] == pytest.approx(1.5, abs=0.05)

    def test_critical_when_wall_close_and_debounced(self) -> None:
        ranges, angles = _directional_scan(forward_dist=0.06, side_dist=0.2)
        risk, _ = assess_collision_risk(
            ranges,
            angles,
            critical_distance=0.10,
            fwd_critical_count=3,  # debounced
            fwd_critical_threshold=2,
            is_open_challenge=True,
        )
        assert risk == RiskLevel.CRITICAL

    def test_single_spike_returns_safe_before_debounce(self) -> None:
        ranges, angles = _directional_scan(forward_dist=0.06, side_dist=0.5)
        risk, _ = assess_collision_risk(
            ranges,
            angles,
            critical_distance=0.10,
            fwd_critical_count=0,  # not yet debounced
            fwd_critical_threshold=2,
            is_open_challenge=True,
        )
        assert risk == RiskLevel.SAFE

    def test_obstacle_classification_in_open_space(self) -> None:
        # Wide sides (open corridor), debounced, obstacles challenge
        ranges, angles = _directional_scan(forward_dist=0.06, side_dist=0.6)
        risk, _ = assess_collision_risk(
            ranges,
            angles,
            critical_distance=0.10,
            fwd_critical_count=3,
            fwd_critical_threshold=2,
            is_open_challenge=False,  # obstacles challenge
        )
        assert risk == RiskLevel.OBSTACLE

    def test_gpu_artifact_large_forward_returns_critical(self) -> None:
        # Forward > 4 m is physically impossible inside 3×3 m track
        ranges, angles = _uniform_scan(5.0)
        risk, _ = assess_collision_risk(
            ranges,
            angles,
            critical_distance=0.10,
            fwd_critical_count=0,
            fwd_critical_threshold=2,
            is_open_challenge=True,
            is_simulation=True,
        )
        assert risk == RiskLevel.CRITICAL
