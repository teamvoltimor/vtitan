"""The simulated LIDAR must look like the C1, not like a clean raycast.

MEASURED 2026-09-14 against 5,763,600 real rays from ``/scan`` over three of the
2026-09-13/14 hardware rounds. The model before this was Gaussian noise plus a
uniform 1% dropout, which is far cleaner than the real sensor in exactly the
0.04-0.10 m band where ``contact_dist`` and the escape gates live. That is why a
256-scenario corpus could not arbitrate those knobs: it was answering about a
sensor the robot does not have.

Ground truth these tests assert against:

    whole sweep    no-return 25.4%   sub-floor (< 0.044) 6.9%
    |bearing| 25-60 deg   no-return 68.9%   sub-floor 31.1%
    outside the bands     no-return  9.5%   sub-floor  0.13%

Tolerances are wide on purpose. These pin the ORDER OF MAGNITUDE and the SHAPE
(bands vs uniform), which is what was wrong before -- 1% against 25%, and zero
sub-floor against 6.9%. They are not a re-measurement of the sensor.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.domain.enums import Direction, Section

from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.kinematics import AckermannState
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.simulated_hardware_gateway import SimulatedHardwareGateway
from src.simulation.track_model import TrackModel

_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
# The nav-side filter floor. Hardcoded rather than imported because the point of
# these tests is that the SENSOR model straddles it correctly; reading it from
# the same config the model reads would let both move together and assert
# nothing.
_FILTER_FLOOR_M = 0.044
_TICKS = 300
_DT = 0.05


def _gateway() -> SimulatedHardwareGateway:
    metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
    return SimulatedHardwareGateway(
        track=TrackModel(corridor_widths_from_metadata(metadata.model_dump())),
        initial_state=AckermannState(x=1.5, y=0.45, yaw=0.0),
    )


def _sweep_stats() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(bearings, no_return_fraction, sub_floor_fraction)`` per ray over a run."""
    gw = _gateway()
    angles = np.asarray(gw.get_lidar_scan().angles_rad)
    drops = np.zeros(angles.size)
    lows = np.zeros(angles.size)
    for _ in range(_TICKS):
        gw.advance(_DT)
        ranges = np.asarray(gw.get_lidar_scan().ranges_m)
        # sanitize_lidar_ranges substitutes MAX_RANGE for non-finite, exactly as
        # the ROS2 node does, so a dropout is observable only as max range.
        drops += ranges >= RobotSpecs.LIDAR_MAX_RANGE - 0.01
        lows += ranges < _FILTER_FLOOR_M
    return angles, drops / _TICKS, lows / _TICKS


class TestChassisOcclusionBands:
    """The dropouts are geometry, not a scalar rate."""

    def test_the_bands_are_far_blinder_than_the_rest_of_the_sweep(self) -> None:
        angles, drop, _ = _sweep_stats()
        band = (np.abs(angles) >= math.radians(25.0)) & (np.abs(angles) <= math.radians(60.0))

        in_band = drop[band].mean()
        out_band = drop[~band].mean()

        assert in_band == pytest.approx(0.689, abs=0.10), "band dropout should track the measured 68.9%"
        assert out_band < 0.20, "outside the bands the real sweep is clean (9.5%)"
        # The shape is the point: a uniform rate cannot produce this ratio, and
        # a uniform rate is what the model used to have.
        assert in_band > 3.0 * out_band

    def test_returns_inside_the_bands_are_chassis_reflections(self) -> None:
        """Whatever does come back in a band must be BELOW the nav filter floor.

        A chassis reflection that reads as a valid range is the defect this
        whole model exists to stop reproducing: the old code clipped every short
        ray to LIDAR_MIN_RANGE = 0.045, and since 0.045 > 0.044 those survived
        the filter as valid 4.5 cm obstacles.
        """
        gw = _gateway()
        angles = np.asarray(gw.get_lidar_scan().angles_rad)
        band = (np.abs(angles) >= math.radians(25.0)) & (np.abs(angles) <= math.radians(60.0))

        returned: list[float] = []
        for _ in range(_TICKS):
            gw.advance(_DT)
            ranges = np.asarray(gw.get_lidar_scan().ranges_m)
            in_band = ranges[band]
            returned.extend(in_band[in_band < RobotSpecs.LIDAR_MAX_RANGE - 0.01].tolist())

        assert returned, "the bands must not be 100% dropout, 31% of real rays do return"
        values = np.asarray(returned)
        assert (values < _FILTER_FLOOR_M).mean() > 0.95, "measured: 99.7% of band returns are sub-floor"
        assert values.min() >= 0.0


class TestSubFloorReturnsExist:
    """The filter the robot runs must actually have something to filter."""

    def test_the_sweep_contains_sub_floor_returns(self) -> None:
        _, _, low = _sweep_stats()

        assert low.mean() > 0.01, (
            "the simulator used to clip to LIDAR_MIN_RANGE = 0.045, which is ABOVE "
            "min_valid_range_m = 0.044, so it produced exactly zero sub-floor rays "
            "and min_valid_range_m was never exercised in any sweep"
        )

    def test_no_ray_piles_up_on_the_old_clip_value(self) -> None:
        """0.045 was where every short ray used to land. Real hardware: 471 of 5.76M."""
        gw = _gateway()
        piled = 0
        total = 0
        for _ in range(_TICKS):
            gw.advance(_DT)
            ranges = np.asarray(gw.get_lidar_scan().ranges_m)
            total += ranges.size
            piled += int(np.isclose(ranges, RobotSpecs.LIDAR_MIN_RANGE, atol=1e-9).sum())

        assert piled / total < 1e-3


class TestWholeSweepMatchesHardware:
    def test_overall_dropout_is_the_measured_order_of_magnitude(self) -> None:
        _, drop, _ = _sweep_stats()

        # 25.4% measured. The old model gave 1%, which is what this guards.
        assert 0.10 < drop.mean() < 0.45
