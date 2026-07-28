"""Reading absolute heading off the walls.

Heading is otherwise unbounded: LidarLocalizer solves position with yaw *given*,
so the IMU is the sole source, and a BNO085 in UART-RVC mode is 6-axis with no
magnetometer. Its error is a ramp, and the sweeps say that is what decides
rounds -- 0.1 deg/s of drift took 28/28 to 13/28, while 20 cm of position error
costs nothing.

The track is a Manhattan world, so a sweep observes absolute heading mod 90
degrees. These pin that it works, that it declines to answer rather than
guessing when the scan does not support it, and the sampling trap that made the
first implementation silently useless.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.enums import Direction, Section

from src.navigation.start_conditions import start_pose
from src.navigation.track_geometry import corridor_widths_from_metadata
from src.navigation.wall_heading import (
    MIN_CONCENTRATION,
    estimate_yaw_from_walls,
    heading_error,
)
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.track_model import TrackModel

_RAYS = 360
_NOISE = 0.03


def _scan(track: TrackModel, x: float, y: float, yaw: float, noise: float = _NOISE, seed: int = 0):
    angles = np.linspace(-math.pi, math.pi, _RAYS)
    ranges = track.raycast_scan(x, y, yaw, angles)
    if noise:
        ranges = ranges + np.random.default_rng(seed).normal(0, noise, _RAYS)
    return ranges.tolist(), angles.tolist()


def _track_and_pose(width_mm: int = 1000, section: Section = Section.SOUTH):
    metadata = build_open_metadata(uniform_widths(width_mm), section, Direction.CLOCKWISE)
    geometry = corridor_widths_from_metadata(metadata.model_dump())
    by_name = geometry.to_widths_dict()
    by_name_str = {s.value.lower(): w for s, w in by_name.items()}
    x, y, yaw = start_pose(section, Direction.CLOCKWISE, by_name_str)
    return TrackModel(geometry), x, y, yaw


def _err_deg(estimated: float, true: float) -> float:
    return abs(math.degrees(heading_error(estimated, true)))


class TestAccuracy:
    @pytest.mark.parametrize("offset_deg", [0.0, 5.0, -8.0, 20.0, -25.0])
    def test_recovers_heading_under_noise(self, offset_deg: float) -> None:
        track, x, y, yaw = _track_and_pose()
        true = yaw + math.radians(offset_deg)
        ranges, angles = _scan(track, x, y, true)

        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=true + 0.15)

        assert estimated is not None
        assert _err_deg(estimated, true) < 3.0

    @pytest.mark.parametrize("section", list(Section))
    def test_works_from_every_corridor(self, section: Section) -> None:
        track, x, y, yaw = _track_and_pose(section=section)
        ranges, angles = _scan(track, x, y, yaw)

        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=yaw + 0.1)

        assert estimated is not None
        assert _err_deg(estimated, yaw) < 3.0

    def test_does_not_inherit_the_prior(self) -> None:
        """The walls decide the value; the prior only picks the quadrant.

        A estimator that quietly returned the prior would look perfect in every
        other test here and be worth nothing.
        """
        track, x, y, yaw = _track_and_pose()
        ranges, angles = _scan(track, x, y, yaw)

        close = estimate_yaw_from_walls(ranges, angles, prior_yaw=yaw + math.radians(12))
        assert close is not None
        assert _err_deg(close, yaw) < 3.0
        # Far from the truth, and far from the prior it was handed.
        assert _err_deg(close, yaw + math.radians(12)) > 5.0


class TestQuadrantResolution:
    """Heading is observed mod 90 degrees; the prior says which one is meant."""

    @pytest.mark.parametrize("k", [-2, -1, 0, 1, 2])
    def test_locks_to_the_quadrant_nearest_the_prior(self, k: int) -> None:
        track, x, y, yaw = _track_and_pose()
        ranges, angles = _scan(track, x, y, yaw)

        shifted = yaw + k * (math.pi / 2)
        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=shifted)

        assert estimated is not None
        assert _err_deg(estimated, shifted) < 3.0

    def test_a_prior_beyond_45_degrees_locks_to_the_wrong_quadrant(self) -> None:
        """The known limit: this bounds a drifting heading, it cannot rescue a lost one.

        Documented rather than guarded, because the caller cannot detect it
        either -- a 46 degree error is indistinguishable from a 44 degree one
        in the opposite direction when all you see is walls.
        """
        track, x, y, yaw = _track_and_pose()
        ranges, angles = _scan(track, x, y, yaw)

        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=yaw + math.radians(50))

        assert estimated is not None
        assert _err_deg(estimated, yaw) > 45.0


class TestRefusesToGuess:
    def test_returns_none_on_an_empty_scan(self) -> None:
        assert estimate_yaw_from_walls([], [], prior_yaw=0.0) is None

    def test_returns_none_when_every_ray_is_a_no_return(self) -> None:
        """Sanitised no-returns sit at max range and carry no wall direction."""
        angles = np.linspace(-math.pi, math.pi, _RAYS).tolist()
        ranges = [12.0] * _RAYS

        assert estimate_yaw_from_walls(ranges, angles, prior_yaw=0.0) is None

    def test_returns_none_on_noise_with_no_structure(self) -> None:
        angles = np.linspace(-math.pi, math.pi, _RAYS).tolist()
        ranges = np.random.default_rng(0).uniform(0.3, 2.0, _RAYS).tolist()

        assert estimate_yaw_from_walls(ranges, angles, prior_yaw=0.0) is None


class TestSamplingBaseline:
    """The trap that made the first implementation return nothing at all.

    Comparing *adjacent* returns is the obvious way to get a wall direction and
    is unusable here: at a typical 0.7 m wall distance neighbouring rays land
    12 mm apart against 30 mm of range noise, so the segment direction is mostly
    noise pointing radially. Concentration measured 0.08 with noise against 0.99
    without -- the signal is real and entirely buried.
    """

    def test_survives_noise_larger_than_adjacent_ray_spacing(self) -> None:
        track, x, y, yaw = _track_and_pose(width_mm=600)
        ranges, angles = _scan(track, x, y, yaw, noise=_NOISE)

        spacing = float(np.median(ranges)) * (2 * math.pi / _RAYS)
        assert spacing < _NOISE, "precondition: noise must exceed adjacent spacing"

        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=yaw + 0.1)
        assert estimated is not None
        assert _err_deg(estimated, yaw) < 3.0

    def test_still_works_with_no_noise(self) -> None:
        track, x, y, yaw = _track_and_pose()
        ranges, angles = _scan(track, x, y, yaw, noise=0.0)

        estimated = estimate_yaw_from_walls(ranges, angles, prior_yaw=yaw + 0.1)
        assert estimated is not None
        assert _err_deg(estimated, yaw) < 1.0


def test_concentration_threshold_is_below_a_real_scan() -> None:
    """A threshold above what real scans achieve would reject everything."""
    assert 0.0 < MIN_CONCENTRATION < 0.9
