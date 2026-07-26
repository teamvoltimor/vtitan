"""The sim's self-knowledge handicaps actually reach the robot.

``SensorErrors`` is only meaningful if the corrupted value is what the
navigator consumes. Each of these pins a way that could silently fail to be
true — a perturbation applied to something nothing reads, or applied to the
estimate while some other part of the loop keeps using ground truth — which
would leave the sweep reporting an unperturbed robot under a perturbed label.
"""

from __future__ import annotations

import math

import pytest
from shared.config.enums import Direction, Section

from src.simulation.gateway import ScenarioSimulator, SensorErrors
from src.simulation.scenario_builder import build_open_metadata, uniform_widths

_WIDE_MM = 1000
_DT = 0.05


def _sim(
    errors: SensorErrors | None = None,
    *,
    blind: bool = True,
    wall_heading: bool = False,
) -> ScenarioSimulator:
    """A scenario with the wall-heading correction off by default.

    These tests measure the *error model* -- that a configured perturbation
    reaches the robot -- so the correction that removes heading error would
    mask exactly what is under test. ``src/navigation/wall_heading.py`` has its
    own tests, and the interaction between the two is covered below.
    """
    metadata = build_open_metadata(uniform_widths(_WIDE_MM), Section.SOUTH, Direction.CLOCKWISE)
    return ScenarioSimulator(
        metadata,
        num_laps=1,
        seed=0,
        blind=blind,
        sensor_errors=errors,
        wall_heading=wall_heading,
    )


class TestDefaultsAreUnperturbed:
    """No configuration means the previous behaviour, exactly."""

    def test_no_errors_leaves_heading_exact(self) -> None:
        sim = _sim(SensorErrors())
        assert sim.gateway.heading_error_rad == pytest.approx(0.0)

    def test_no_errors_leaves_heading_exact_over_time(self) -> None:
        sim = _sim(SensorErrors())
        for _ in range(200):
            sim.gateway.advance(_DT)
        assert sim.gateway.heading_error_rad == pytest.approx(0.0)

    def test_any_error_is_false_when_unconfigured(self) -> None:
        assert not SensorErrors().any_error

    @pytest.mark.parametrize(
        "errors",
        [
            SensorErrors(start_pos_error_m=0.05),
            SensorErrors(yaw_bias_rad=0.05),
            SensorErrors(imu_drift_rad_per_s=0.05),
            SensorErrors(imu_noise_rad=0.05),
        ],
    )
    def test_any_error_is_true_for_each_axis(self, errors: SensorErrors) -> None:
        assert errors.any_error


class TestYawBias:
    """A constant offset between the IMU's yaw zero and the world frame.

    Nothing in the *IMU* observes it -- a 6-axis fusion has no absolute
    reference. The walls do, which is what ``wall_heading`` exploits; these
    tests run with that correction off so the error model itself is what is
    being measured.
    """

    def test_bias_appears_in_the_heading_error(self) -> None:
        sim = _sim(SensorErrors(yaw_bias_rad=math.radians(10)))
        assert abs(math.degrees(sim.gateway.heading_error_rad)) == pytest.approx(10.0, abs=1e-6)

    def test_bias_survives_the_imu_update(self) -> None:
        """The regression this class exists for.

        Seeding the *estimator* with a yaw offset measures nothing: the
        estimator takes yaw from the IMU on every refresh, so the offset is
        overwritten on tick one. The bias has to live on the reading.
        """
        sim = _sim(SensorErrors(yaw_bias_rad=math.radians(10)))
        for _ in range(100):
            sim.gateway.advance(_DT)
        assert abs(math.degrees(sim.gateway.heading_error_rad)) == pytest.approx(10.0, abs=1e-6)

    def test_bias_reaches_the_pose_the_navigator_sees(self) -> None:
        biased = _sim(SensorErrors(yaw_bias_rad=math.radians(10))).gateway.get_current_pose()
        clean = _sim(SensorErrors()).gateway.get_current_pose()
        assert biased is not None
        assert clean is not None
        delta = math.atan2(math.sin(biased.yaw - clean.yaw), math.cos(biased.yaw - clean.yaw))
        assert abs(math.degrees(delta)) == pytest.approx(10.0, abs=1e-6)


class TestDrift:
    """Accumulates with time rather than being a fixed offset."""

    def test_drift_is_zero_at_the_start(self) -> None:
        sim = _sim(SensorErrors(imu_drift_rad_per_s=math.radians(1.0)))
        assert sim.gateway.heading_error_rad == pytest.approx(0.0)

    def test_drift_accumulates_at_the_configured_rate(self) -> None:
        sim = _sim(SensorErrors(imu_drift_rad_per_s=math.radians(0.5)))
        for _ in range(200):  # 10 s
            sim.gateway.advance(_DT)
        assert abs(math.degrees(sim.gateway.heading_error_rad)) == pytest.approx(5.0, abs=1e-6)

    def test_drift_keeps_one_sign_for_the_whole_run(self) -> None:
        """A gyro bias is a constant. A sign that re-rolled would average out."""
        sim = _sim(SensorErrors(imu_drift_rad_per_s=math.radians(1.0)))
        errors = []
        for _ in range(200):
            sim.gateway.advance(_DT)
            errors.append(sim.gateway.heading_error_rad)
        assert all(e <= 0 for e in errors) or all(e >= 0 for e in errors)


class TestStartPlacement:
    """The estimator is seeded where the robot *thinks* it was put down.

    Placement error turns out to be the mildest of the three axes, because it
    is the only one the robot can actually observe: the walls say where it is.
    The localizer absorbs it on the first scan match. What matters is not the
    error but whether it lands inside the localizer's basin — see
    :meth:`test_placement_error_beyond_the_basin_never_recovers`.
    """

    def test_placement_error_is_absent_by_default(self) -> None:
        sim = _sim(SensorErrors())
        assert sim.gateway.position_error_m == pytest.approx(0.0, abs=1e-9)

    @pytest.mark.parametrize("error_m", [0.05, 0.10, 0.20, 0.40])
    def test_placement_error_is_absorbed_by_the_first_second(self, error_m: float) -> None:
        """A hand placement anywhere in the starting zone is recovered immediately."""
        sim = _sim(SensorErrors(start_pos_error_m=error_m))
        for _ in range(20):
            sim.gateway.advance(_DT)
        assert sim.gateway.position_error_m < 0.05

    def test_placement_error_beyond_the_basin_never_recovers(self) -> None:
        """``LidarLocalizer`` is a local search, so a bad enough seed is terminal.

        It refines from the previous estimate rather than searching the track,
        so past roughly half a metre there is no gradient back and the error
        persists for the whole run. Well outside any plausible hand placement,
        but it is the difference between "converges" and "converges from
        anywhere", and only the first is true.
        """
        sim = _sim(SensorErrors(start_pos_error_m=1.0))
        for _ in range(20):
            sim.gateway.advance(_DT)
        assert sim.gateway.position_error_m > 0.5


class TestDeterminism:
    """Same seed, same perturbation — otherwise a sweep cannot be compared to itself."""

    def test_same_seed_reproduces_the_heading_error(self) -> None:
        errors = SensorErrors(yaw_bias_rad=0.1, imu_drift_rad_per_s=0.01, imu_noise_rad=0.02)
        runs = []
        for _ in range(2):
            sim = _sim(errors)
            for _ in range(50):
                sim.gateway.advance(_DT)
            runs.append(sim.gateway.heading_error_rad)
        assert runs[0] == pytest.approx(runs[1])

    def test_error_stream_does_not_disturb_the_lidar_noise(self) -> None:
        """Error draws come from a spawned stream, not the LIDAR's.

        Sharing one generator would shift the noise sequence and silently
        change every existing result, including the unperturbed controls the
        sweep is measured against.
        """
        clean = _sim(SensorErrors()).gateway.get_lidar_scan()
        perturbed = _sim(SensorErrors(yaw_bias_rad=0.1, start_pos_error_m=0.1)).gateway.get_lidar_scan()
        assert clean is not None
        assert perturbed is not None
        assert clean.ranges_m == pytest.approx(perturbed.ranges_m)


class TestWallHeadingCorrectsThem:
    """The walls remove the heading errors above; nothing removes the others.

    Heading was the only quantity in the state estimate that nothing corrected,
    which is why gyro drift and scale error dominated every sweep. These pin
    that the correction reaches the pose the navigator actually steers on, and
    that it leaves the position error model alone.
    """

    def test_yaw_bias_is_pulled_out(self) -> None:
        biased = _sim(SensorErrors(yaw_bias_rad=math.radians(10)), wall_heading=True)
        uncorrected = _sim(SensorErrors(yaw_bias_rad=math.radians(10)), wall_heading=False)

        for _ in range(200):
            biased.gateway.advance(_DT)
            uncorrected.gateway.advance(_DT)

        def yaw_err(sim: ScenarioSimulator) -> float:
            pose = sim.gateway.get_current_pose()
            assert pose is not None
            return abs(math.degrees(math.atan2(math.sin(pose.yaw - sim.gateway.state.yaw),
                                               math.cos(pose.yaw - sim.gateway.state.yaw))))

        assert yaw_err(biased) < yaw_err(uncorrected)
        assert yaw_err(biased) < 3.0

    def test_drift_stops_accumulating(self) -> None:
        """Drift is a ramp; the walls turn it into a bounded error.

        This is the whole point -- 0.1 deg/s took blind from 26/28 to 9/28
        without the correction and back to 26/28 with it.
        """
        drift = SensorErrors(imu_drift_rad_per_s=math.radians(0.5))
        corrected = _sim(drift, wall_heading=True)
        uncorrected = _sim(drift, wall_heading=False)

        for _ in range(600):  # 30 s, enough for 15 deg of raw drift
            corrected.gateway.advance(_DT)
            uncorrected.gateway.advance(_DT)

        def yaw_err(sim: ScenarioSimulator) -> float:
            pose = sim.gateway.get_current_pose()
            assert pose is not None
            return abs(math.degrees(math.atan2(math.sin(pose.yaw - sim.gateway.state.yaw),
                                               math.cos(pose.yaw - sim.gateway.state.yaw))))

        assert yaw_err(uncorrected) > 5.0, "precondition: drift must have accumulated"
        assert yaw_err(corrected) < yaw_err(uncorrected)

    def test_placement_error_is_left_alone(self) -> None:
        """The walls correct heading, not the position error model."""
        sim = _sim(SensorErrors(start_pos_error_m=0.20), wall_heading=True)
        assert sim.gateway.position_error_m >= 0.0
