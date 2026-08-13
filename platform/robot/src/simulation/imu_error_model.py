"""What the simulated robot can be wrong about regarding its own pose.

:class:`SensorErrors` configures the perturbation; :class:`ImuErrorModel`
applies it to produce the yaw the IMU actually reports, given the true yaw,
elapsed time and accumulated rotation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
  import numpy as np


@dataclass(frozen=True, slots=True)
class SensorErrors:
    """Imperfections in what the robot knows about itself, as opposed to the track.

    The layout is withheld by ``blind``; this withholds the two things the sim
    otherwise hands over for free about the *robot*:

    Where it starts. The estimator is normally seeded with the exact pose the
    body was placed at, which no operator can supply — the robot is set down
    by hand somewhere inside a starting zone, not on a surveyed point. The
    localizer can only correct that error by matching scans, so a bad seed is
    a real search problem, not a bookkeeping one.

    Which way it is pointing. IMU yaw is otherwise ground truth forever. A
    BNO085 drifts, and blind mode leans on heading harder than anything else
    does: ``corridor_estimator.section_from_heading`` attributes every width
    reading by heading, so yaw error does not merely steer badly, it can
    file a measurement under the wrong corridor.

    All heading error is modelled on the *reading*, never as a one-off seed of
    the estimator. The estimator takes yaw from the IMU on every update, so a
    seeded yaw offset would be overwritten on the first tick and measure
    nothing. That is also the physical truth: the BNO085's yaw zero is fixed at
    boot, so a chassis set down askew is wrong by that angle for the whole
    round rather than converging out of it.

    All values are magnitudes; the sign and bearing are drawn from the run's
    seeded RNG, so a scenario perturbs the same way every time it is run while
    different scenarios perturb differently.

    Attributes:
        start_pos_error_m: Distance between where the body is and where the
            estimator is told it is (random bearing). The localizer can work
            this off by matching scans; it is a search problem, not a fixed
            handicap.
        yaw_bias_rad: Constant offset between the IMU's yaw zero and the world
            frame -- the chassis set down askew, or the IMU zeroed askew.
            Never corrected, because nothing else observes absolute heading.
        imu_drift_rad_per_s: Yaw drift rate accumulated over elapsed time
            (random sign). This is the BNO085's quoted 0.5 deg/min figure.
        gyro_scale_error: Fractional error in how much rotation the gyro
            reports, e.g. ``0.005`` for 0.5%. Accumulates per *degree turned*
            rather than per second, which is why it is modelled separately from
            drift: a lap-driving robot turns 12 corners of 90 degrees in three
            laps, so it banks over 1080 degrees of deliberate rotation and the
            error scales with the course rather than the clock. A robot vacuum
            wanders and largely cancels this out; this one does not.
        imu_noise_rad: Per-reading Gaussian yaw noise.
    """

    start_pos_error_m: float = 0.0
    yaw_bias_rad: float = 0.0
    imu_drift_rad_per_s: float = 0.0
    gyro_scale_error: float = 0.0
    imu_noise_rad: float = 0.0

    @property
    def any_error(self) -> bool:
        """True if this configures any perturbation at all."""
        return bool(
            self.start_pos_error_m
            or self.yaw_bias_rad
            or self.imu_drift_rad_per_s
            or self.gyro_scale_error
            or self.imu_noise_rad,
        )


class ImuErrorModel:
    """Turns :class:`SensorErrors` into the yaw an IMU with those flaws would report.

    Bias, drift and scale-error signs are drawn once at construction and held
    fixed for the model's lifetime: a gyro bias is a constant, and a sign that
    wandered would average itself out and understate the damage.
    """

    def __init__(self, errors: SensorErrors, rng: np.random.Generator) -> None:
        self._errors = errors
        self._rng = rng
        self._drift_sign = float(rng.choice([-1.0, 1.0]))
        self._bias_sign = float(rng.choice([-1.0, 1.0]))
        self._scale_sign = float(rng.choice([-1.0, 1.0]))

    def yaw(self, true_yaw: float, elapsed_s: float, rotation_rad: float) -> float:
        """Heading as the IMU reports it: truth plus accumulated drift and noise.

        Args:
            true_yaw: Ground-truth chassis heading (radians).
            elapsed_s: Time elapsed since the run started, for ``imu_drift_rad_per_s``.
            rotation_rad: Signed, unwrapped rotation turned through so far, for
                ``gyro_scale_error``, which accumulates per degree turned rather
                than per second.
        """
        errors = self._errors
        yaw = (
            true_yaw
            + self._bias_sign * errors.yaw_bias_rad
            + self._drift_sign * errors.imu_drift_rad_per_s * elapsed_s
            + self._scale_sign * errors.gyro_scale_error * rotation_rad
        )
        if errors.imu_noise_rad > 0.0:
            yaw += float(self._rng.normal(0.0, errors.imu_noise_rad))
        return yaw
