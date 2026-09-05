"""Abstract base classes and value objects for motor drivers.

Interface segregation (ISP) so each actuator implements only what it can do:

- ``SteeringDriver``  — front-wheel steering (a 180-deg or 360-deg servo, or a
  geared steering motor). Knows nothing about propulsion.
- ``DriveDriver``     — rear-wheel propulsion (an open-loop DC motor or a smart
  motor, driven through whatever H-bridge is wired up). Knows nothing about
  steering, and nothing about encoder feedback -- a quadrature encoder is a
  separate physical part clipped to the motor shaft, not something the
  H-bridge chip itself provides.
- ``EncoderSensor``   — a quadrature encoder's counts/RPM/odometry, independent
  of which ``DriveDriver`` (if any) is turning the shaft it is reading.
- ``ClosedLoopDrive`` — composes a ``DriveDriver`` + ``EncoderSensor`` + a PID
  to expose closed-loop RPM control, without either half needing to know about
  the other.

The legacy combined ``Driver`` (steering + drive in one object, as the LEGO
Build HAT provides) is retained as an alias so existing adapters keep working
-- the Build HAT's LEGO motors report their own position/speed through the
Build HAT protocol itself, so they need no external ``EncoderSensor`` and stay
on ``DriveDriver`` directly. A drive-only motor must not be forced to stub out
steering, and an encoder-equipped motor has a home for its counts — that is
the whole point of the split, and what lets the navigation/ROS2 stack swap
actuators unchanged.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from src.hardware.motors.encoder.control import PIDController

STEERING_CENTER_DEG = 0.0
"""Absolute steering angle (deg) for wheels-straight, by convention."""

DEFAULT_STEERING_SPEED = 20
"""Default steering move speed (deg/s) used when a caller does not give one.

Mirrors motors.toml:steering.centering_speed. Kept as a plain literal here
rather than reading the pydantic-settings ``Config`` at import time: this is
the abstract driver interface, and every servo.centering_speed override
already reaches the concrete drivers (servo/driver.py, build_hat/driver.py)
through their own config, not through this default -- it only matters for a
caller that omits ``speed`` entirely."""


@dataclass(frozen=True)
class CalibrationData:
    """Steering calibration captured at connect time.

    Attributes:
        center_offset_deg: Encoder reading (deg) for wheels-straight.
        left_limit_deg: Max steering travel left of centre (deg).
        right_limit_deg: Max steering travel right of centre (deg).
    """

    center_offset_deg: float = 0.0
    left_limit_deg: float = 0.0
    right_limit_deg: float = 0.0


@dataclass(frozen=True)
class DriveOdometry:
    """Wheel odometry sample from an encoder-equipped drive.

    Attributes:
        counts: Raw quadrature counts since the last reset.
        revolutions: Output-shaft revolutions since the last reset.
        rpm: Output-shaft speed (signed, RPM).
        distance_m: Linear distance travelled at the wheel (metres).
    """

    counts: int
    revolutions: float
    rpm: float
    distance_m: float


class SteeringDriver(ABC):
    """Front-wheel steering actuator (servo or geared steering motor).

    Subclasses implement only the two primitives :meth:`move_steering_to` and
    :meth:`get_steering_position`. Centring, the centre-relative helpers,
    open-loop speed read-back and the connection lifecycle have concrete
    defaults here, so a feedback-less actuator (e.g. an RC servo) need not
    restate them; richer backends (e.g. the Build HAT) override as needed.
    """

    @property
    def center_position(self) -> float:
        """Absolute steering angle (deg) that corresponds to wheels-straight."""
        return STEERING_CENTER_DEG

    def connect(self) -> None:
        """Open the steering hardware. No-op unless a backend needs it."""

    def disconnect(self) -> None:
        """Release the steering hardware. No-op unless a backend needs it."""

    @abstractmethod
    def get_steering_position(self) -> float:
        """Get current steering position in degrees."""

    @abstractmethod
    def move_steering_to(self, position: float, speed: int = DEFAULT_STEERING_SPEED) -> None:
        """Move steering to absolute position."""

    def get_steering_speed(self) -> float:
        """Get current steering speed in degrees/s (0.0 when there is no feedback)."""
        return 0.0

    def center_steering(self) -> None:
        """Center steering wheels."""
        self.move_steering_to(self.center_position)

    def move_steering_to_right_from_center(self, position: float, speed: int = DEFAULT_STEERING_SPEED) -> None:
        """Move steering right by ``position`` degrees from the center position."""
        self.move_steering_to(self.center_position + position, speed)

    def move_steering_to_left_from_center(self, position: float, speed: int = DEFAULT_STEERING_SPEED) -> None:
        """Move steering left by ``position`` degrees from the center position."""
        self.move_steering_to(self.center_position - position, speed)


class DriveDriver(ABC):
    """Rear-wheel propulsion actuator (open-loop DC motor or smart motor)."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to the motor hardware."""

    def disconnect(self) -> None:
        """Release the drive hardware. No-op unless a backend needs it."""

    @abstractmethod
    def get_drive_position(self) -> float:
        """Get current drive position in degrees."""

    @abstractmethod
    def get_drive_speed(self) -> float:
        """Get current drive speed in degrees/s."""

    @abstractmethod
    def run_drive_forward(self, speed: float | None = None) -> None:
        """Run drive motor forward at ``speed`` percent duty (default backend-chosen).

        Accepts a float as well as an int so a closed loop (``ClosedLoopDrive``)
        can command sub-percent duty resolution instead of being rounded to
        whole percent -- the PWM peripheral itself resolves far finer than 1%.
        """

    @abstractmethod
    def run_drive_reverse(self, speed: float | None = None) -> None:
        """Run drive motor in reverse at ``speed`` percent duty (default backend-chosen)."""

    @abstractmethod
    def stop_drive(self) -> None:
        """Stop drive motor."""


class EncoderSensor(ABC):
    """A quadrature encoder's counts/RPM/odometry, independent of any drive motor.

    Physically a separate part (clipped to the motor shaft) from whatever
    H-bridge is turning it, so this makes no assumption about which
    ``DriveDriver`` (if any) is driving the shaft it reads.
    """

    @abstractmethod
    def connect(self) -> None:
        """Open the encoder GPIO lines."""

    def disconnect(self) -> None:
        """Release the encoder GPIO lines. No-op unless a backend needs it."""

    @abstractmethod
    def reset(self) -> None:
        """Zero the encoder counts."""

    @abstractmethod
    def get_counts(self) -> int:
        """Raw quadrature counts since the last reset."""

    @abstractmethod
    def get_rpm(self) -> float:
        """Sample and return output-shaft speed (signed RPM).

        MUTATES internal state -- consumes the counts accrued since the
        previous call to compute the interval's rate. Call this from exactly
        one place at a fixed rate (a closed loop); every other consumer must
        read ``get_last_rpm()`` instead, or two callers sampling at different
        rates will each steal part of the other's window and both under-read
        the true speed.
        """

    @abstractmethod
    def get_last_rpm(self) -> float:
        """The most recent value ``get_rpm()`` computed, without resampling."""

    @abstractmethod
    def get_odometry(self) -> DriveOdometry:
        """Full odometry sample, built from the cached (not resampled) RPM."""


class ClosedLoopDrive:
    """Composes a ``DriveDriver`` + ``EncoderSensor`` + a PID into closed-loop RPM control.

    Neither the drive H-bridge nor the encoder needs to know about the other,
    or about the PID -- this is the one place that reads the encoder, runs the
    PID, and commands the drive, so a caller (the ROS2 node) can hold a single
    object regardless of which H-bridge/encoder pair is behind it.
    """

    def __init__(self, drive: DriveDriver, encoder: EncoderSensor, pid: PIDController) -> None:
        self._drive = drive
        self._encoder = encoder
        self._pid = pid

    @property
    def drive(self) -> DriveDriver:
        """The underlying H-bridge drive."""
        return self._drive

    @property
    def encoder(self) -> EncoderSensor:
        """The underlying encoder."""
        return self._encoder

    def connect(self) -> None:
        """Connect both the drive and the encoder."""
        self._drive.connect()
        self._encoder.connect()

    def disconnect(self) -> None:
        """Disconnect both the drive and the encoder."""
        self._drive.disconnect()
        self._encoder.disconnect()

    def run_drive_at_rpm(self, rpm: float, dt: float) -> None:
        """One closed-loop step: PID the drive's duty toward ``rpm`` from encoder feedback.

        Duty is passed through as a float percent (not rounded to an int),
        so the PID's fine-grained correction is not quantised away by the
        open-loop percent-based ``run_drive_forward``/``run_drive_reverse``
        contract.
        """
        measured = self._encoder.get_rpm()
        duty = self._pid.update(rpm, measured, dt=dt)
        logger.debug(
            "PID step: target_rpm=%.1f measured_rpm=%.1f duty=%.3f", rpm, measured, duty
        )
        if duty >= 0:
            self._drive.run_drive_forward(duty * 100.0)
        else:
            self._drive.run_drive_reverse(-duty * 100.0)

    def stop_drive(self) -> None:
        """Stop the drive and clear the PID state."""
        self._pid.reset()
        self._drive.stop_drive()

    def reset_drive_encoder(self) -> None:
        """Zero the encoder counts."""
        self._encoder.reset()

    def get_drive_counts(self) -> int:
        """Raw quadrature counts since the last reset."""
        return self._encoder.get_counts()

    def get_drive_rpm(self) -> float:
        """Sample and return output-shaft speed (signed RPM). See ``EncoderSensor.get_rpm``."""
        return self._encoder.get_rpm()

    def get_drive_odometry(self) -> DriveOdometry:
        """Full odometry sample."""
        return self._encoder.get_odometry()

    def get_drive_position(self) -> float:
        """Output-shaft angle in degrees, from the encoder's cached odometry."""
        return self._encoder.get_odometry().revolutions * 360.0

    def get_drive_speed(self) -> float:
        """Wheel speed in degrees/s, from the encoder's last SAMPLED (not resampled) estimate.

        Deliberately reads ``get_last_rpm()`` rather than ``get_rpm()`` -- this
        is polled by the feedback publisher at a different rate than the
        control loop calls ``run_drive_at_rpm``, and resampling here as well
        would steal part of each window from the control loop's own sample
        (see ``EncoderSensor.get_rpm``'s docstring).
        """
        return self._encoder.get_last_rpm() / 60.0 * 360.0


class Driver(SteeringDriver, DriveDriver, ABC):
    """Legacy combined steering + drive interface (e.g. LEGO Build HAT).

    Retained so existing single-object adapters keep working. New single-purpose
    actuators should implement ``SteeringDriver``, ``DriveDriver`` or
    ``DriveDriver``/``EncoderSensor``/``ClosedLoopDrive`` directly instead of this combined contract.
    """
