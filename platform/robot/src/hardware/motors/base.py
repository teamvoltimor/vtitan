"""Abstract base classes and value objects for motor drivers.

Interface segregation (ISP) so each actuator implements only what it can do:

- ``SteeringDriver``  — front-wheel steering (a 180-deg or 360-deg servo, or a
  geared steering motor). Knows nothing about propulsion.
- ``DriveDriver``     — rear-wheel propulsion (an open-loop DC motor or a smart
  motor). Knows nothing about steering.
- ``EncodedDriveDriver`` — a ``DriveDriver`` that additionally exposes wheel
  odometry and closed-loop RPM control (a DC motor *with* an encoder).

The legacy combined ``Driver`` (steering + drive in one object, as the LEGO
Build HAT provides) is retained as an alias so existing adapters keep working.
A drive-only motor must not be forced to stub out steering, and an
encoder-equipped motor has a home for its counts — that is the whole point of
the split, and what lets the navigation/ROS2 stack swap actuators unchanged.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

DEFAULT_STEERING_SPEED = 20
"""Default steering move speed (deg/s) used when a caller does not give one."""

STEERING_CENTER_DEG = 0.0
"""Absolute steering angle (deg) for wheels-straight, by convention."""


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

    @abstractmethod
    def get_drive_position(self) -> float:
        """Get current drive position in degrees."""

    @abstractmethod
    def get_drive_speed(self) -> float:
        """Get current drive speed in degrees/s."""

    @abstractmethod
    def run_drive_forward(self, speed: int | None = None) -> None:
        """Run drive motor forward."""

    @abstractmethod
    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Run drive motor in reverse."""

    @abstractmethod
    def stop_drive(self) -> None:
        """Stop drive motor."""


class EncodedDriveDriver(DriveDriver):
    """A drive motor that also exposes encoder odometry and closed-loop speed."""

    @abstractmethod
    def reset_drive_encoder(self) -> None:
        """Zero the encoder counts."""

    @abstractmethod
    def get_drive_counts(self) -> int:
        """Raw quadrature counts since the last reset."""

    @abstractmethod
    def get_drive_rpm(self) -> float:
        """Output-shaft speed (signed RPM)."""

    @abstractmethod
    def get_drive_odometry(self) -> DriveOdometry:
        """Full odometry sample."""

    @abstractmethod
    def run_drive_at_rpm(self, rpm: float) -> None:
        """Closed-loop: hold the given output-shaft RPM."""


class Driver(SteeringDriver, DriveDriver, ABC):
    """Legacy combined steering + drive interface (e.g. LEGO Build HAT).

    Retained so existing single-object adapters keep working. New single-purpose
    actuators should implement ``SteeringDriver``, ``DriveDriver`` or
    ``EncodedDriveDriver`` directly instead of this combined contract.
    """
