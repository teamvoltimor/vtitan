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
    """Front-wheel steering actuator (servo or geared steering motor)."""

    @abstractmethod
    def get_steering_position(self) -> float:
        """Get current steering position in degrees."""

    @abstractmethod
    def get_steering_speed(self) -> float:
        """Get current steering speed in degrees/s."""

    @abstractmethod
    def move_steering_to(self, position: float, speed: int = 20) -> None:
        """Move steering to absolute position."""

    @abstractmethod
    def center_steering(self) -> None:
        """Center steering wheels."""

    @abstractmethod
    def move_steering_to_right_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to right relative position in degrees from center position."""

    @abstractmethod
    def move_steering_to_left_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to left relative position in degrees from center position."""


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
