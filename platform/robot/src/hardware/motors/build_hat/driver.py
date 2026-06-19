"""Build HAT motor driver implementation."""

from __future__ import annotations

import logging
import signal
from typing import Self, override

from buildhat import Motor

from src.hardware.exceptions import MotorConnectionError, MotorTimeoutError
from src.hardware.motors.base import (
    CalibrationData,
    Driver as MotorDriver,
)
from src.hardware.motors.config import Config
from src.logger import configure_json_logging

configure_json_logging()

CONNECTION_TIMEOUT = 5
"""Default timeout in seconds for motor connection attempts."""


class _TimeoutHandler:
    """Context manager for connection timeout handling."""

    def __init__(self, timeout_seconds: int):
        """
        Initialize timeout handler.

        Args:
            timeout_seconds (int): Number of seconds before timing out the connection attempt.
        """
        self.timeout_seconds = timeout_seconds
        self._original_handler = None

    def __enter__(self) -> Self @ _TimeoutHandler:
        """Set timeout alarm."""

        def _timeout_handler(_signum: int, _frame: signal.FrameType) -> None:
            """
            Signal handler for connection timeout. Raises MotorTimeoutError when the alarm signal is received.

            Args:
                _signum (int): The signal number (should be signal.SIGALRM).
                _frame (signal.FrameType): The current stack frame (not used).
            """
            msg = f"Motor connection timed out after {self.timeout_seconds} seconds"
            raise MotorTimeoutError(msg)

        # Set the signal handler for SIGALRM to our timeout handler and start the alarm
        self._original_handler = signal.signal(signal.SIGALRM, _timeout_handler)
        signal.alarm(self.timeout_seconds)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: object,
    ) -> None:
        """
        Cancel timeout alarm.

        Args:
            exc_type (type[BaseException] | None): The type of exception raised (if any) within the context block.
            exc_value (BaseException | None): The exception instance raised (if any) within the context block.
            traceback (object): The traceback object associated with the exception (if any) raised within the context block.
        """
        signal.alarm(0)
        if self._original_handler is not None:
            signal.signal(signal.SIGALRM, self._original_handler)


class Driver(MotorDriver):
    """Driver for Build HAT motor control."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._steering: Motor | None = None
        self._drive: Motor | None = None
        self._calibration: CalibrationData | None = None
        self.logger = logging.getLogger(__name__)

    @override
    def connect(self) -> None:
        """Connect to motors with timeout protection.

        Raises:
            MotorConnectionError: If motor connection fails or times out.
        """
        self.logger.info("Connecting to Build HAT motors")

        try:
            with _TimeoutHandler(CONNECTION_TIMEOUT):
                self._steering = Motor(self.config.steering.port)
                self._drive = Motor(self.config.drive.port)
        except MotorTimeoutError as e:
            raise MotorConnectionError(
                [self.config.steering.port, self.config.drive.port],
                "Connection timeout (hardware not responding)",
            ) from e
        except FileNotFoundError as e:
            raise MotorConnectionError(
                [self.config.steering.port, self.config.drive.port],
                "Motor not found (USB disconnected?)",
            ) from e
        except PermissionError as e:
            raise MotorConnectionError(
                [self.config.steering.port, self.config.drive.port],
                "Permission denied (not running as root?)",
            ) from e
        except Exception as e:
            raise MotorConnectionError(
                [self.config.steering.port, self.config.drive.port],
                f"Unexpected connection error: {type(e).__name__}",
            ) from e

        self.logger.info(
            "Connected",
            extra={"details": {"steering": self.config.steering.port, "drive": self.config.drive.port}},
        )

    @property
    def steering(self) -> Motor:
        """Get steering motor."""
        if self._steering is None:
            self.connect()
        return self._steering

    @property
    def drive(self) -> Motor:
        """Get drive motor."""
        if self._drive is None:
            self.connect()
        return self._drive

    @override
    def get_steering_position(self) -> float:
        """Get current steering position in degrees."""
        return self.steering.get_aposition()

    @override
    def get_drive_position(self) -> float:
        """Get current drive position in degrees."""
        return self.drive.get_aposition()

    @override
    def get_steering_speed(self) -> float:
        """Get current steering speed in degrees/s."""
        return self.steering.get_speed()

    @override
    def get_drive_speed(self) -> float:
        """Get current drive speed in degrees/s."""
        return self.drive.get_speed()

    def _clamp_speed(self, speed: int) -> int:
        """Clamp speed to configured limits."""
        speed = abs(speed)
        return max(min(speed, self.config.drive.max_speed), self.config.drive.min_speed)

    def _clamp_position(self, position: float) -> float:
        """Clamp steering position to configured limits."""
        return max(
            min(position, self.config.steering.right_limit_angle),
            self.config.steering.left_limit_angle,
        )

    @override
    def run_drive_forward(self, speed: int | None = None) -> None:
        """Run drive motor forward with optional speed limit."""
        s = self._clamp_speed(speed or self.config.drive.default_speed)
        self.logger.info("Starting drive forward", extra={"details": {"speed": s}})
        self.drive.start(s)

    @override
    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Run drive motor in reverse."""
        s = self._clamp_speed(speed or self.config.drive.default_speed)
        self.logger.info("Starting drive reverse", extra={"details": {"speed": -s}})
        self.drive.start(-s)

    @override
    def stop_drive(self) -> None:
        """Stop drive motor."""
        self.drive.stop()
        self.logger.info("Drive stopped")

    @override
    def move_steering_to(self, position: float, speed: int | None = None) -> None:
        """Move steering to absolute position in degrees."""
        s = self._clamp_speed(speed)
        self.logger.info("Moving steering", extra={"details": {"target_position": position, "speed": speed}})
        self.steering.run_to_position(position, speed=s)

    @override
    def center_steering(self) -> None:
        """Center steering wheels."""
        self.move_steering_to(self.config.steering.center_angle, speed=self.config.steering.centering_speed)

    @override
    def move_steering_to_right_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to right relative position in degrees from center position."""
        self.logger.info("Moving steering right", extra={"details": {"relative_position": position, "speed": speed}})
        position = self._clamp_position(position)
        target_position = (
            self.config.steering.center_angle + position
            if not self.config.steering.reversed
            else self.config.steering.center_angle - position
        )
        self.steering.run_to_position(target_position, speed=speed)

    @override
    def move_steering_to_left_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to left relative position in degrees from center position."""
        self.logger.info("Moving steering left", extra={"details": {"relative_position": position, "speed": speed}})
        position = self._clamp_position(position)
        target_position = (
            self.config.steering.center_angle - position
            if not self.config.steering.reversed
            else self.config.steering.center_angle + position
        )
        self.steering.run_to_position(target_position, speed=speed)
