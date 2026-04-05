"""Build HAT motor driver implementation."""

import logging
from dataclasses import dataclass
from pathlib import Path

from buildhat import Motor

from src.env import EnvVar
from src.hardware.motors.base import CalibrationData
from src.hardware.motors.base import Config as BaseConfig
from src.hardware.motors.base import Driver as MotorDriver
from src.logger import configure_json_logging

configure_json_logging()

STEERING_PORT = EnvVar[str](key="MOTOR_STEERING_PORT", default="A")
DRIVE_PORT = EnvVar[str](key="MOTOR_DRIVE_PORT", default="B")
DEFAULT_SPEED = EnvVar[int](key="MOTOR_DEFAULT_SPEED", default=15, cast=int)
TEST_DURATION = EnvVar[float](key="MOTOR_TEST_DURATION", default=1.5, cast=float)


@dataclass
class Config(BaseConfig):
    """Build HAT motor configuration."""

    steering_port: str = STEERING_PORT.value
    drive_port: str = DRIVE_PORT.value
    default_speed: int = DEFAULT_SPEED.value
    test_duration: float = TEST_DURATION.value


class Driver(MotorDriver):
    """Driver for Build HAT motor control."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._steering: Motor | None = None
        self._drive: Motor | None = None
        self._calibration: CalibrationData | None = None
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to motors."""
        self.logger.info("Connecting to Build HAT motors")
        self._steering = Motor(self.config.steering_port)
        self._drive = Motor(self.config.drive_port)
        self.logger.info(
            "Connected",
            extra={"details": {"steering": self.config.steering_port, "drive": self.config.drive_port}},
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

    def get_steering_position(self) -> float:
        """Get current steering position in degrees."""
        return self.steering.get_aposition()

    def get_drive_position(self) -> float:
        """Get current drive position in degrees."""
        return self.drive.get_aposition()

    def get_steering_speed(self) -> float:
        """Get current steering speed in degrees/s."""
        return self.steering.get_speed()

    def get_drive_speed(self) -> float:
        """Get current drive speed in degrees/s."""
        return self.drive.get_speed()

    def run_drive_forward(self, speed: int | None = None) -> None:
        """Run drive motor forward."""
        s = speed or self.config.default_speed
        self.logger.info("Starting drive forward", extra={"details": {"speed": s}})
        self.drive.start(s)

    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Run drive motor in reverse."""
        s = speed or self.config.default_speed
        self.logger.info("Starting drive reverse", extra={"details": {"speed": -s}})
        self.drive.start(-s)

    def stop_drive(self) -> None:
        """Stop drive motor."""
        self.drive.stop()
        self.logger.info("Drive stopped")

    def move_steering_to(self, position: float, speed: int = 20) -> None:
        """Move steering to absolute position."""
        self.logger.info("Moving steering", extra={"details": {"target_position": position, "speed": speed}})
        self.steering.run_to_position(position, speed=speed)

    def center_steering(self) -> None:
        """Center steering wheels."""
        self.move_steering_to(0.0)

    def load_calibration(self, calibration_file: Path | None = None) -> CalibrationData:
        """Load calibration from file."""
        if calibration_file is None:
            calibration_file = Path(__file__).parent.parent.parent / "config" / "calibration.json"

        if not calibration_file.exists():
            self.logger.warning("Calibration file not found", extra={"details": {"file": str(calibration_file)}})
            return CalibrationData(left_limit=-45.0, right_limit=45.0)

        import json

        with open(calibration_file) as f:
            data = json.load(f)

        self._calibration = CalibrationData(
            left_limit=data["steering"]["left_limit"],
            right_limit=data["steering"]["right_limit"],
            center=data["steering"].get("center", 0.0),
        )

        self.logger.info("Calibration loaded", extra={"details": {"calibration": self._calibration.__dict__}})
        return self._calibration

    def save_calibration(self, left_limit: float, right_limit: float, calibration_file: Path | None = None) -> None:
        """Save calibration to file."""
        if calibration_file is None:
            calibration_file = Path(__file__).parent.parent.parent / "config" / "calibration.json"

        calibration_file.parent.mkdir(parents=True, exist_ok=True)

        data = {"steering": {"left_limit": left_limit, "right_limit": right_limit, "center": 0.0}}

        import json

        with open(calibration_file, "w") as f:
            json.dump(data, f, indent=2)

        self._calibration = CalibrationData(left_limit=left_limit, right_limit=right_limit)
        self.logger.info("Calibration saved", extra={"details": {"file": str(calibration_file)}})
