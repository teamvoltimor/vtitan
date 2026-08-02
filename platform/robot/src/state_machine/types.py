"""4-stage state machine for WRO competition robot control.

States:
    BOOT_CHECK: Hardware verification and initialization
    READY: Waiting for button press to start
    RACING: Autonomous racing mode
    FINISHED: Race completed or emergency stop
"""

from dataclasses import dataclass
from enum import Enum, StrEnum

# Re-export RobotState/ScenarioType from shared module (single source of truth). ScenarioType
# is the existing open-vs-obstacles concept (already used by the navigator's scenario
# metadata) -- the challenge-mode jumper reuses it rather than introducing a duplicate enum.
from shared.config.enums import RobotState, ScenarioType, Section

__all__ = [
    "LidarMetrics",
    "PathStatus",
    "RaceMetrics",
    "RobotState",
    "ScenarioType",
    "SensorStatus",
    "StateTransitionReason",
    "SystemStatus",
    "VisionMetrics",
]


class PathStatus(StrEnum):
    """Current state of the navigation path."""

    CLEAR = "CLEAR"
    """No obstacles detected, clear path."""

    BLOCKED = "BLOCKED"
    """Path is blocked by obstacles."""

    NARROW = "NARROW"
    """Path is clear but narrow."""


@dataclass
class SensorStatus:
    """Status of individual sensor/component."""

    name: str
    """Component name (e.g., 'IMU', 'LiDAR', 'Hailo', 'Drive')."""

    is_ready: bool
    """Whether the component is responsive and ready."""

    error_message: str | None = None
    """Error message if component is not ready."""


@dataclass
class SystemStatus:
    """Overall system status for BOOT_CHECK state."""

    imu_status: SensorStatus
    """IMU sensor status."""

    lidar_status: SensorStatus
    """LiDAR sensor status."""

    hailo_status: SensorStatus
    """Hailo NPU status including model loading."""

    drive_status: SensorStatus
    """Ackermann drive system status."""

    challenge_mode_status: SensorStatus
    """Challenge-mode jumper (GPIO23) status -- not ready while the reading is unstable."""

    network_status: str
    """Network status: IP address or 'OFFLINE'."""

    all_ready: bool
    """True if all required components are ready."""

    challenge_mode: ScenarioType | None = None
    """Detected challenge mode once the jumper reading has stabilized, else None."""


@dataclass
class RaceStatus:
    """Instantaneous race-state snapshot for telemetry display."""

    laps_completed: int
    """Number of laps completed (0-3)."""

    total_race_time: float
    """Total elapsed race time in seconds."""

    current_velocity: float
    """Current commanded velocity in m/s."""

    current_steering: float
    """Current commanded steering angle in degrees."""

    gyro_yaw: float
    """Current gyroscope yaw in degrees."""

    current_corridor: Section | None = None
    """Active track corridor, or None if unknown."""


@dataclass
class VisionMetrics:
    """Hailo AI vision metrics."""

    npu_fps: float
    """NPU inference rate in frames per second."""

    target_confidence: float | None
    """Confidence score of active target (0.0-1.0)."""

    bbox_x: int | None
    """Bounding box X coordinate."""

    bbox_y: int | None
    """Bounding box Y coordinate."""

    bbox_width: int | None
    """Bounding box width."""

    bbox_height: int | None
    """Bounding box height."""

    estimated_distance: float | None
    """Estimated distance to target in meters."""


@dataclass
class LidarMetrics:
    """LiDAR spatial awareness metrics."""

    front_clearance_cm: float
    """Front clearance distance in centimeters."""

    left_clearance_cm: float
    """Left clearance distance in centimeters."""

    right_clearance_cm: float
    """Right clearance distance in centimeters."""

    path_status: PathStatus
    """Current path status."""


class StateTransitionReason(Enum):
    """Reasons for state transitions."""

    # BOOT_CHECK → READY
    BOOT_COMPLETE = "boot_complete"
    """All hardware checks passed."""

    BOOT_FAILED = "boot_failed"
    """Hardware verification failed."""

    # READY → RACING
    BUTTON_PRESSED = "button_pressed"
    """Physical button was pressed to start race."""

    # RACING → FINISHED
    LAPS_COMPLETED = "laps_completed"
    """Successfully completed 3 laps."""

    EMERGENCY_STOP = "emergency_stop"
    """Emergency stop triggered (2-second button hold)."""

    # Any → BOOT_CHECK
    SYSTEM_RESET = "system_reset"
    """System reset requested."""
