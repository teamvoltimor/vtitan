"""4-stage state machine for WRO competition robot control.

States:
    BOOT_CHECK: Hardware verification and initialization
    READY: Waiting for button press to start
    RACING: Autonomous racing mode
    FINISHED: Race completed or emergency stop
"""

from dataclasses import dataclass
from enum import StrEnum

# Re-export RobotState/ScenarioType from shared module (single source of truth). ScenarioType
# is the existing open-vs-obstacles concept (already used by the navigator's scenario
# metadata) -- the challenge-mode jumper reuses it rather than introducing a duplicate enum.
from shared.domain.enums import RobotState, ScenarioType

__all__ = [
    "RobotState",
    "ScenarioType",
    "SensorStatus",
    "StateTransitionReason",
    "SystemStatus",
]


@dataclass(slots=True)
class SensorStatus:
    """Status of individual sensor/component."""

    name: str
    """Component name (e.g., 'IMU', 'LiDAR', 'Hailo', 'Drive')."""

    is_ready: bool
    """Whether the component is responsive and ready."""

    error_message: str | None = None
    """Error message if component is not ready."""


@dataclass(slots=True)
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


@dataclass(slots=True)
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

    current_corridor: str | None = None
    """Active track corridor (Section name string from track_navigator_node), or None if unknown."""


class StateTransitionReason(StrEnum):
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
