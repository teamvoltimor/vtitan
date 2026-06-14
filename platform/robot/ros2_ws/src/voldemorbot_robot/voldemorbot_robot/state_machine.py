"""State machine definitions for WRO 2026 competition."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class RobotState(StrEnum):
    """Competition states for the robot state machine."""

    BOOT_CHECK = "BOOT_CHECK"
    READY = "READY"
    RACING = "RACING"
    FINISHED = "FINISHED"


class StateTransitionReason(StrEnum):
    """Reasons for state machine transitions."""

    BOOT_COMPLETE = "BOOT_COMPLETE"
    BUTTON_PRESSED = "BUTTON_PRESSED"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    LAPS_COMPLETED = "LAPS_COMPLETED"


@dataclass
class SensorStatus:
    """Status of a single hardware sensor."""

    name: str
    is_ready: bool
    error_message: str | None


@dataclass
class SystemStatus:
    """Aggregate status of all hardware subsystems."""

    imu_status: SensorStatus
    lidar_status: SensorStatus
    hailo_status: SensorStatus
    drive_status: SensorStatus
    network_status: str
    all_ready: bool


@dataclass
class Transition:
    """Represents a state transition between two robot states."""

    from_state: RobotState
    to_state: RobotState
    reason: StateTransitionReason


@dataclass
class RaceMetrics:
    """Race performance metrics."""

    laps_completed: int
    total_race_time: float
    current_velocity: float
    current_steering: float
    gyro_yaw: float
    current_corridor: str


class StateMachine:
    """Simple state machine for WRO competition flow."""

    def __init__(self) -> None:
        self._current_state = RobotState.BOOT_CHECK
        self._transition_callbacks: list[object] = []

    @property
    def current_state(self) -> RobotState:
        """Get the current robot state."""
        return self._current_state

    def register_transition_callback(self, callback: object) -> None:
        """Register a callback invoked on every state transition."""
        self._transition_callbacks.append(callback)

    def transition_to(self, new_state: RobotState, reason: StateTransitionReason) -> None:
        """Transition to a new state and notify callbacks."""
        from_state = self._current_state
        self._current_state = new_state
        tx = Transition(from_state=from_state, to_state=new_state, reason=reason)
        for cb in self._transition_callbacks:
            cb(tx)
