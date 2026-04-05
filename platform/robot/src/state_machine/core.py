"""State machine core logic and state transition manager."""

import logging
from dataclasses import dataclass
from typing import Callable

from src.logger import configure_json_logging
from src.state_machine.types import RobotState, StateTransitionReason

logger = configure_json_logging()


@dataclass
class StateTransition:
    """State transition event."""

    from_state: RobotState
    """State transitioning from."""

    to_state: RobotState
    """State transitioning to."""

    reason: StateTransitionReason
    """Reason for transition."""


class StateMachine:
    """4-stage state machine for WRO competition robot.

    Manages state transitions and ensures competition rule compliance:
    - BOOT_CHECK: Verify all hardware before allowing start
    - READY: Wait for single button press
    - RACING: Ignore short presses, only accept 2-second hold for E-STOP
    - FINISHED: Display final results
    """

    def __init__(self):
        self._current_state: RobotState = RobotState.BOOT_CHECK
        self._transition_callbacks: list[Callable[[StateTransition], None]] = []
        self.logger: logging.Logger = logging.getLogger(__name__)

    @property
    def current_state(self) -> RobotState:
        """Get current state."""
        return self._current_state

    def register_transition_callback(self, callback: Callable[[StateTransition], None]) -> None:
        """Register a callback to be called on state transitions.

        Args:
            callback: Function to call with StateTransition object.
        """
        self._transition_callbacks.append(callback)

    def transition_to(self, new_state: RobotState, reason: StateTransitionReason) -> bool:
        """Transition to a new state.

        Args:
            new_state: Target state to transition to.
            reason: Reason for the transition.

        Returns:
            bool: True if transition was allowed, False if blocked.
        """
        # Validate transition
        if not self._is_valid_transition(self._current_state, new_state, reason):
            self.logger.warning(
                "Invalid state transition blocked",
                extra={
                    "details": {
                        "from": self._current_state.value,
                        "to": new_state.value,
                        "reason": reason.value,
                    }
                },
            )
            return False

        # Create transition object
        transition = StateTransition(from_state=self._current_state, to_state=new_state, reason=reason)

        # Log transition
        self.logger.info(
            "State transition",
            extra={
                "details": {
                    "from": self._current_state.value,
                    "to": new_state.value,
                    "reason": reason.value,
                }
            },
        )

        # Update state
        self._current_state = new_state

        # Notify callbacks
        for callback in self._transition_callbacks:
            try:
                callback(transition)
            except Exception as e:
                self.logger.error(f"Transition callback error: {e}")

        return True

    def _is_valid_transition(self, from_state: RobotState, to_state: RobotState, reason: StateTransitionReason) -> bool:
        """Validate if a state transition is allowed.

        Args:
            from_state: Current state.
            to_state: Target state.
            reason: Reason for transition.

        Returns:
            bool: True if transition is valid.
        """
        # Define valid transitions
        valid_transitions: dict[RobotState, dict[StateTransitionReason, RobotState]] = {
            RobotState.BOOT_CHECK: {
                StateTransitionReason.BOOT_COMPLETE: RobotState.READY,
                StateTransitionReason.BOOT_FAILED: RobotState.FINISHED,
            },
            RobotState.READY: {
                StateTransitionReason.BUTTON_PRESSED: RobotState.RACING,
                StateTransitionReason.SYSTEM_RESET: RobotState.BOOT_CHECK,
            },
            RobotState.RACING: {
                StateTransitionReason.LAPS_COMPLETED: RobotState.FINISHED,
                StateTransitionReason.EMERGENCY_STOP: RobotState.FINISHED,
                StateTransitionReason.SYSTEM_RESET: RobotState.BOOT_CHECK,
            },
            RobotState.FINISHED: {
                StateTransitionReason.SYSTEM_RESET: RobotState.BOOT_CHECK,
            },
        }

        # Check if transition exists in valid transitions
        if from_state not in valid_transitions:
            return False

        allowed_target = valid_transitions[from_state].get(reason)
        return allowed_target == to_state

    def can_start_race(self) -> bool:
        """Check if robot can start racing.

        Returns:
            bool: True if in READY state.
        """
        return self._current_state == RobotState.READY

    def is_racing(self) -> bool:
        """Check if robot is currently racing.

        Returns:
            bool: True if in RACING state.
        """
        return self._current_state == RobotState.RACING

    def is_finished(self) -> bool:
        """Check if race is finished.

        Returns:
            bool: True if in FINISHED state.
        """
        return self._current_state == RobotState.FINISHED

    def is_boot_checking(self) -> bool:
        """Check if system is performing boot checks.

        Returns:
            bool: True if in BOOT_CHECK state.
        """
        return self._current_state == RobotState.BOOT_CHECK
