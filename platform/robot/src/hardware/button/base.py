"""Abstract base classes for physical button interface."""

from abc import ABC, abstractmethod

from src.hardware.button.state import ButtonState


class Driver(ABC):
    """Abstract button driver interface."""

    @abstractmethod
    def connect(self) -> None:
        """Initialize GPIO and configure button pin."""

    @abstractmethod
    def get_state(self) -> ButtonState:
        """Get current button state.

        Returns:
            ButtonState: Current state including press duration and last event.
        """

    @abstractmethod
    def is_pressed(self) -> bool:
        """Check if button is currently pressed.

        Returns:
            bool: True if button is pressed, False otherwise.
        """

    @abstractmethod
    def wait_for_press(self, timeout: float | None = None) -> bool:
        """Wait for button press event.

        Args:
            timeout: Maximum time to wait in seconds. None for blocking wait.

        Returns:
            bool: True if button was pressed, False if timeout occurred.
        """

    @abstractmethod
    def wait_for_release(self, timeout: float | None = None) -> bool:
        """Wait for button release event.

        Args:
            timeout: Maximum time to wait in seconds. None for blocking wait.

        Returns:
            bool: True if button was released, False if timeout occurred.
        """

    @abstractmethod
    def close(self) -> None:
        """Clean up GPIO resources."""
