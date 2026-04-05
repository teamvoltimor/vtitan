"""Abstract base classes for physical button interface."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum


class ButtonEvent(Enum):
    """Button event types."""

    PRESSED = "pressed"
    RELEASED = "released"
    SHORT_PRESS = "short_press"
    LONG_PRESS = "long_press"


@dataclass
class ButtonState:
    """Current button state data."""

    is_pressed: bool
    """Whether the button is currently pressed."""

    press_duration: float
    """Duration of current press in seconds."""

    last_event: ButtonEvent | None
    """Last detected button event."""


@dataclass
class Config:
    """Configuration for button driver."""

    gpio_pin: int
    """GPIO pin number for the button."""

    pull_up: bool
    """Whether to use internal pull-up resistor."""

    debounce_ms: int
    """Debounce delay in milliseconds."""

    long_press_threshold_sec: float
    """Duration threshold for long press detection in seconds."""


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
