"""GPIO button driver with debouncing and long-press detection."""

import logging
import time
from dataclasses import dataclass
from typing import override

try:
    import gpiozero
    from gpiozero import Button
except ImportError:
    gpiozero = None  # type: ignore
    Button = None  # type: ignore

from src.env import EnvVar
from src.hardware.button.base import (
    ButtonEvent,
    ButtonState,
    Config as BaseConfig,
    Driver as BaseDriver,
)
from src.logger import configure_json_logging

logger = configure_json_logging()


BUTTON_GPIO_PIN = EnvVar[int](key="BUTTON_GPIO_PIN", default=17, cast=int)
"""GPIO pin number for the physical button. Default is GPIO17 (Pin 11 on Pi 5)."""

BUTTON_PULL_UP = EnvVar[bool](key="BUTTON_PULL_UP", default=True, cast=lambda x: x.lower() in ("true", "1", "yes"))
"""Whether to use internal pull-up resistor. Default is True."""

BUTTON_DEBOUNCE_MS = EnvVar[int](key="BUTTON_DEBOUNCE_MS", default=50, cast=int)
"""Debounce delay in milliseconds. Default is 50ms."""

BUTTON_LONG_PRESS_SEC = EnvVar[float](key="BUTTON_LONG_PRESS_SEC", default=2.0, cast=float)
"""Duration threshold for long press detection in seconds. Default is 2.0s."""


@dataclass
class Config(BaseConfig):
    """Configuration for GPIO button driver."""

    gpio_pin: int = BUTTON_GPIO_PIN.value
    pull_up: bool = BUTTON_PULL_UP.value
    debounce_ms: int = BUTTON_DEBOUNCE_MS.value
    long_press_threshold_sec: float = BUTTON_LONG_PRESS_SEC.value


class Driver(BaseDriver):
    """GPIO button driver with debouncing and long-press detection.

    Uses gpiozero library for GPIO access. Supports:
    - Debouncing to filter out noise
    - Short press and long press detection
    - Event-based and polling-based APIs
    """

    def __init__(self, config: Config | None = None):
        if gpiozero is None:
            raise ImportError("gpiozero library is required for button driver. Install with: pip install gpiozero")

        self.config: Config = config or Config()
        self._button: Button | None = None
        self._press_start_time: float | None = None
        self._last_event: ButtonEvent | None = None
        self._is_pressed: bool = False
        self.logger: logging.Logger = logging.getLogger(__name__)

    @override
    def connect(self) -> None:
        """Initialize GPIO and configure button pin."""
        self.logger.info(
            "Connecting to button",
            extra={
                "details": {
                    "gpio_pin": self.config.gpio_pin,
                    "pull_up": self.config.pull_up,
                    "debounce_ms": self.config.debounce_ms,
                },
            },
        )

        # Create button with debouncing
        self._button = Button(
            self.config.gpio_pin,
            pull_up=self.config.pull_up,
            bounce_time=self.config.debounce_ms / 1000.0,  # Convert to seconds
        )

        # Set up event callbacks
        self._button.when_pressed = self._on_pressed
        self._button.when_released = self._on_released

        self.logger.info("Button connected successfully")

    def _on_pressed(self) -> None:
        """Internal callback when button is pressed."""
        self._is_pressed = True
        self._press_start_time = time.time()
        self._last_event = ButtonEvent.PRESSED
        self.logger.debug("Button pressed")

    def _on_released(self) -> None:
        """Internal callback when button is released."""
        self._is_pressed = False

        # Determine if it was a short or long press
        if self._press_start_time is not None:
            press_duration = time.time() - self._press_start_time

            if press_duration >= self.config.long_press_threshold_sec:
                self._last_event = ButtonEvent.LONG_PRESS
                self.logger.info(
                    "Button long press detected", extra={"details": {"duration": round(press_duration, 2)}},
                )
            else:
                self._last_event = ButtonEvent.SHORT_PRESS
                self.logger.debug(
                    "Button short press detected", extra={"details": {"duration": round(press_duration, 2)}},
                )

            self._press_start_time = None
        else:
            self._last_event = ButtonEvent.RELEASED

    @override
    def get_state(self) -> ButtonState:
        """Get current button state."""
        if self._button is None:
            self.connect()

        # Calculate current press duration if button is pressed
        press_duration = 0.0
        if self._is_pressed and self._press_start_time is not None:
            press_duration = time.time() - self._press_start_time

        return ButtonState(is_pressed=self._is_pressed, press_duration=press_duration, last_event=self._last_event)

    @override
    def is_pressed(self) -> bool:
        """Check if button is currently pressed."""
        if self._button is None:
            self.connect()

        return self._button.is_pressed if self._button is not None else False

    @override
    def wait_for_press(self, timeout: float | None = None) -> bool:
        """Wait for button press event."""
        if self._button is None:
            self.connect()

        if self._button is None:
            return False

        return self._button.wait_for_press(timeout=timeout)

    @override
    def wait_for_release(self, timeout: float | None = None) -> bool:
        """Wait for button release event."""
        if self._button is None:
            self.connect()

        if self._button is None:
            return False

        return self._button.wait_for_release(timeout=timeout)

    @override
    def close(self) -> None:
        """Clean up GPIO resources."""
        if self._button is not None:
            self._button.close()
            self._button = None

        self.logger.info("Button connection closed")
