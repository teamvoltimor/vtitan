"""GPIO button driver with debouncing and long-press detection."""

import logging
import threading
import time
from typing import override

from gpiozero import Button
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict

from src.hardware.button.base import Driver as ABC_Driver
from src.hardware.button.config import Config as ButtonConfig
from src.hardware.button.event import ButtonEvent
from src.hardware.button.state import ButtonState
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging

configure_json_logging()


class Config(HardwareBaseSettings):
    """Configuration for GPIO button driver."""

    model_config = SettingsConfigDict(
        env_prefix="",
        # "__" so nested leaves with underscores parse, e.g.
        # BUTTON__DEBOUNCE_MS -> button.debounce_ms.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "button" / "gpio.toml",
    )

    # No prefix on this class, so gpio_pin needs an explicit alias to reach
    # BUTTON_GPIO_PIN -- it would otherwise only match a bare GPIO_PIN var.
    # Accepts both the SHOUT_CASE env-var spelling and the lowercase TOML
    # key (button_gpio_pin).
    gpio_pin: int = Field(validation_alias=AliasChoices("BUTTON_GPIO_PIN", "button_gpio_pin"))
    """GPIO pin number for the button."""

    # ButtonConfig has no defaults for pull_up/debounce_ms/long_press_threshold_sec
    # -- this factory only succeeds when the nested BUTTON__* env vars are
    # set; mypy can't see that env resolution, hence the ignore.
    button: ButtonConfig = Field(default_factory=lambda: ButtonConfig())  # type: ignore[call-arg]


class Driver(ABC_Driver):
    """GPIO button driver with debouncing and long-press detection.

    Uses gpiozero library for GPIO access. Supports:
    - Debouncing to filter out noise
    - Short press and long press detection
    - Event-based and polling-based APIs
    """

    def __init__(self, config: Config | None = None) -> None:
        # gpio_pin is required with no default -- resolved from an env var
        # when config isn't passed explicitly; mypy can't see that.
        self.config: Config = config or Config()  # type: ignore[call-arg]
        self._button: Button | None = None
        self._press_start_time: float | None = None
        self._last_event: ButtonEvent | None = None
        self._is_pressed: bool = False
        self._lock: threading.Lock = threading.Lock()
        self.logger: logging.Logger = logging.getLogger(__name__)

    @override
    def connect(self) -> None:
        """Initialize GPIO and configure button pin."""
        self._lock.acquire()
        self.logger.info(
            "Connecting to button",
            extra={
                "details": {
                    "gpio_pin": self.config.gpio_pin,
                    "pull_up": self.config.button.pull_up,
                    "debounce_ms": self.config.button.debounce_ms,
                },
            },
        )

        # Create button with debouncing
        self._button = Button(
            self.config.gpio_pin,
            pull_up=self.config.button.pull_up,
            bounce_time=self.config.button.debounce_ms / 1000.0,  # Convert to seconds
        )

        # Set up event callbacks
        self._button.when_pressed = self._on_pressed
        self._button.when_released = self._on_released

        self.logger.info("Button connected successfully")
        self._lock.release()

    def _on_pressed(self) -> None:
        """Internal callback when button is pressed."""
        with self._lock:
            self._is_pressed = True
            self._press_start_time = time.time()
            self._last_event = ButtonEvent.PRESSED
        self.logger.debug("Button pressed")

    def _on_released(self) -> None:
        """Internal callback when button is released."""
        with self._lock:
            self._is_pressed = False

            # Determine if it was a short or long press
            if self._press_start_time is None:
                self._last_event = ButtonEvent.RELEASED
            else:
                press_duration = time.time() - self._press_start_time

                # Log the press duration and type of press
                if press_duration >= self.config.button.long_press_threshold_sec:
                    self._last_event = ButtonEvent.LONG_PRESS
                    self.logger.info(
                        "Button long press detected",
                        extra={"details": {"duration": round(press_duration, 2)}},
                    )
                else:
                    self._last_event = ButtonEvent.SHORT_PRESS
                    self.logger.debug(
                        "Button short press detected",
                        extra={"details": {"duration": round(press_duration, 2)}},
                    )

                self._press_start_time = None

    @override
    def get_state(self) -> ButtonState:
        """Get current button state.

        Reading last_event consumes it -- it reads as None again until the next new event, so a
        caller polling this in a loop (e.g. button_node) sees each event exactly once instead of
        republishing the same stale event on every poll.
        """
        if self._button is None:
            self.connect()

        with self._lock:
            # Calculate current press duration if button is pressed
            press_duration = 0.0
            if self._is_pressed and self._press_start_time is not None:
                press_duration = time.time() - self._press_start_time

            last_event = self._last_event
            self._last_event = None

        return ButtonState(is_pressed=self._is_pressed, press_duration=press_duration, last_event=last_event)

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

        return bool(self._button.wait_for_press(timeout=timeout))

    @override
    def wait_for_release(self, timeout: float | None = None) -> bool:
        """Wait for button release event."""
        if self._button is None:
            self.connect()

        if self._button is None:
            return False

        return bool(self._button.wait_for_release(timeout=timeout))

    @override
    def close(self) -> None:
        """Clean up GPIO resources."""
        if self._button is not None:
            self._button.close()
            self._button = None

        self.logger.info("Button connection closed")
