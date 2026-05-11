"""GPIO button driver with debouncing and long-press detection via MCP2221A USB bridge."""

import logging
import os
import threading
import time
from typing import override

import board
import digitalio

from src.hardware.button.base import (
    ButtonEvent,
    ButtonState,
    Driver as BaseDriver,
)
from src.hardware.button.mcp2221.config import Config
from src.logger import configure_json_logging

os.environ.setdefault("BLINKA_MCP2221", "1")

configure_json_logging()


class Driver(BaseDriver):
    """GPIO button driver with debouncing and long-press detection.

    Uses MCP2221A USB HID bridge for GPIO access. Supports:
    - Debouncing to filter out noise
    - Short press and long press detection
    - Event-based and polling-based APIs
    """

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        self._pin: digitalio.DigitalInOut | None = None
        self._press_start_time: float | None = None
        self._last_event: ButtonEvent | None = None
        self._is_pressed: bool = False
        self._last_reading_time: float = 0.0
        self._last_reading_value: bool = False
        self._polling_thread: threading.Thread | None = None
        self._stop_polling: threading.Event = threading.Event()
        self._state_lock: threading.Lock = threading.Lock()
        self.logger: logging.Logger = logging.getLogger(__name__)

        self._pin_map = {
            0: board.GP0,
            1: board.GP1,
            2: board.GP2,
            3: board.GP3,
        }

    def _configure_pin(self) -> None:
        """Configure GPIO pin as input."""
        pin_obj = digitalio.DigitalInOut(self._pin_map[self._gpio_pin])
        pin_obj.direction = digitalio.Direction.INPUT
        pin_obj.pull = digitalio.Pull.UP if self._pull_up else digitalio.Pull.DOWN
        self._pin = pin_obj
        self.logger.debug("Configured GP%d as input", self._gpio_pin)

    @override
    def connect(self) -> None:
        """Initialize GPIO and configure button pin."""
        self.logger.info(
            "Connecting to MCP2221 button",
            extra={
                "details": {
                    "gpio_pin": self._gpio_pin,
                    "debounce_ms": self.config.button.debounce_ms,
                    "pull_up": self._pull_up,
                },
            },
        )

        self._configure_pin()

        self._stop_polling.clear()
        self._polling_thread = threading.Thread(target=self._poll_gpio, daemon=True)
        self._polling_thread.start()

        self.logger.info("MCP2221 button connected successfully")

    def _read_gpio(self) -> bool:
        """Read GPIO pin state."""
        if self._pin is None:
            return False

        try:
            return self._pin.value
        except Exception:
            self.logger.exception("Error reading GPIO")
            return False

    def _poll_gpio(self) -> None:
        """Polling loop for GPIO state changes."""
        while not self._stop_polling.is_set():
            try:
                raw_state = self._read_gpio()
                current_time = time.time()

                with self._state_lock:
                    last_reading_time = self._last_reading_time
                    last_reading_value = self._last_reading_value
                    time_since_last_reading = current_time - last_reading_time

                    if raw_state != last_reading_value and time_since_last_reading >= self._debounce_sec:
                        if raw_state:
                            self._on_pressed()
                        else:
                            self._on_released()

                        self._last_reading_time = current_time
                        self._last_reading_value = raw_state

            except Exception:
                self.logger.exception("Error in polling loop")

            time.sleep(0.001)

    def _on_pressed(self) -> None:
        """Internal callback when button is pressed."""
        with self._state_lock:
            self._is_pressed = True
            self._press_start_time = time.time()
            self._last_event = ButtonEvent.PRESSED
        self.logger.debug("Button pressed")

    def _on_released(self) -> None:
        """Internal callback when button is released."""
        with self._state_lock:
            self._is_pressed = False

            if self._press_start_time is not None:
                press_duration = time.time() - self._press_start_time

                if press_duration >= self._long_press_threshold_sec:
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
            else:
                self._last_event = ButtonEvent.RELEASED

    @override
    def get_state(self) -> ButtonState:
        """Get current button state."""
        if self._pin is None:
            self.connect()

        with self._state_lock:
            press_duration = 0.0
            if self._is_pressed and self._press_start_time is not None:
                press_duration = time.time() - self._press_start_time

            return ButtonState(
                is_pressed=self._is_pressed,
                press_duration=press_duration,
                last_event=self._last_event,
            )

    @override
    def is_pressed(self) -> bool:
        """Check if button is currently pressed."""
        if self._pin is None:
            self.connect()

        with self._state_lock:
            return self._is_pressed

    @override
    def wait_for_press(self, timeout: float | None = None) -> bool:
        """Wait for button press event."""
        if self._pin is None:
            self.connect()

        start_time = time.time()

        while True:
            with self._state_lock:
                if self._is_pressed:
                    return True

            if timeout is not None and (time.time() - start_time) >= timeout:
                return False

            time.sleep(0.01)

    @override
    def wait_for_release(self, timeout: float | None = None) -> bool:
        """Wait for button release event."""
        if self._pin is None:
            self.connect()

        start_time = time.time()

        while True:
            with self._state_lock:
                if not self._is_pressed:
                    return True

            if timeout is not None and (time.time() - start_time) >= timeout:
                return False

            time.sleep(0.01)

    @override
    def close(self) -> None:
        """Clean up GPIO resources."""
        self._stop_polling.set()

        if self._polling_thread is not None:
            self._polling_thread.join(timeout=1.0)
            self._polling_thread = None

        if self._pin is not None:
            self._pin.deinit()
            self._pin = None

        self.logger.info("MCP2221 button connection closed")
