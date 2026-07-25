"""GPIO driver for the challenge-mode jumper (GPIO23 shorted to GND vs. floating high)."""

import logging

from gpiozero import InputDevice

from src.hardware.challenge_mode.config import Config
from src.logger import configure_json_logging

configure_json_logging()


class Driver:
    """Reads the challenge-mode jumper via the internal pull-up, no external resistor.

    Shorted to GND (LOW) -- Obstacle Challenge. Absent, pulled up internally (HIGH) --
    Open Challenge. Unlike the button driver this exposes single instantaneous reads only:
    the jumper is a static wire link, not a momentary control, so there's no
    debounce/long-press state to track. Sampling the read several times to confirm it's
    stable before trusting it is the caller's responsibility -- see state_machine_node's
    boot-check loop, which needs non-blocking per-tick reads rather than a driver-internal
    sleep loop.
    """

    def __init__(self, config: Config | None = None) -> None:
        self.config: Config = config or Config()
        self._input: InputDevice | None = None
        self.logger: logging.Logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Initialize GPIO with the internal pull-up enabled."""
        self.logger.info(
            "Connecting to challenge-mode jumper",
            extra={"details": {"gpio_pin": self.config.gpio_pin}},
        )
        self._input = InputDevice(self.config.gpio_pin, pull_up=True)
        self.logger.info("Challenge-mode jumper connected successfully")

    def is_jumper_inserted(self) -> bool:
        """True if the jumper is shorting the pin to GND (Obstacle Challenge)."""
        if self._input is None:
            self.connect()
        assert self._input is not None
        return self._input.is_active

    def close(self) -> None:
        """Clean up GPIO resources."""
        if self._input is not None:
            self._input.close()
            self._input = None
