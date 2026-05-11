"""SSD1306 OLED display driver for I2C."""

import logging
import threading
from typing import override

import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image

from src.hardware.display.base import (
    Driver as ABC_Driver,
)
from src.hardware.display.ssd1306.config import Config
from src.logger import configure_json_logging

configure_json_logging()


class Driver(ABC_Driver):
    """SSD1306 OLED display driver for I2C communication.

    Supports 128x64 or 128x32 monochrome OLED displays.
    Uses Adafruit CircuitPython libraries for I2C communication.
    """

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        self._display: SSD1306_I2C | None = None
        self._i2c: busio.I2C | None = None
        self._conn_lock: threading.Lock = threading.Lock()
        self.logger: logging.Logger = logging.getLogger(__name__)

    @override
    def connect(self) -> None:
        """Initialize I2C connection and configure display."""
        self._conn_lock.acquire()
        self.logger.info(
            "Connecting to SSD1306 display",
            extra={
                "details": {
                    "width": self.config.width,
                    "height": self.config.height,
                    "i2c_address": hex(self.config.i2c_address),
                    "i2c_bus": self.config.i2c_bus,
                },
            },
        )

        # If using MCP2221A, Blinka will automatically detect it as a USB HID device and provide I2C access via busio
        # Remember that if that's the case, BLINKA_MCP2221 env var must be set to "1" and the MCP2221A must be properly connected to the I2C bus with correct wiring and power.
        self._i2c = busio.I2C(board.SCL, board.SDA)

        # Create display object
        self._display = SSD1306_I2C(self.config.width, self.config.height, self._i2c, addr=self.config.i2c_address)

        # Clear display on startup
        self.clear()

        self.logger.info("SSD1306 display connected successfully")
        self._conn_lock.release()

    @override
    def clear(self) -> None:
        """Clear the display."""
        if self._display is None:
            self.connect()

        if self._display is not None:
            self._display.fill(0)
            self._display.show()

    @override
    def show_image(self, image: Image) -> None:
        """Display an image on the OLED.

        Args:
            image: PIL Image object to display. Must match display dimensions.
        """
        if self._display is None:
            self.connect()

        if self._display is None:
            return

        # Convert image to 1-bit mode if needed
        if image.mode != "1":
            image = image.convert("1")

        # Verify dimensions
        if image.size != (self.config.width, self.config.height):
            self.logger.warning(
                "Image dimensions do not match display",
                extra={
                    "details": {
                        "image_size": image.size,
                        "display_size": (self.config.width, self.config.height),
                    },
                },
            )
            # Resize image to fit display
            image = image.resize((self.config.width, self.config.height))

        # Display the image
        self._display.image(image)
        self._display.show()

    @override
    def get_blank_image(self) -> Image:
        """Create a blank image with correct dimensions for this display."""
        return Image.new("1", (self.config.width, self.config.height))

    @override
    def get_width(self) -> int:
        """Get display width in pixels."""
        return self.config.width

    @override
    def get_height(self) -> int:
        """Get display height in pixels."""
        return self.config.height

    @override
    def close(self) -> None:
        """Clean up display resources."""
        self._conn_lock.acquire()
        if self._display is not None:
            self.clear()
            self._display = None

        if self._i2c is not None:
            self._i2c.deinit()
            self._i2c = None

        self.logger.info("Display connection closed")
        self._conn_lock.release()
