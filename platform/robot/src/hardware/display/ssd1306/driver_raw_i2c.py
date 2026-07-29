"""SSD1306 OLED display driver using raw Linux I2C ioctl calls (no smbus2/Blinka dependency).

Alternative backend to driver.py's Adafruit CircuitPython (Blinka) implementation --
written after diagnosing a reproducible I2C bus hang isolated to Blinka's busio.I2C +
adafruit_ssd1306. Confirmed via smbus2-based stress tests (100+ back-to-back 32-byte
block writes, zero hangs, ~3.2ms each) that the I2C hardware, wiring, and kernel driver
are completely reliable -- the bug is specific to that library stack, not the bus.

Talks to /dev/i2c-N directly via the I2C_SLAVE ioctl + plain write() -- the same simple,
non-combined transaction shape smbus2's write_i2c_block_data used under the hood, just
without depending on the smbus2 PyPI package (which repeatedly destabilized `pixi
install` on the resource-constrained Pi Zero 2W).

Select via DISPLAY_BACKEND=raw_i2c (see oled_display_node.py).
"""

import logging
import os
import threading
from fcntl import ioctl
from typing import override

from PIL import Image

from src.hardware.display.base import Driver as ABC_Driver
from src.hardware.display.ssd1306.config import Config
from src.logger import configure_json_logging

configure_json_logging()

_I2C_SLAVE = 0x0703
"""Linux ioctl request code to set the target I2C slave address on a /dev/i2c-N fd."""

_CONTROL_COMMAND = 0x00
_CONTROL_DATA = 0x40

_DISPLAY_HEIGHT_128X64 = 64
"""Pixel height of the 128x64 SSD1306 variant (vs. 128x32), selecting COM pin config."""

_DISPLAYOFF = 0xAE
_DISPLAYON = 0xAF
_SETDISPLAYCLOCKDIV = 0xD5
_SETMULTIPLEX = 0xA8
_SETDISPLAYOFFSET = 0xD3
_SETSTARTLINE = 0x40
_CHARGEPUMP = 0x8D
_MEMORYMODE = 0x20
_SEGREMAP = 0xA1
_COMSCANDEC = 0xC8
_SETCOMPINS = 0xDA
_SETCONTRAST = 0x81
_SETPRECHARGE = 0xD9
_SETVCOMDETECT = 0xDB
_DISPLAYALLON_RESUME = 0xA4
_NORMALDISPLAY = 0xA6
_COLUMNADDR = 0x21
_PAGEADDR = 0x22


class RawI2CDriver(ABC_Driver):
    """SSD1306 OLED display driver using raw /dev/i2c-N ioctl calls."""

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        self._fd: int | None = None
        self._pages: int = self.config.height // 8
        self._conn_lock: threading.Lock = threading.Lock()
        self.logger: logging.Logger = logging.getLogger(__name__)

    def _write_command(self, *commands: int) -> None:
        if self._fd is None:
            return
        for cmd in commands:
            os.write(self._fd, bytes([_CONTROL_COMMAND, cmd]))

    def _write_data(self, data: bytes) -> None:
        """Write the full data buffer in one I2C transaction.

        Was 32-byte chunks (one os.write() per chunk, 32 syscalls + I2C
        START/STOP pairs for a 1024-byte frame) -- that limit comes from the
        SMBus block-write protocol's 32-byte max, which does not apply here:
        this path is a plain i2c-dev write() with the control byte sent once
        up front, not repeated SMBus block transfers. The SSD1306 protocol
        only needs the control byte once per transaction (Co bit low means
        "everything after this is data"), and the kernel's i2c-dev write()
        accepts a buffer far larger than 1025 bytes. Measured on hardware:
        this was ~100ms of blocking os.write() calls per frame at 10Hz --
        i.e. the entire timer period -- and a real, confirmed contributor to
        pi_zero_peripherals_node's CPU cost.
        """
        if self._fd is None:
            return
        os.write(self._fd, bytes([_CONTROL_DATA]) + data)

    def _set_addressing_window(self) -> None:
        self._write_command(
            _COLUMNADDR,
            0,
            self.config.width - 1,
            _PAGEADDR,
            0,
            self._pages - 1,
        )

    @override
    def connect(self) -> None:
        """Initialize I2C connection and configure display."""
        self._conn_lock.acquire()
        self.logger.info(
            "Connecting to SSD1306 display (raw_i2c backend)",
            extra={
                "details": {
                    "width": self.config.width,
                    "height": self.config.height,
                    "i2c_address": hex(self.config.i2c_address),
                    "i2c_bus": self.config.i2c_bus,
                },
            },
        )

        self._fd = os.open(f"/dev/i2c-{self.config.i2c_bus}", os.O_RDWR)
        ioctl(self._fd, _I2C_SLAVE, self.config.i2c_address)

        multiplex = self.config.height - 1
        com_pins = 0x12 if self.config.height == _DISPLAY_HEIGHT_128X64 else 0x02

        self._write_command(
            _DISPLAYOFF,
            _SETDISPLAYCLOCKDIV,
            0x80,
            _SETMULTIPLEX,
            multiplex,
            _SETDISPLAYOFFSET,
            0x00,
            _SETSTARTLINE,
            _CHARGEPUMP,
            0x14,
            _MEMORYMODE,
            0x00,
            _SEGREMAP,
            _COMSCANDEC,
            _SETCOMPINS,
            com_pins,
            _SETCONTRAST,
            0xCF,
            _SETPRECHARGE,
            0xF1,
            _SETVCOMDETECT,
            0x40,
            _DISPLAYALLON_RESUME,
            _NORMALDISPLAY,
            _DISPLAYON,
        )

        self.clear()

        self.logger.info("SSD1306 display connected successfully (raw_i2c backend)")
        self._conn_lock.release()

    @override
    def clear(self) -> None:
        """Clear the display."""
        if self._fd is None:
            self.connect()
            return

        blank = bytes(self.config.width * self._pages)
        self._set_addressing_window()
        self._write_data(blank)

    @override
    def show_image(self, image: Image.Image) -> None:
        """Display an image on the OLED.

        Args:
            image: PIL Image object to display. Must match display dimensions.
        """
        if self._fd is None:
            self.connect()

        if self._fd is None:
            return

        if image.mode != "1":
            image = image.convert("1")

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
            image = image.resize((self.config.width, self.config.height))

        buffer = self._image_to_pages(image)
        self._set_addressing_window()
        self._write_data(buffer)

    def _image_to_pages(self, image: Image.Image) -> bytes:
        """Convert a 1-bit PIL image to SSD1306 page-addressed, column-major bytes.

        Each output byte packs 8 vertically-stacked pixels (LSB = topmost row of the
        page) for one column -- the native SSD1306 horizontal-addressing-mode layout,
        not the row-major layout PIL's own tobytes() would give.
        """
        width, height = image.size
        pixels = image.load()
        assert pixels is not None
        buffer = bytearray(width * self._pages)
        for page in range(self._pages):
            for x in range(width):
                byte = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if y < height and pixels[x, y]:
                        byte |= 1 << bit
                buffer[page * width + x] = byte
        return bytes(buffer)

    @override
    def get_blank_image(self) -> Image.Image:
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
        if self._fd is not None:
            self._write_command(_DISPLAYOFF)
            os.close(self._fd)
            self._fd = None

        self.logger.info("Display connection closed (raw_i2c backend)")
        self._conn_lock.release()
