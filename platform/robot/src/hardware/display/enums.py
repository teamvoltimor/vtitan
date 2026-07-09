"""Display backend selector.

Typed backend lets the OLED node choose its I2C implementation from configuration
without string comparisons. RAW_I2C exists alongside BLINKA (not a replacement) --
BLINKA is the original Adafruit CircuitPython implementation; RAW_I2C talks to
/dev/i2c-N directly via ioctl, written after diagnosing a reproducible I2C bus hang
isolated to Blinka's busio.I2C + adafruit_ssd1306 (confirmed via smbus2-based stress
tests that the hardware/kernel/wiring are completely reliable, then ported off
smbus2 itself after it repeatedly destabilized `pixi install` on the Pi Zero 2W).
"""

from enum import Enum


class DisplayBackend(str, Enum):
    """OLED display I2C backend."""

    BLINKA = "blinka"
    RAW_I2C = "raw_i2c"
