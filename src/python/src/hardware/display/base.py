"""Abstract base classes for OLED display interface."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PIL import Image


@dataclass(slots=True)
class Config:
    """Configuration for OLED display driver."""

    width: int
    """Display width in pixels."""

    height: int
    """Display height in pixels."""

    i2c_address: int
    """I2C address of the display."""

    i2c_bus: int
    """I2C bus number."""


class Driver(ABC):
    """Abstract OLED display driver interface."""

    @abstractmethod
    def connect(self) -> None:
        """Initialize I2C connection and configure display."""

    @abstractmethod
    def clear(self) -> None:
        """Clear the display."""

    @abstractmethod
    def show_image(self, image: "Image.Image") -> None:
        """Display an image on the OLED.

        Args:
            image: PIL Image object to display. Must be in '1' or 'L' mode.
        """

    @abstractmethod
    def get_blank_image(self) -> "Image.Image":
        """Create a blank image with correct dimensions for this display.

        Returns:
            PIL Image object in '1' mode (1-bit pixels, black and white).
        """

    @abstractmethod
    def get_width(self) -> int:
        """Get display width in pixels."""

    @abstractmethod
    def get_height(self) -> int:
        """Get display height in pixels."""

    @abstractmethod
    def close(self) -> None:
        """Clean up display resources."""
