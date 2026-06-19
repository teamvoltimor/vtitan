"""Base class for hardware driver nodes with common lifecycle management."""

import logging
from abc import ABC, abstractmethod

from rclpy.node import Node

logger = logging.getLogger(__name__)


class HardwareNode(Node, ABC):
    """Abstract base class for all hardware driver ROS2 nodes.

    Provides common initialization pattern, error handling, and lifecycle
    management to eliminate duplication across hardware nodes.

    Subclasses must implement:
        - create_driver(): Instantiate and return driver
        - init_driver(driver): Perform initialization steps
    """

    @abstractmethod
    def create_driver(self) -> object:
        """Create and return driver instance.

        Returns:
            An initialized driver object (hardware specific).
        """

    @abstractmethod
    def init_driver(self, driver: object) -> None:
        """Perform driver-specific initialization steps.

        Args:
            driver: The driver instance from create_driver().

        Raises:
            HardwareError: If initialization fails.
        """

    def init_hardware(self) -> bool:
        """Common hardware initialization pattern with error handling.

        Creates driver and initializes it with structured error logging.
        Subclasses call this in __init__.
        """
        try:
            self.driver = self.create_driver()
            self.init_driver(self.driver)
        except (RuntimeError, ValueError, ImportError, OSError, TimeoutError) as e:
            self.get_logger().error(
                f"{self.__class__.__name__} hardware initialization failed: {type(e).__name__}: {e}",
            )
            raise
        else:
            self.get_logger().info(
                f"{self.__class__.__name__} hardware initialized successfully "
                f"(driver: {self.driver.__class__.__name__})",
            )
            return True
