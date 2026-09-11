"""Camera capture via the rpicam CLI, for hosts without picamera2."""

from src.hardware.camera.rpicam.driver import Config, Driver

__all__ = ["Config", "Driver"]
