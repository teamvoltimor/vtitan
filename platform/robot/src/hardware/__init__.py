"""
Hardware drivers for Klevor v2 robot.

Modules:
    - motors: LEGO motors via Build HAT (Pi Zero)
    - lidar: Slamtec RPLiDAR C1 (Pi 5)
    - camera: RPi Camera Module 3 (Pi 5)
    - imu: BNO085 IMU via MCP2221A (I2C / UART RVC)
    - hailo: Hailo 8 NPU (Pi 5)
    - button: Physical push-button with GPIO
    - display: SSD1306 OLED display
"""

from src.hardware.button import ButtonEvent, ButtonState, Config as ButtonConfig, Driver as ButtonDriver
from src.hardware.camera import CameraConfig, CameraDriver, CameraFrame
from src.hardware.display import Config as DisplayConfig, Driver as DisplayDriver
from src.hardware.hailo import HailoConfig, HailoDriver, InferenceResult
from src.hardware.imu import (
    Data as IMUData,
    Driver as IMUDriver,
    RVCData as IMU_RVCData,
    RVCDriver as IMU_RVCDriver,
)
from src.hardware.lidar import LidarConfig, LidarPoint, RPLidarDriver
from src.hardware.motors import BuildHatDriver, CalibrationData, MotorConfig

__all__ = [
    "BuildHatDriver",
    "ButtonConfig",
    "ButtonDriver",
    "ButtonEvent",
    "ButtonState",
    "CalibrationData",
    "CameraConfig",
    "CameraDriver",
    "CameraFrame",
    "DisplayConfig",
    "DisplayDriver",
    "HailoConfig",
    "HailoDriver",
    "IMUData",
    "IMUDriver",
    "IMU_RVCData",
    "IMU_RVCDriver",
    "InferenceResult",
    "LidarConfig",
    "LidarPoint",
    "MotorConfig",
    "RPLidarDriver",
]
