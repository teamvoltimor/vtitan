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
    - mcp2221: Shared MCP2221 USB bridge configuration
"""

from src.hardware.button import (
    ButtonEvent,
    ButtonState,
    Config as ButtonConfig,
    Driver as ButtonDriver,
)
from src.hardware.camera import (
    Config as CameraConfig,
    Driver as CameraDriver,
    Frame as CameraFrame,
)
from src.hardware.display import (
    Config as DisplayConfig,
    Driver as DisplayDriver,
)
from src.hardware.hailo import (
    Config as HailoConfig,
    Driver as HailoDriver,
    InferenceResult,
)
from src.hardware.imu import (
    Data as IMUData,
    Driver as IMUDriver,
    RVCData as IMU_RVCData,
    RVCDriver as IMU_RVCDriver,
)
from src.hardware.mcp2221 import MCP2221Config
from src.hardware.motors import (
    CalibrationData,
    Config as MotorConfig,
    Driver as BuildHatDriver,
)

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
    "MCP2221Config",
    "MotorConfig",
]
