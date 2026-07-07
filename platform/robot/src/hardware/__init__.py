"""
Hardware drivers for the robot.

Modules:
    - motors: LEGO motors via Build HAT (Pi Zero) / DC-encoder drive (Pi 5)
    - lidar: Slamtec RPLiDAR C1 (Pi 5)
    - camera: RPi Camera Module 3 (Pi 5)
    - imu: BNO085 IMU via MCP2221A (I2C / UART RVC)
    - hailo: Hailo 8 NPU (Pi 5)
    - button: Physical push-button with GPIO
    - display: SSD1306 OLED display
    - mcp2221: Shared MCP2221 USB bridge configuration

Symbols are resolved lazily (PEP 562): a backend's platform-specific
dependency (picamera2, hailort, buildhat, lgpio) is imported only when its
symbol is first accessed, so importing this package — or any submodule under
it — does not drag in every backend. This keeps the package importable on dev
machines and stops a consumer that needs only motors from pulling the camera
and NPU stacks.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

# Public name -> (submodule, attribute) for lazy resolution.
_LAZY: dict[str, tuple[str, str]] = {
    "ButtonEvent": ("button", "ButtonEvent"),
    "ButtonState": ("button", "ButtonState"),
    "ButtonConfig": ("button", "Config"),
    "ButtonDriver": ("button", "Driver"),
    "CameraConfig": ("camera", "Config"),
    "CameraDriver": ("camera", "Driver"),
    "CameraFrame": ("camera", "Frame"),
    "DisplayConfig": ("display", "Config"),
    "DisplayDriver": ("display", "Driver"),
    "HailoConfig": ("hailo", "Config"),
    "HailoDriver": ("hailo", "Driver"),
    "InferenceResult": ("hailo", "InferenceResult"),
    "IMUData": ("imu", "Data"),
    "IMUDriver": ("imu", "Driver"),
    "IMU_RVCData": ("imu", "RVCData"),
    "IMU_RVCDriver": ("imu", "RVCDriver"),
    "MCP2221Config": ("mcp2221", "MCP2221Config"),
    "CalibrationData": ("motors", "CalibrationData"),
    "MotorConfig": ("motors", "Config"),
    "BuildHatDriver": ("motors", "Driver"),
}

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


def __getattr__(name: str) -> object:
    """Resolve a public hardware symbol on first access (PEP 562)."""
    try:
        submodule, attr = _LAZY[name]
    except KeyError as err:
        msg = f"module {__name__!r} has no attribute {name!r}"
        raise AttributeError(msg) from err
    module = importlib.import_module(f"{__name__}.{submodule}")
    return getattr(module, attr)


def __dir__() -> list[str]:
    return __all__


if TYPE_CHECKING:
    # Static re-exports for type checkers / IDEs (no runtime import cost).
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
