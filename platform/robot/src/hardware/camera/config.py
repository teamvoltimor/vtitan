from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings):
    """Camera streaming configuration."""

    model_config = SettingsConfigDict(
        env_prefix="camera_",
        toml_file=CONFIG_DIR / "camera" / "config.toml",
    )

    device: str = "/dev/video0"
    """
    Camera device path (e.g., /dev/video0).
    """

    width: int = 640
    """
    Camera capture width in pixels.
    """

    height: int = 640
    """
    Camera capture height in pixels.
    """

    fps: int = 30
    """
    Target frames per second for camera capture.
    """

    rotation: int = 0
    """
    Camera rotation in degrees (0, 90, 180, 270).
    """

    hflip: bool = False
    """
    Horizontal flip the camera image.
    """

    vflip: bool = False
    """
    Vertical flip the camera image.
    """
