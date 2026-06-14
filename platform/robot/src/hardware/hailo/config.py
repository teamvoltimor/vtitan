from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from src.hardware.camera.config import Config as CameraConfig
from src.hardware.hailo.utils import load_class_map_from_yaml


class Config(BaseSettings):
    """Hailo configuration."""

    model_config = SettingsConfigDict(
        env_prefix="hailo_",
    )

    model_path: str = "/usr/local/hailo/models/yolo11n.hef"
    """
    Path to the Hailo HEF model file. This should point to the compiled model that the Hailo driver will load for inference. If not provided, a default path will be used.
    """

    benchmark_iterations: int = 10
    """
    Number of iterations to run when benchmarking latency. If not provided, a default value will be used.
    """

    data_yaml_path: str = "/usr/local/hailo/models/data.yaml"
    """
    Path to YOLO data.yaml file containing class names. If not provided, a default class map will be used.
    """

    min_confidence: float = 0.45
    """
    Minimum confidence threshold for object detection. Detections with confidence below this value will be filtered out. Default is 0.45.
    """

    def __post_init__(self):
        try:
            self.class_map = load_class_map_from_yaml(self.data_yaml_path)
        except (FileNotFoundError, ValueError, KeyError):
            self.class_map = {0: "red_pillar", 1: "green_pillar", 2: "wall"}


class StreamingConfig(CameraConfig):
    """Configuration for continuous Hailo inference with camera."""

    model_config = SettingsConfigDict(
        env_prefix="hailo_stream_",
    )

    # Model input configuration
    model_input_width: int = 640
    """
    Width to resize image before inference (should match model input).
    """

    model_input_height: int = 640
    """
    Height to resize image before inference (should match model input).
    """

    # Inference configuration
    conf_threshold: float = Field(default=0.45, validation_alias="min_confidence")
    """
    Minimum confidence threshold for detections.
    """

    # Processing configuration
    queue_size: int = 1
    """
    Maximum number of frames to keep in the inference queue.
    """

    async_inference: bool = False
    """
    Use asynchronous inference (non-blocking).
    """
