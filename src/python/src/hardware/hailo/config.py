from typing import Self

from pydantic import Field, model_validator
from pydantic_settings import SettingsConfigDict
from shared.domain.enums import GMR_CLASS_NAMES

from src.hardware.camera.config import Config as CameraConfig
from src.hardware.hailo.utils import load_class_map_from_yaml
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


def _detection_confidence_floor() -> float:
    """The shipped detection-confidence floor, one read of detector.toml.

    The three vision backends (CPU YOLO, Hailo-8, Hailo streaming) must drop
    below-threshold detections at the SAME confidence or a sign seen on the
    CPU path could vanish on the Hailo path. The shipped number lives once, in
    ``config/hardware/vision/detector.toml`` (the detector owns the concept:
    every backend resolves its own key back to that file); these hailo fields
    cite it via ``default_factory`` rather than restating the 0.45, which had
    drifted before as three un-linked literals held together only by prose.

    Deferred import: ``src.vision.detector`` imports ``src.hardware.hailo``
    (for ``iter_nms_by_class``), so importing it at module level would cycle.
    The placeholders match how ``node.py``'s own threshold resolution builds a
    bare DetectorConfig -- only min_confidence's resolved TOML/env value wants
    reading.
    """
    from src.vision.detector import DetectorConfig  # noqa: PLC0415 - cycle-break, see docstring

    return DetectorConfig(model_path="", class_to_color={}).min_confidence


class Config(HardwareBaseSettings):
    """Hailo configuration."""

    model_config = SettingsConfigDict(
        env_prefix="hailo_",
        toml_file=CONFIG_DIR / "hailo.toml",
    )

    model_path: str = "/usr/local/hailo/models/gmr.hef"
    """
    Path to the Hailo HEF model file. This should point to the compiled model that the Hailo driver will load for inference. If not provided, a default path will be used.
    """

    inference_timeout_ms: int = 10000
    """
    Milliseconds to wait for a single async inference job before giving up. Guards against a wedged pipeline blocking the control loop forever.
    """

    benchmark_iterations: int = 10
    """
    Number of iterations to run when benchmarking latency. If not provided, a default value will be used.
    """

    data_yaml_path: str = "/usr/local/hailo/models/data.yaml"
    """
    Path to YOLO data.yaml file containing class names. If not provided, a default class map will be used.
    """

    min_confidence: float = Field(default_factory=_detection_confidence_floor)
    """
    Minimum confidence threshold for object detection. Detections with confidence below this value will be filtered out. The shipped value is detector.toml's min_confidence -- the one threshold the three backends share; override here (env HAILO_MIN_CONFIDENCE) only for deliberate A/B work.
    """

    class_map: dict[int, str] = Field(default_factory=dict)
    """Class id → name map, loaded from data_yaml_path (falls back to defaults below)."""

    @model_validator(mode="after")
    def _load_class_map(self) -> Self:
        # Falls back to the detector's own declared class order rather than a
        # separate guess. The previous fallback was (red_pillar, green_pillar,
        # wall) -- the wrong order, plus a class this model does not have --
        # which mislabels every detection without erroring.
        try:
            self.class_map = load_class_map_from_yaml(self.data_yaml_path)
        except (FileNotFoundError, ValueError, KeyError):
            self.class_map = dict(GMR_CLASS_NAMES)
        return self


class StreamingConfig(CameraConfig):
    """Configuration for continuous Hailo inference with camera."""

    model_config = SettingsConfigDict(
        env_prefix="hailo_stream_",
        toml_file=CONFIG_DIR / "hailo_streaming.toml",
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
    conf_threshold: float = Field(
        default_factory=_detection_confidence_floor,
        validation_alias="min_confidence",
    )
    """
    Minimum confidence threshold for detections. Resolved from detector.toml's min_confidence like HailoConfig's -- one shipped number for all three backends.
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
