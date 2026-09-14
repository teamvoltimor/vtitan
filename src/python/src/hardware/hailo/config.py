from typing import Self

from pydantic import Field, model_validator
from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.hailo_schema import HardwareHailo
from shared.config.generated.hardware.hailo_streaming_schema import HardwareHailoStreaming
from shared.domain.enums import GMR_CLASS_NAMES

from src.hardware.hailo.utils import load_class_map_from_yaml
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


def _detection_confidence_floor() -> float:
    """The shipped detection-confidence floor, one read of detector.toml.

    The three vision backends (CPU YOLO, Hailo-8, Hailo streaming) must drop
    below-threshold detections at the SAME confidence or a sign seen on the
    CPU path could vanish on the Hailo path. The shipped number lives once, in
    ``src/config/hardware/vision/detector.toml`` (the detector owns the concept:
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


class Config(HardwareBaseSettings, HardwareHailo):
    """Hailo configuration.

    Subclasses the generated DTO for the file-backed keys; ``min_confidence``
    and ``class_map`` stay wrapper behavior (see their docstrings) because the
    shared confidence floor deliberately lives once, in detector.toml, and the
    class map is derived from ``data_yaml_path``.
    """

    model_config = SettingsConfigDict(
        env_prefix="hailo_",
        toml_file=CONFIG_DIR / "hailo.toml",
        # The generated DTO is frozen; this loader resolves class_map after
        # validation (see _load_class_map), so it must stay mutable like the
        # hand-written model it replaces.
        frozen=False,
    )

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


class StreamingConfig(HardwareBaseSettings, HardwareHailoStreaming):
    """Configuration for continuous Hailo inference with camera.

    Subclasses the generated DTO for the file-backed keys; ``min_confidence``
    stays wrapper behavior for the same reason as :class:`Config` (the shared
    detector.toml floor).
    """

    model_config = SettingsConfigDict(
        env_prefix="hailo_stream_",
        toml_file=CONFIG_DIR / "hailo_streaming.toml",
        frozen=False,
    )

    min_confidence: float = Field(default_factory=_detection_confidence_floor)
    """
    Minimum confidence threshold for detections. Resolved from detector.toml's min_confidence like HailoConfig's -- one shipped number for all three backends.
    """
