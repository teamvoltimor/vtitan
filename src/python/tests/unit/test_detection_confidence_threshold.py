"""The three vision backends drop detections at ONE confidence threshold.

The floor used to be written three times -- detector.toml, hailo.toml and
hailo_streaming.toml each declared min_confidence = 0.45, the last under a key
(conf_threshold) that its own comment had to explain -- and the only thing
holding the three together was an "Matches hailo.toml..." prose note. Nothing
failed when one copy moved: a sign at threshold confidence would then vanish
from the Hailo path while clearing CPU detection, or the reverse -- neither
backend is wrong-looking on its own, and the sim (CPU) and the robot (Hailo)
would disagree in a way no single-file diff shows.

The shipped copy now lives once, in detector.toml, and the two hailo configs
resolve their fields back to it. This pins the agreement instead of the
prose: re-declaring a backend copy re-points that field at its own file, at
which point equal resolutions fail here loudly.
"""

import tomllib
from pathlib import Path

import pytest

from src.hardware.hailo.config import (
    Config as HailoConfig,
    StreamingConfig,
)
from src.vision.detector import DetectorConfig

_CONFIG = Path(__file__).resolve().parents[3] / "config" / "hardware"


def _shipped(toml_path: Path) -> object:
    with toml_path.open("rb") as f:
        return tomllib.load(f).get("min_confidence")


def test_all_three_backends_resolve_the_same_threshold():
    detector = DetectorConfig(model_path="", class_to_color={})
    assert detector.min_confidence == HailoConfig().min_confidence
    assert detector.min_confidence == StreamingConfig().conf_threshold


def test_bare_defaults_agree_with_the_single_shipped_copy():
    expected = _shipped(_CONFIG / "vision" / "detector.toml")
    assert DetectorConfig(model_path="", class_to_color={}).min_confidence == pytest.approx(expected)
    assert HailoConfig().min_confidence == pytest.approx(expected)
    assert StreamingConfig().conf_threshold == pytest.approx(expected)


def test_the_hailo_tomls_no_longer_declare_their_own_copy():
    """Re-declaring min_confidence re-points that field at its own file (the
    TOML source outranks the default in HardwareBaseSettings), which detaches
    that backend from the shared floor and fails the two tests above -- but
    only if the copy drifts. The key's return is the failure mode this exists
    to prevent, so its re-appearance fails here even while numerically equal:
    a per-backend A/B override stays an env var (HAILO_MIN_CONFIDENCE,
    DETECTOR_MIN_CONFIDENCE), where the divergence is visible next to itself.
    """
    assert _shipped(_CONFIG / "hailo.toml") is None
    assert _shipped(_CONFIG / "hailo_streaming.toml") is None
