"""Tests for src.server.registry — model availability rules and the registry."""

import pytest

from src.core.enums import ModelType
from src.core.exceptions import ModelNotAvailable, ModelNotFound
from src.server.registry import ModelCapabilities, ModelConfig, ModelRegistry, _is_available


@pytest.fixture()
def checkpoint(tmp_path):
    """A checkpoint file that exists on disk."""
    ckpt = tmp_path / "model.pt"
    ckpt.write_bytes(b"fake weights")
    return ckpt


class TestIsAvailableSam1:
    def test_available_with_existing_checkpoint(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is True

    def test_unavailable_with_missing_checkpoint(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint="missing.pt")
        assert _is_available(cfg, tmp_path) is False

    def test_unavailable_with_no_checkpoint(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM1)
        assert _is_available(cfg, tmp_path) is False


class TestIsAvailableSam2:
    def test_available_via_hf_repo_alone(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM2, hf_repo="facebook/sam2.1")
        assert _is_available(cfg, tmp_path) is True

    def test_available_via_checkpoint_alone(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM2, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is True

    def test_unavailable_with_neither(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM2)
        assert _is_available(cfg, tmp_path) is False


class TestIsAvailableSam3:
    def test_available_with_hf_repo(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM3, hf_repo="facebook/sam3")
        assert _is_available(cfg, tmp_path) is True

    def test_unavailable_with_empty_hf_repo(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM3, hf_repo="")
        assert _is_available(cfg, tmp_path) is False

    def test_checkpoint_alone_is_not_enough_for_sam3(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM3, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is False


class TestIsAvailableYoloe:
    def test_available_with_existing_checkpoint(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.YOLOE, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is True

    def test_hf_repo_alone_is_not_enough_for_yoloe(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.YOLOE, hf_repo="some/repo")
        assert _is_available(cfg, tmp_path) is False


class TestIsAvailableYolo11:
    def test_available_with_existing_checkpoint(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.YOLO11, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is True

    def test_unavailable_with_missing_checkpoint(self, tmp_path):
        cfg = ModelConfig(id="m1", model_type=ModelType.YOLO11, checkpoint="missing.pt")
        assert _is_available(cfg, tmp_path) is False


class TestIsAvailablePathResolution:
    def test_relative_checkpoint_resolves_against_base_dir(self, tmp_path, checkpoint):
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint=checkpoint.name)
        assert _is_available(cfg, tmp_path) is True

    def test_absolute_checkpoint_ignores_base_dir(self, tmp_path, checkpoint):
        other_base = tmp_path / "unrelated_subdir"
        other_base.mkdir()
        cfg = ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint=str(checkpoint))
        assert _is_available(cfg, other_base) is True


class TestModelCapabilities:
    def test_repr_lists_supported_capabilities(self):
        caps = ModelCapabilities(supports_points=True, supports_text=True)
        assert repr(caps) == "ModelCapabilities(points, text)"

    def test_repr_with_no_capabilities(self):
        assert repr(ModelCapabilities()) == "ModelCapabilities()"


class TestModelRegistry:
    def test_get_returns_config_and_capabilities_for_available_model(self, tmp_path, checkpoint):
        configs = [ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint=checkpoint.name)]
        registry = ModelRegistry(configs, tmp_path)

        cfg, caps = registry.get("m1")

        assert cfg.id == "m1"
        assert caps.supports_points is True

    def test_get_raises_not_found_for_unknown_id(self, tmp_path):
        registry = ModelRegistry([], tmp_path)
        with pytest.raises(ModelNotFound):
            registry.get("nonexistent")

    def test_get_raises_not_available_for_missing_checkpoint(self, tmp_path):
        configs = [ModelConfig(id="m1", model_type=ModelType.SAM1, checkpoint="missing.pt")]
        registry = ModelRegistry(configs, tmp_path)

        with pytest.raises(ModelNotAvailable):
            registry.get("m1")

    def test_all_available_excludes_unavailable_models(self, tmp_path, checkpoint):
        configs = [
            ModelConfig(id="available", model_type=ModelType.SAM1, checkpoint=checkpoint.name),
            ModelConfig(id="unavailable", model_type=ModelType.SAM1, checkpoint="missing.pt"),
        ]
        registry = ModelRegistry(configs, tmp_path)

        assert registry.all_available() == ["available"]

    def test_repr_reports_available_over_total_count(self, tmp_path, checkpoint):
        configs = [
            ModelConfig(id="a", model_type=ModelType.SAM1, checkpoint=checkpoint.name),
            ModelConfig(id="b", model_type=ModelType.SAM1, checkpoint="missing.pt"),
        ]
        registry = ModelRegistry(configs, tmp_path)

        assert repr(registry) == "ModelRegistry(1 available / 2 total)"
