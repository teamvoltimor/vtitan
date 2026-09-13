"""Tests for src.config's pydantic-settings-backed env-var reads."""

import pytest

from src.core.config import APIConfig, InferenceConfig, PathConfig


@pytest.fixture(autouse=True)
def _clear_env(monkeypatch):
    """Every config-related env var starts unset unless a test sets it explicitly."""
    for key in (
        "MODELS_DIR",
        "DB_PATH",
        "MODELS_CONFIG",
        "SERVER_CONFIG",
        "HF_HUB_CACHE",
        "HF_TOKEN",
        "API_PORT",
        "API_PUBLIC_URL",
        "DEFAULT_MODEL",
    ):
        monkeypatch.delenv(key, raising=False)


class TestPathConfig:
    def test_defaults_are_relative_to_base_dir(self):
        cfg = PathConfig.load()
        assert cfg.models_dir == cfg.base_dir / "models"
        assert cfg.db_path == cfg.base_dir / "data" / "manifest.db"

    def test_env_vars_override_defaults(self, monkeypatch, tmp_path):
        monkeypatch.setenv("MODELS_DIR", str(tmp_path / "custom-models"))
        monkeypatch.setenv("DB_PATH", str(tmp_path / "custom.db"))

        cfg = PathConfig.load()

        assert cfg.models_dir == tmp_path / "custom-models"
        assert cfg.db_path == tmp_path / "custom.db"


class TestAPIConfig:
    def test_defaults(self):
        cfg = APIConfig.load()
        assert cfg.port == 8000
        assert cfg.public_url == "http://localhost:8000"

    def test_public_url_defaults_from_resolved_port(self, monkeypatch):
        monkeypatch.setenv("API_PORT", "9100")
        cfg = APIConfig.load()
        assert cfg.port == 9100
        assert cfg.public_url == "http://localhost:9100"

    def test_env_vars_override_defaults(self, monkeypatch):
        monkeypatch.setenv("API_PORT", "9100")
        monkeypatch.setenv("API_PUBLIC_URL", "https://annotator.example.com")

        cfg = APIConfig.load()

        assert cfg.port == 9100
        assert cfg.public_url == "https://annotator.example.com"


class TestInferenceConfig:
    def test_defaults_to_empty_strings(self):
        cfg = InferenceConfig.load()
        assert cfg.hf_hub_cache == ""
        assert cfg.hf_token == ""
        assert cfg.default_model == ""

    def test_env_vars_override_defaults(self, monkeypatch):
        monkeypatch.setenv("HF_TOKEN", "secret-token")
        monkeypatch.setenv("DEFAULT_MODEL", "sam2.1_l")

        cfg = InferenceConfig.load()

        assert cfg.hf_token == "secret-token"
        assert cfg.default_model == "sam2.1_l"
