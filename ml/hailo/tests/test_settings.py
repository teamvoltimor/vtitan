"""Unit tests for src.settings' env-var-driven defaults."""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.settings import HailoSettings

if TYPE_CHECKING:
    import pytest


def test_defaults_when_no_env_vars_set(monkeypatch: pytest.MonkeyPatch) -> None:
    for key in (
        "HAILO_DOCKER_CONTAINER",
        "HAILO_DOCKER_IMAGE",
        "HAILO_HOST_UID",
        "HAILO_VIDEO_GID",
        "HAILO_X11_DISPLAY",
    ):
        monkeypatch.delenv(key, raising=False)

    settings = HailoSettings(_env_file=None)

    assert settings.docker_container == "hailo8_ai_sw_suite_2025-10_container"
    assert settings.host_uid == 1000
    assert settings.video_gid == 44
    assert settings.x11_display == ":0"


def test_env_vars_override_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("HAILO_DOCKER_CONTAINER", "custom_container")
    monkeypatch.setenv("HAILO_HOST_UID", "2000")
    monkeypatch.setenv("HAILO_VIDEO_GID", "108")
    monkeypatch.setenv("HAILO_X11_DISPLAY", ":1")

    settings = HailoSettings(_env_file=None)

    assert settings.docker_container == "custom_container"
    assert settings.host_uid == 2000
    assert settings.video_gid == 108
    assert settings.x11_display == ":1"
