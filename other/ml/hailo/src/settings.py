"""Host/deployment-specific environment configuration.

Everything here varies by machine (container/image tags, host UID/GID, X11
display) rather than by model or dataset -- the latter stay as plain code
constants/CLI flags in :mod:`src.constants`. Per the repo-wide convention, all
env-var reading goes through ``pydantic-settings`` rather than ad-hoc
``os.environ`` calls.
"""

from __future__ import annotations

from pydantic_settings import BaseSettings, SettingsConfigDict


class HailoSettings(BaseSettings):
    """Deployment defaults, overridable via ``HAILO_*`` env vars or a ``.env`` file.

    Every field also has a corresponding CLI flag (see ``main.py``); CLI flags
    take precedence since argparse defaults are populated from these settings
    at parser-construction time, and an explicit ``--flag`` always overrides
    an argparse default.
    """

    model_config = SettingsConfigDict(env_prefix="HAILO_", env_file=".env", extra="ignore")

    # DOCKER_SHARED_MOUNT (src/docker.py) is deliberately not here: it's a
    # fixed contract with the Hailo AI Software Suite image's own internal
    # mount expectations, not something that varies per host.
    docker_container: str = "hailo8_ai_sw_suite_2025-10_container"
    docker_image: str = "hailo8_ai_sw_suite_2025-10:1"

    # Host UID/GID for the Docker container's XDG_RUNTIME_DIR mount and
    # --group-add (video group). Defaults match a typical Debian/Ubuntu
    # desktop; override on hosts where these differ (e.g. Fedora's video
    # group GID, or a non-default primary user UID).
    host_uid: int = 1000
    video_gid: int = 44

    x11_display: str = ":0"


def load_settings() -> HailoSettings:
    """Return a freshly loaded :class:`HailoSettings` from the environment/.env file."""
    return HailoSettings()
