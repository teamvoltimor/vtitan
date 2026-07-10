"""src.config – Typed configuration management for paths, server, and inference.

Consolidates environment variables, TOML files, and hardcoded defaults into
strongly-typed dataclasses. Single entry point for all app configuration.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

# Environment variable name constants.
# Defined here so load() methods and the _*EnvSettings classes below reference
# named symbols instead of bare strings.

ENV_MODELS_DIR: str = "MODELS_DIR"
"""Environment variable: directory containing model checkpoint files."""

ENV_DB_PATH: str = "DB_PATH"
"""Environment variable: path to the SQLite manifest database."""

ENV_MODELS_CONFIG: str = "MODELS_CONFIG"
"""Environment variable: path to the models.toml configuration file."""

ENV_SERVER_CONFIG: str = "SERVER_CONFIG"
"""Environment variable: path to the server config TOML file."""

ENV_MODEL_SERVER_PORT: str = "MODEL_SERVER_PORT"
"""Legacy environment variable used to override the model server port."""

ENV_SERVER_PORT: str = "SERVER_PORT"
"""Environment variable used to override the port specified in the server config file."""

ENV_HF_HUB_CACHE: str = "HF_HUB_CACHE"
"""Environment variable: HuggingFace hub cache directory."""

ENV_HF_TOKEN: str = "HF_TOKEN"  # noqa: S105 — this is an env var name, not a credential
"""Environment variable: HuggingFace API token (required for gated models)."""

ENV_API_PORT: str = "API_PORT"
"""HTTP API server port."""

ENV_API_PUBLIC_URL: str = "API_PUBLIC_URL"
"""Base URL exposed to clients when building image URLs."""

ENV_DEFAULT_MODEL: str = "DEFAULT_MODEL"
"""Environment variable: model ID to load on server startup."""


class _PathEnvSettings(BaseSettings):
    """Raw env-var reads for :class:`PathConfig`.

    Unset fields stay ``None`` so ``PathConfig.load()`` can apply its own
    ``base_dir``-relative defaults.
    """

    model_config = SettingsConfigDict(extra="ignore")

    models_dir: Path | None = Field(default=None, validation_alias=ENV_MODELS_DIR)
    db_path: Path | None = Field(default=None, validation_alias=ENV_DB_PATH)
    models_config: Path | None = Field(default=None, validation_alias=ENV_MODELS_CONFIG)
    server_config: Path | None = Field(default=None, validation_alias=ENV_SERVER_CONFIG)


class _ServerEnvSettings(BaseSettings):
    """Raw env-var reads for :class:`ServerConfig`'s port precedence.

    Two separate optional fields (rather than one field with ``AliasChoices``)
    so ``ServerConfig.load()`` can report exactly which env var supplied the
    port in ``port_source``, matching the pre-migration behavior.
    """

    model_config = SettingsConfigDict(extra="ignore")

    server_port: int | None = Field(default=None, validation_alias=ENV_SERVER_PORT)
    model_server_port: int | None = Field(default=None, validation_alias=ENV_MODEL_SERVER_PORT)


class _APIEnvSettings(BaseSettings):
    """Raw env-var reads for :class:`APIConfig`."""

    model_config = SettingsConfigDict(extra="ignore")

    api_port: int | None = Field(default=None, validation_alias=ENV_API_PORT)
    api_public_url: str | None = Field(default=None, validation_alias=ENV_API_PUBLIC_URL)


class _InferenceEnvSettings(BaseSettings):
    """Raw env-var reads for :class:`InferenceConfig`."""

    model_config = SettingsConfigDict(extra="ignore")

    hf_hub_cache: str = Field(default="", validation_alias=ENV_HF_HUB_CACHE)
    hf_token: str = Field(default="", validation_alias=ENV_HF_TOKEN)
    default_model: str = Field(default="", validation_alias=ENV_DEFAULT_MODEL)


def _load_toml(path: Path) -> dict[str, Any]:
    """Load and parse a TOML file, returning empty dict if missing or invalid."""
    if not path.exists():
        return {}
    try:
        with path.open("rb") as fp:
            return tomllib.load(fp)
    except (tomllib.TOMLDecodeError, OSError):
        return {}


@dataclass(frozen=True)
class PathConfig:
    """Filesystem paths derived from environment variables or defaults.

    Attributes:
        base_dir: Project root directory (parent of src/).
        models_dir: Directory containing model checkpoints (env: MODELS_DIR).
        pending_dir: Directory for unprocessed images to annotate.
        labels_dir: Directory where YOLO label files are written.
        images_dir: Directory where annotated images are organized by class.
        data_yaml_path: Path to generated YOLO data.yaml.
        db_path: SQLite manifest database path (env: DB_PATH).
        config_file: Models TOML configuration (env: MODELS_CONFIG).
        server_config_file: Server configuration TOML (env: SERVER_CONFIG).
    """

    base_dir: Path
    models_dir: Path
    pending_dir: Path
    labels_dir: Path
    images_dir: Path
    data_yaml_path: Path
    db_path: Path
    config_file: Path
    server_config_file: Path

    @classmethod
    def load(cls) -> PathConfig:
        """Load paths from environment variables and defaults."""
        base_dir = Path(__file__).parent.parent
        env = _PathEnvSettings()

        return PathConfig(
            base_dir=base_dir,
            models_dir=env.models_dir or base_dir / "models",
            pending_dir=base_dir / "data" / "pending",
            labels_dir=base_dir / "data" / "labels",
            images_dir=base_dir / "data" / "images",
            data_yaml_path=base_dir / "data" / "data.yaml",
            db_path=env.db_path or base_dir / "data" / "manifest.db",
            config_file=env.models_config or base_dir / "config" / "models.toml",
            server_config_file=env.server_config or base_dir / "config" / "server.toml",
        )


@dataclass(frozen=True)
class ServerConfig:
    """Server network configuration (model server, TCP).

    Attributes:
        host: Localhost address for model server and TCP client.
        port: TCP port the model server binds to.
        port_source: Description of where port was resolved from.
        recv_chunk_size: Maximum bytes per socket recv() call.
    """

    host: str
    port: int
    port_source: str
    recv_chunk_size: int

    @classmethod
    def load(cls, paths: PathConfig) -> ServerConfig:
        """Load server configuration from environment, config file, or defaults."""
        server_config_dict = _load_toml(paths.server_config_file).get("server", {})
        default_port = 8765
        env = _ServerEnvSettings()

        if env.server_port is not None:
            port, source = env.server_port, f"env:{ENV_SERVER_PORT}"
        elif env.model_server_port is not None:
            port, source = env.model_server_port, f"env:{ENV_MODEL_SERVER_PORT}"
        elif server_config_dict.get("port"):
            port, source = int(server_config_dict["port"]), f"config:{paths.server_config_file.name}"
        else:
            port, source = default_port, "default"

        return ServerConfig(
            host="127.0.0.1",
            port=port,
            port_source=source,
            recv_chunk_size=65536,
        )


@dataclass(frozen=True)
class APIConfig:
    """HTTP API configuration.

    Attributes:
        port: Port the HTTP API listens on (env: API_PORT).
        public_url: Base URL exposed to clients for image downloads (env: API_PUBLIC_URL).
    """

    port: int
    public_url: str

    @classmethod
    def load(cls) -> APIConfig:
        """Load API configuration from environment variables."""
        env = _APIEnvSettings()
        port = env.api_port if env.api_port is not None else 8000
        return APIConfig(
            port=port,
            public_url=env.api_public_url if env.api_public_url is not None else f"http://localhost:{port}",
        )


@dataclass(frozen=True)
class InferenceConfig:
    """Model inference configuration (SAM2, masks, etc.).

    Attributes:
        sam2_checkpoint_filename: Local SAM 2.1 checkpoint file to search for.
        sam2_hf_repo: HuggingFace repo to download from if local checkpoint missing.
        sam2_config_path: Relative path to Hiera YAML config inside SAM2 module.
        default_mask_score: Fallback confidence score when backend returns none.
        mask_labels: Human-readable labels for SAM's three mask outputs.
        hf_hub_cache: HuggingFace hub cache directory (env: HF_HUB_CACHE).
        hf_token: HuggingFace API token for gated models (env: HF_TOKEN).
        default_model: Model ID to load on startup (env: DEFAULT_MODEL).
    """

    sam2_checkpoint_filename: str
    sam2_hf_repo: str
    sam2_config_path: str
    default_mask_score: float
    mask_labels: list[str]
    hf_hub_cache: str
    hf_token: str
    default_model: str

    @classmethod
    def load(cls) -> InferenceConfig:
        """Load inference configuration from environment and defaults."""
        env = _InferenceEnvSettings()
        return InferenceConfig(
            sam2_checkpoint_filename="sam2.1_l.pt",
            sam2_hf_repo="facebook/sam2.1-hiera-large",
            sam2_config_path="configs/sam2.1/sam2.1_hiera_l.yaml",
            default_mask_score=1.0,
            mask_labels=["Precise (0)", "Object (1)", "Broad (2)"],
            hf_hub_cache=env.hf_hub_cache,
            hf_token=env.hf_token,
            default_model=env.default_model,
        )


@dataclass(frozen=True)
class AppConfig:
    """Complete application configuration (singleton loaded at startup).

    Attributes:
        paths: Filesystem path configuration.
        server: Model server TCP configuration.
        api: HTTP API configuration.
        inference: Model inference configuration.
    """

    paths: PathConfig
    server: ServerConfig
    api: APIConfig
    inference: InferenceConfig

    @classmethod
    def load(cls) -> AppConfig:
        """Load all configuration from environment, config files, and defaults."""
        paths = PathConfig.load()
        return AppConfig(
            paths=paths,
            server=ServerConfig.load(paths),
            api=APIConfig.load(),
            inference=InferenceConfig.load(),
        )
