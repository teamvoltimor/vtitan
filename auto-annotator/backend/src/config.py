"""src.config – Typed configuration management for paths, server, and inference.

Consolidates environment variables, TOML files, and hardcoded defaults into
strongly-typed dataclasses. Single entry point for all app configuration.
"""

from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


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

        return PathConfig(
            base_dir=base_dir,
            models_dir=Path(
                os.environ.get("MODELS_DIR", str(base_dir / "models")),
            ),
            pending_dir=base_dir / "data" / "pending",
            labels_dir=base_dir / "data" / "labels",
            images_dir=base_dir / "data" / "images",
            data_yaml_path=base_dir / "data" / "data.yaml",
            db_path=Path(
                os.environ.get("DB_PATH", str(base_dir / "data" / "manifest.db")),
            ),
            config_file=Path(
                os.environ.get("MODELS_CONFIG", str(base_dir / "config" / "models.toml")),
            ),
            server_config_file=Path(
                os.environ.get("SERVER_CONFIG", str(base_dir / "config" / "server.toml")),
            ),
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

        port, source = (
            (int(os.environ.get("SERVER_PORT")), "env:SERVER_PORT")
            if os.environ.get("SERVER_PORT")
            else (
                (int(os.environ.get("MODEL_SERVER_PORT")), "env:MODEL_SERVER_PORT")
                if os.environ.get("MODEL_SERVER_PORT")
                else (
                    (int(server_config_dict.get("port")), f"config:{paths.server_config_file.name}")
                    if server_config_dict.get("port")
                    else (default_port, "default")
                )
            )
        )

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
        port = int(os.environ.get("API_PORT", "8000"))
        return APIConfig(
            port=port,
            public_url=os.environ.get(
                "API_PUBLIC_URL",
                f"http://localhost:{port}",
            ),
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
        return InferenceConfig(
            sam2_checkpoint_filename="sam2.1_l.pt",
            sam2_hf_repo="facebook/sam2.1-hiera-large",
            sam2_config_path="configs/sam2.1/sam2.1_hiera_l.yaml",
            default_mask_score=1.0,
            mask_labels=["Precise (0)", "Object (1)", "Broad (2)"],
            hf_hub_cache=os.environ.get("HF_HUB_CACHE", ""),
            hf_token=os.environ.get("HF_TOKEN", ""),
            default_model=os.environ.get("DEFAULT_MODEL", ""),
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
