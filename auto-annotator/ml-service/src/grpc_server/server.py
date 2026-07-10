"""gRPC server hosting the SAM / augmentation / training compute services.

All three services run in one process on one port; the Go API may point its
SEGMENT/AUGMENT/TRAIN addresses at the same endpoint. Split into separate
processes later if GPU isolation is needed.
"""

from __future__ import annotations

from concurrent import futures

import grpc
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from src.config import AppConfig
from src.grpc_server.pb import compute_pb2_grpc as pb_grpc
from src.grpc_server.servicers import (
    AugmentationServicer,
    SegmentationServicer,
    TrainingServicer,
)
from src.utils import get_logger

logger = get_logger(__name__)

_DEFAULT_PORT = 50051
_DEFAULT_MAX_WORKERS = 8


class _GrpcEnvSettings(BaseSettings):
    """Raw env-var read for the gRPC server port override."""

    model_config = SettingsConfigDict(extra="ignore")

    grpc_port: int | None = Field(default=None, validation_alias="GRPC_PORT")


def _load_grpc_config() -> tuple[int, int]:
    """Return (port, max_workers) from config file, with env var override for port."""
    import tomllib  # noqa: PLC0415 — stdlib, cheap import

    paths = AppConfig.load().paths
    grpc_cfg: dict = {}
    if paths.server_config_file.exists():
        with paths.server_config_file.open("rb") as f:
            grpc_cfg = tomllib.load(f).get("grpc", {})

    env_port = _GrpcEnvSettings().grpc_port
    port = env_port if env_port is not None else int(grpc_cfg.get("port", _DEFAULT_PORT))
    max_workers = int(grpc_cfg.get("max_workers", _DEFAULT_MAX_WORKERS))
    return port, max_workers


def build_server(port: int, max_workers: int = _DEFAULT_MAX_WORKERS) -> grpc.Server:
    """Construct (but do not start) the gRPC server with all services registered."""
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    pb_grpc.add_SegmentationServiceServicer_to_server(SegmentationServicer(), server)
    pb_grpc.add_AugmentationServiceServicer_to_server(AugmentationServicer(), server)
    pb_grpc.add_TrainingServiceServicer_to_server(TrainingServicer(), server)
    server.add_insecure_port(f"[::]:{port}")
    return server


def serve(port: int | None = None) -> None:
    """Start the gRPC server and block until terminated."""
    cfg_port, max_workers = _load_grpc_config()
    resolved = port or cfg_port
    server = build_server(resolved, max_workers)
    server.start()
    logger.info("grpc_server_started", extra={"port": resolved, "max_workers": max_workers})
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
