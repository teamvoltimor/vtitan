"""Klevor backend — telemetry API server."""

from __future__ import annotations

import logging

import uvicorn

from src.telemetry.config import ServerConfig


def main() -> None:
    """Bootstrap the FastAPI server that exposes telemetry APIs."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(name)s %(message)s",
    )
    config = ServerConfig.from_env()
    uvicorn.run(
        "src.telemetry.app:app",
        host="0.0.0.0",  # noqa: S104
        port=config.port,
        log_level="info",
        reload=config.reload,
    )


if __name__ == "__main__":
    main()
