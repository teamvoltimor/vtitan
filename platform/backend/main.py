"""Klevor backend — telemetry API server."""

from __future__ import annotations

import os

import uvicorn

from src.telemetry.server import app


def main() -> None:
    """Bootstrap the FastAPI server that exposes telemetry APIs."""
    port = int(os.environ.get("TELEMETRY_PORT", "8010"))
    uvicorn.run(
        "src.telemetry.server:app",
        host="0.0.0.0",  # noqa: S104
        port=port,
        log_level="info",
        reload=os.environ.get("TELEMETRY_RELOAD", "0") == "1",
    )


if __name__ == "__main__":
    main()
