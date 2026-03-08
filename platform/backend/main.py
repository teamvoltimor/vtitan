"""Klevor backend — telemetry API server."""

from __future__ import annotations

import logging
import os

import uvicorn


def main() -> None:
    """Bootstrap the FastAPI server that exposes telemetry APIs."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(name)s %(message)s",
    )
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
