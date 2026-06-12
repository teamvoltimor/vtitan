"""CLI entrypoint for the Auto-Annotator backend stack.

Starts two co-located processes in one container:

1. **TCP model server** (port ``SERVER_PORT``, default 8765) – loads SAM once and
   keeps it in GPU memory so the HTTP API can delegate inference without reloading.
   Runs in a daemon thread; it binds its socket immediately so the API can connect
   before model loading finishes.

2. **FastAPI HTTP API** (port ``API_PORT``, default 8000) – serves the React
   frontend.  Runs on the main thread via uvicorn so the container exits cleanly
   on SIGTERM.
"""

from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path

from dotenv import load_dotenv

from src.config import AppConfig
from src.utils import get_logger

load_dotenv(Path(__file__).parent / ".env")

_START = time.monotonic()
logger = get_logger(__name__)


def _run_model_server(config: AppConfig) -> None:
    from src.server.main import run_server

    run_server(config)


def _run_api(config: AppConfig) -> None:
    import uvicorn

    uvicorn.run(
        "src.api.app:app",
        host=os.environ.get("API_HOST", "0.0.0.0"),  # noqa: S104
        port=config.api.port,
        log_level="warning",
        access_log=False,
    )


def main() -> None:
    """Start the SAM model server and FastAPI HTTP API."""
    parser = argparse.ArgumentParser(
        prog="auto-annotator",
        description="SAM model server + FastAPI API for the Auto-Annotator frontend.",
    )
    parser.add_argument(
        "--model",
        dest="default_model",
        help="Override the default SAM model ID loaded on startup.",
        default=None,
    )
    args = parser.parse_args()

    config = AppConfig.load()

    if args.default_model:
        import dataclasses

        config = dataclasses.replace(
            config,
            inference=dataclasses.replace(config.inference, default_model=args.default_model),
        )

    elapsed = time.monotonic() - _START
    logger.info("Launching backend stack; import took %.2fs", elapsed)

    server_thread = threading.Thread(
        target=_run_model_server,
        args=(config,),
        daemon=True,
        name="model-server",
    )
    server_thread.start()

    _run_api(config)


if __name__ == "__main__":
    main()
