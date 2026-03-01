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

from src.utils import get_logger

load_dotenv(Path(__file__).parent / ".env")

_START = time.monotonic()
logger = get_logger(__name__)



def _run_model_server(default_model: str | None) -> None:
    from src.server.main import run_server

    run_server(default_model)


def _run_api(api_host: str, api_port: int) -> None:
    import uvicorn

    uvicorn.run(
        "src.api.app:app",
        host=api_host,
        port=api_port,
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

    default_model = args.default_model or os.environ.get("DEFAULT_MODEL")
    api_port = int(os.environ.get("API_PORT", "8000"))
    api_host = os.environ.get("API_HOST", "0.0.0.0")  # noqa: S104

    elapsed = time.monotonic() - _START
    logger.info("Launching backend stack; import took %.2fs", elapsed)

    # Start the TCP model server in a daemon thread.  It binds its socket before
    # loading the model, so the 1-second grace period below is sufficient.
    server_thread = threading.Thread(
        target=_run_model_server,
        args=(default_model,),
        daemon=True,
        name="model-server",
    )
    server_thread.start()

    # connect_to_model_server() inside the FastAPI lifespan retries until the
    # TCP socket is ready, so no sleep is needed here.
    # Block on uvicorn; container exits cleanly when it receives SIGTERM.
    _run_api(api_host, api_port)


if __name__ == "__main__":
    main()
