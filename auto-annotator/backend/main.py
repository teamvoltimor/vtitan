"""CLI entrypoint for the Auto-Annotator backend stack."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from dotenv import load_dotenv

from src.utils import get_logger

load_dotenv(Path(__file__).parent / ".env")

_START = time.monotonic()
logger = get_logger(__name__)


def _run_server(default_model: str | None) -> None:
    from src.server.main import run_server

    run_server(default_model)


def main() -> None:
    """Start the SAM model server after parsing CLI arguments."""
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

    elapsed = time.monotonic() - _START
    logger.info("Launching SAM server; startup took %.2fs", elapsed)
    _run_server(default_model)


if __name__ == "__main__":
    main()
