"""main.py – Auto-Annotator CLI entrypoint.

Usage
-----
    # Start the Gradio annotation app:
    uv run python main.py app

    # Start the SAM model server (keep running, restart app freely):
    uv run python main.py server

Both modes load .env from the project root before doing anything else.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

_START = time.monotonic()


def _run_app() -> None:
    from src.app import run_app  # noqa: PLC0415

    run_app()
    elapsed = time.monotonic() - _START
    print(f"[main] App built in {elapsed:.2f}s")  # noqa: T201


def _run_server() -> None:
    from src.server.main import run_server  # noqa: PLC0415

    run_server()


def main() -> None:
    """Parse CLI arguments and dispatch to the selected mode."""
    parser = argparse.ArgumentParser(
        prog="auto-annotator",
        description="Interactive SAM2-based image annotation tool.",
    )
    parser.add_argument(
        "mode",
        choices=["app", "server"],
        help="'app' starts the Gradio UI; 'server' starts the SAM model server.",
    )
    args = parser.parse_args()

    if args.mode == "app":
        _run_app()
    else:
        _run_server()


if __name__ == "__main__":
    main()
