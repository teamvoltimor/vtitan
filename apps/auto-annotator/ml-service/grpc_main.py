"""ML-service entrypoint: gRPC compute server (model loads on its own background thread)."""
from __future__ import annotations

import argparse
import time
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

from src.core.config import AppConfig  # noqa: E402 — must come after load_dotenv
from src.utils import get_logger  # noqa: E402

logger = get_logger(__name__)


def main() -> None:
    """Run the ML-service (SAM model server + gRPC server)."""
    parser = argparse.ArgumentParser(prog="ml-service")
    parser.add_argument("--model", dest="default_model", default=None)
    args = parser.parse_args()

    config = AppConfig.load()
    if args.default_model:
        config = config.model_copy(
            update={"inference": config.inference.model_copy(update={"default_model": args.default_model})},
        )

    logger.info("starting ml-service", extra={"startup_s": round(time.monotonic(), 2)})

    from src.grpc_server.server import serve  # noqa: PLC0415

    serve(config)


if __name__ == "__main__":
    main()
