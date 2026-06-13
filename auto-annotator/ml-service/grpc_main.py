"""ML-service entrypoint: SAM model server (daemon thread) + gRPC compute server."""
from __future__ import annotations

import argparse
import dataclasses
import threading
import time
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

from src.config import AppConfig  # noqa: E402 — must come after load_dotenv
from src.utils import get_logger  # noqa: E402

logger = get_logger(__name__)


def main() -> None:
    """Run the ML-service (SAM model server + gRPC server)."""
    parser = argparse.ArgumentParser(prog="ml-service")
    parser.add_argument("--model", dest="default_model", default=None)
    args = parser.parse_args()

    config = AppConfig.load()
    if args.default_model:
        config = dataclasses.replace(
            config,
            inference=dataclasses.replace(config.inference, default_model=args.default_model),
        )

    logger.info("starting ml-service", extra={"startup_s": round(time.monotonic(), 2)})

    from src.server.main import run_server  # noqa: PLC0415

    t = threading.Thread(target=run_server, args=(config,), daemon=True, name="sam-model-server")
    t.start()

    from src.grpc_server.server import serve  # noqa: PLC0415

    serve()


if __name__ == "__main__":
    main()
