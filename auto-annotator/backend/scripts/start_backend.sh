#!/bin/bash
# Starts the TCP model server in the background, then launches the FastAPI
# HTTP API in the foreground so Docker can track its lifecycle.
set -e

uv run python main.py server &

exec uv run uvicorn src.api.app:app \
    --host 0.0.0.0 \
    --port "${API_PORT:-8000}"
