# Build stage – install Python deps (devel base provides compilers for any C extensions).
# Ubuntu 24.04 ships Python 3.12 in main repos; no deadsnakes PPA needed.
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.9"

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.12 python3.12-dev python3.12-venv git curl \
    && rm -rf /var/lib/apt/lists/*

# Install uv from its official image – no pip bootstrap needed.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /app

# Copy dependency manifests first so this layer is cached until they change.
COPY backend/pyproject.toml backend/uv.lock ./

# Use a BuildKit cache mount so uv's wheel cache persists across builds.
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev --frozen --python /usr/bin/python3.12


# Runtime stage – much smaller: no build tools, no CUDA headers.
FROM nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04

ARG SERVER_PORT=8765
ARG API_PORT=8000

ENV DEBIAN_FRONTEND=noninteractive \
    SERVER_PORT=${SERVER_PORT} \
    API_PORT=${API_PORT} \
    HF_HUB_CACHE=/app/models \
    PYTHONUNBUFFERED=1 \
    PATH="/app/.venv/bin:$PATH"

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.12 libgl1 libglib2.0-0 curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy the virtual environment built in the builder stage.
COPY --from=builder /app/.venv .venv

# Copy application source (models are mounted as a volume at runtime).
COPY backend/main.py .
COPY backend/config/ config/
COPY backend/src/ src/

EXPOSE ${SERVER_PORT} ${API_PORT}

# DEFAULT_MODEL is optional; skip --model flag when unset.
CMD ["sh", "-c", "python main.py ${DEFAULT_MODEL:+--model $DEFAULT_MODEL}"]
