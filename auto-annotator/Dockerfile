# Build stage – install Python deps (devel base provides compilers for any C extensions).
FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.9"

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common gnupg curl git \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3.12 python3.12-dev python3.12-venv \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /app

COPY backend/pyproject.toml backend/uv.lock ./

RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev --frozen --python /usr/bin/python3.12


# Runtime stage – no build tools, no CUDA headers.
FROM nvidia/cuda:12.1.0-cudnn8-runtime-ubuntu22.04

ARG SERVER_PORT=8765

ENV DEBIAN_FRONTEND=noninteractive \
    SERVER_PORT=${SERVER_PORT} \
    HF_HUB_CACHE=/app/models \
    PYTHONUNBUFFERED=1 \
    PATH="/app/.venv/bin:$PATH"

RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common gnupg \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3.12 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /app/.venv .venv

COPY backend/main.py .
COPY backend/config/ config/
COPY backend/src/ src/

EXPOSE ${SERVER_PORT}

CMD ["sh", "-c", "python main.py ${DEFAULT_MODEL:+--model $DEFAULT_MODEL}"]
