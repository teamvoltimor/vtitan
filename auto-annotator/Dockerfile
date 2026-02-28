FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

ARG SERVER_PORT=8765

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.9" \
    HF_HUB_CACHE=/app/models \
    SERVER_PORT=${SERVER_PORT}

RUN apt-get update && apt-get install -y \
    python3.12 \
    python3.12-dev \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install uv

WORKDIR /app

COPY backend/pyproject.toml .
RUN uv sync --no-dev

COPY backend/src/ src/
COPY backend/config/ config/
COPY backend/models/ models/
COPY backend/main.py .

EXPOSE ${SERVER_PORT}

CMD ["sh", "-c", "uv run python main.py server --model \"${DEFAULT_MODEL:-}\""]
