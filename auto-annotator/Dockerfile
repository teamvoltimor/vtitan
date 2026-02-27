FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

ARG SERVER_PORT=8765
ARG MODEL_SERVER_PORT=8765

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.9" \
    HF_HUB_CACHE=/app/models \
    SERVER_PORT=${SERVER_PORT} \
    MODEL_SERVER_PORT=${MODEL_SERVER_PORT}

RUN apt-get update && apt-get install -y \
    python3.12 \
    python3.12-dev \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install uv

WORKDIR /app

COPY pyproject.toml .
RUN uv sync --no-dev

COPY src/ src/
COPY server/ server/
COPY config/ config/
COPY app.py model_server.py ./

EXPOSE ${SERVER_PORT}

CMD ["uv", "run", "python", "app.py"]
