FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

ARG SERVER_PORT=8765

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.9" \
    HF_HUB_CACHE=/app/models \
    SERVER_PORT=${SERVER_PORT}

RUN apt-get update && apt-get install -y \
    software-properties-common \
    gnupg \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:deadsnakes/ppa

RUN apt-get update && apt-get install -y \
    python3.12 \
    python3.12-dev \
    python3-distutils \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py \
    && python3.12 /tmp/get-pip.py --force-reinstall \
    && rm -f /tmp/get-pip.py

RUN python3.12 -m pip install --upgrade pip uv
RUN ln -sf /usr/bin/python3.12 /usr/bin/python \
    && ln -sf /usr/bin/python3.12 /usr/bin/python3

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
