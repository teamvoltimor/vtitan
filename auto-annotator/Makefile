# Makefile – Auto-Annotator convenience targets
#
# Usage: make <target>

.PHONY: run model-server lint format docker-build docker-run docker-dev docker-up docker-upd docker-down help

API_PORT ?= 8000

# Local dev

run:
	cd backend && ./scripts/start_backend.sh

model-server:
	cd backend && uv run python main.py --model $${MODEL_ID}

# Code quality

lint:
	cd backend && uv run ruff check .

format:
	cd backend && uv run ruff check --fix . && uv run ruff format .

# Docker

docker-build:
	docker build -t auto-annotator .

docker-run:
	docker run --gpus all -p ${API_PORT}:${API_PORT} auto-annotator

docker-dev:
	docker run --gpus all -p ${API_PORT}:${API_PORT} \
	  -v "$(CURDIR)/backend/data:/app/data" \
	  -v "$(CURDIR)/backend/models:/app/models" \
	  auto-annotator

docker-up:
	docker-compose up --build

docker-upd:
	docker-compose up -d --build

docker-down:
	docker-compose down

# Help


help:
	@echo "Auto-Annotator Makefile targets:"
	@echo "  run            Start the SAM stack (SAM server + FastAPI via backend/scripts/start_backend.sh)"
	@echo "  model-server   Start the SAM server only (uv run python main.py --model $${MODEL_ID})"
	@echo "  lint           Run ruff linter"
	@echo "  format         Run ruff fix + format"
	@echo "  docker-build   Build Docker image"
	@echo "  docker-run     Run Docker container (GPU)"
	@echo "  docker-dev     Run with data/models volume mounts"
	@echo "  docker-up      docker-compose up"
	@echo "  docker-upd     docker-compose up -d"
	@echo "  docker-down    docker-compose down"
