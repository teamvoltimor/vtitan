# Makefile – Auto-Annotator convenience targets
#
# Usage: make <target>

.PHONY: run model-server lint format docker-build docker-run docker-dev \
        docker-up docker-upd docker-down help

# Local dev

run:
	uv run python main.py app

model-server:
	uv run python main.py server

# Code quality

lint:
	uv run ruff check .

format:
	uv run ruff check --fix . && uv run ruff format .

# Docker

docker-build:
	docker build -t auto-annotator .

docker-run:
	docker run --gpus all -p 7860:7860 auto-annotator

docker-dev:
	docker run --gpus all -p 7860:7860 \
	  -v "$(CURDIR)/data:/app/data" \
	  -v "$(CURDIR)/models:/app/models" \
	  auto-annotator

docker-up:
	docker-compose up

docker-upd:
	docker-compose up -d

docker-down:
	docker-compose down

# Help

help:
	@echo "Auto-Annotator Makefile targets:"
	@echo "  run            Start the Gradio app (uv run python main.py app)"
	@echo "  model-server   Start the SAM model server (uv run python main.py server)"
	@echo "  lint           Run ruff linter"
	@echo "  format         Run ruff fix + format"
	@echo "  docker-build   Build Docker image"
	@echo "  docker-run     Run Docker container (GPU)"
	@echo "  docker-dev     Run with data/models volume mounts"
	@echo "  docker-up      docker-compose up"
	@echo "  docker-upd     docker-compose up -d"
	@echo "  docker-down    docker-compose down"
