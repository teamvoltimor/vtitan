# WRO 2026 Platform — project automation
# Requires: UV (Python), Pixi (ROS2 Kilted via RoboStack)

.PHONY: help install generate generate-open generate-obstacles generate-track \
        navigate record convert-bags extract-frames \
        test lint clean

# ── Defaults ──────────────────────────────────────────────────────────────────

SCENARIOS      ?= 10
CHALLENGE      ?= open
OUTPUT_DIR     ?= ../training_data
BASE_WORLD     ?= ./worlds/wro_track_2026.sdf
METADATA       ?= $(OUTPUT_DIR)/$(CHALLENGE)/scenarios/scenario_0000_metadata.json
LAPS           ?= 3
PARAMS         ?= ./scripts/navigator_params.json

# ── Help ──────────────────────────────────────────────────────────────────────

help:
	@echo ""
	@echo "  WRO 2026 Platform"
	@echo ""
	@echo "  Setup:"
	@echo "    make install              Install Python deps (UV) + ROS2 env (Pixi)"
	@echo ""
	@echo "  Track:"
	@echo "    make generate-track       Re-generate worlds/wro_track_2026.sdf"
	@echo ""
	@echo "  Scenario generation:"
	@echo "    make generate             Generate SCENARIOS (default: 10) open scenarios"
	@echo "    make generate-open        Generate open-challenge scenarios"
	@echo "    make generate-obstacles   Generate obstacles-challenge scenarios"
	@echo ""
	@echo "  Navigation:"
	@echo "    make navigate             Run navigator on METADATA (default: scenario_0000)"
	@echo ""
	@echo "  Recording:"
	@echo "    make record               Record scenario videos (requires Gazebo + ROS2)"
	@echo "    make convert-bags         Convert ROS2 bags to MP4 videos"
	@echo "    make extract-frames       Extract frames from bags and annotate"
	@echo ""
	@echo "  Testing:"
	@echo "    make test                 Run all tests via UV"
	@echo "    make lint                 Run ruff linter via UV"
	@echo ""
	@echo "  Cleanup:"
	@echo "    make clean                Remove generated training data"
	@echo ""
	@echo "  Variables (override with make VAR=value):"
	@echo "    SCENARIOS=$(SCENARIOS)  CHALLENGE=$(CHALLENGE)  LAPS=$(LAPS)"
	@echo "    OUTPUT_DIR=$(OUTPUT_DIR)"
	@echo "    METADATA=$(METADATA)"
	@echo ""

# ── Setup ─────────────────────────────────────────────────────────────────────

install:
	uv sync
	pixi install

# ── Track generation ───────────────────────────────────────────────────────────

generate-track:
	uv run python -m src.generation.track_generator --output $(BASE_WORLD)

# ── Scenario generation ────────────────────────────────────────────────────────

generate: generate-open

generate-open:
	uv run python -m src.generation.generator \
		--challenge open \
		--num-scenarios $(SCENARIOS) \
		--output-dir $(OUTPUT_DIR) \
		--base-world $(BASE_WORLD) \
		--randomize-all

generate-obstacles:
	uv run python -m src.generation.generator \
		--challenge obstacles \
		--num-scenarios $(SCENARIOS) \
		--output-dir $(OUTPUT_DIR) \
		--base-world $(BASE_WORLD) \
		--randomize-all

# ── Navigation ────────────────────────────────────────────────────────────────

navigate:
	pixi run python -m src.navigation.navigator \
		--metadata $(METADATA) \
		--laps $(LAPS) \
		--params $(PARAMS)

# ── Recording ─────────────────────────────────────────────────────────────────

record:
	pixi run python -m src.recording.recorder \
		--output-dir $(OUTPUT_DIR)/$(CHALLENGE)/videos

convert-bags:
	uv run python -m src.recording.bag_converter \
		--input-dir $(OUTPUT_DIR)/$(CHALLENGE)/bags \
		--output-dir $(OUTPUT_DIR)/$(CHALLENGE)/videos

extract-frames:
	uv run python -m src.recording.frame_extractor \
		--input-dir $(OUTPUT_DIR)/$(CHALLENGE)/bags \
		--output-dir $(OUTPUT_DIR)/$(CHALLENGE)/frames

# ── Testing & linting ─────────────────────────────────────────────────────────

test:
	uv run pytest tests/ -v

lint:
	uv run ruff check src/ tests/

# ── Cleanup ───────────────────────────────────────────────────────────────────

clean:
	rm -rf $(OUTPUT_DIR)
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true
