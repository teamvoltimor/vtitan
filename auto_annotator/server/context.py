"""server.context – Shared mutable server state (predictor, config)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ServerContext:
    """Holds all mutable state for the model server."""

    models_config: list[dict[str, Any]] = field(default_factory=list)
    predictor: Any = None          # SAM predictor instance
    text_seg: Any = None           # Text-segmenter instance (SAM3 only)
    model_id: str | None = None    # Currently active model id
    device: str = "cpu"
