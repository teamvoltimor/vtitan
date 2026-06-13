"""src.server.context – Shared mutable server state (predictor, config)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ServerContext:
    """Holds all mutable state for the model server."""

    models_config: list[dict[str, Any]] = field(default_factory=list)
    predictor: Any = None
    text_seg: Any = None
    model_id: str | None = None
    device: str = "cpu"
