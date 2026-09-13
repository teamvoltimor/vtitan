"""Base class for generated config DTOs.

Generated models must be immutable and reject unknown keys. datamodel-codegen
emits plain ``BaseModel`` subclasses, so the strictness lives here and is wired
in with ``--base-class shared.config.codegen_base.StrictModel``.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class StrictModel(BaseModel):
    """Frozen Pydantic base with unknown keys rejected."""

    model_config = ConfigDict(frozen=True, extra="forbid")
