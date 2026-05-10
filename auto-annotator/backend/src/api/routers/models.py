"""SAM model management endpoint."""

from __future__ import annotations

import tomllib

from fastapi import APIRouter

from src.api.schemas import ModelItem
from src.constants import CONFIG_FILE

router = APIRouter()


@router.get("", response_model=list[ModelItem])
def list_models() -> list[ModelItem]:
    """Return all configured models from models.toml."""
    if not CONFIG_FILE.exists():
        return []
    with CONFIG_FILE.open("rb") as fp:
        config = tomllib.load(fp)
    return [ModelItem(id=m["id"], label=m.get("label", m["id"])) for m in config.get("models", [])]
