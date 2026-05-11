"""Annotation class management endpoints."""

from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import APIRouter

from src.api.schemas import ClassItem, UpsertClassRequest

if TYPE_CHECKING:
    from src.api.dependencies import RepositoryDep

router = APIRouter()


@router.get("", response_model=list[ClassItem])
def list_classes(repository: RepositoryDep) -> list[ClassItem]:
    """Return all annotation classes ordered by id."""
    return [ClassItem(id=cls.id, name=cls.name, color=cls.color) for cls in repository.classes.get_all()]


@router.post("", response_model=list[ClassItem])
def upsert_class(payload: UpsertClassRequest, repository: RepositoryDep) -> list[ClassItem]:
    """Insert or update a class by name and return the full updated list."""
    repository.classes.upsert(payload.name, payload.color)
    return [ClassItem(id=cls.id, name=cls.name, color=cls.color) for cls in repository.classes.get_all()]
