"""Loading generated scenario metadata, shared by every diag script that runs scenarios.

The same three lines -- ``json.loads(path.read_text())`` then
``ScenarioMetadata.model_validate(...)``, over a ``glob("*_metadata.json")``
corpus -- were copy-pasted into fourteen scripts. The loader is one function so
a schema change lands in one place, and so the filename convention lives next
to the code that depends on it rather than being retyped per script.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from shared.domain.models import ScenarioMetadata

if TYPE_CHECKING:
    from pathlib import Path

METADATA_GLOB = "*_metadata.json"
"""Filename suffix the Go simgen corpus generator emits for scenario metadata."""


def scenario_paths(directory: Path) -> list[Path]:
    """The generated scenario metadata files under ``directory``, name-sorted.

    Sorted because every corpus sweep reports cases in this order and a
    different iteration order would silently relabel each case's index.
    """
    return sorted(directory.glob(METADATA_GLOB))


def load_scenario(path: Path) -> ScenarioMetadata:
    """Parse one ``*_metadata.json`` fixture into its typed model."""
    return ScenarioMetadata.model_validate_json(path.read_text())


def scenario_from_mapping(raw: object) -> ScenarioMetadata:
    """Typed model from an already-decoded metadata mapping (e.g. ``Scenario.metadata``)."""
    return ScenarioMetadata.model_validate(raw)
