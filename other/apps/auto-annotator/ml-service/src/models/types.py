"""src.types – Domain-specific type aliases for safety.

Provides :func:`typing.NewType` aliases for each primitive that represents a
distinct domain concept (class IDs, image IDs, model IDs, etc.) so the core
logic cannot accidentally mix them up.
"""

from typing import NewType

ClassId = NewType("ClassId", int)
"""Database row ID for annotation classes."""

ImageId = NewType("ImageId", int)
"""Database row ID for images."""

ModelId = NewType("ModelId", str)
"""Identifier string for selectable SAM models."""
