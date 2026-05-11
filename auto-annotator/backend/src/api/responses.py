"""Unified response envelope for all API endpoints.

All endpoints return ApiResponse[T] for consistency.
Enables clients to handle errors uniformly.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
from typing import TypeVar

T = TypeVar("T")


@dataclass
class ApiResponse[T]:
    """Unified response envelope for all API operations.

    Provides consistent error handling, metadata, and status across endpoints.
    """

    data: T | None
    error: str | None = None
    metadata: dict | None = None

    @classmethod
    def success(cls, data: T, metadata: dict | None = None) -> ApiResponse[T]:
        """Create a successful response.

        Args:
            data: Response payload.
            metadata: Optional metadata (timestamp, request_id, etc.).

        Returns:
            ApiResponse with data and no error.
        """
        return cls(data=data, error=None, metadata=metadata or {"timestamp": datetime.now(UTC).isoformat()})

    @classmethod
    def error_response(cls, error: str, metadata: dict | None = None) -> ApiResponse[None]:
        """Create an error response.

        Args:
            error: Error message.
            metadata: Optional metadata.

        Returns:
            ApiResponse with error and no data.
        """
        return cls(data=None, error=error, metadata=metadata or {"timestamp": datetime.now(UTC).isoformat()})
