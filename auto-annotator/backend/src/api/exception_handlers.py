"""RFC 7807 Problem Details exception handlers for FastAPI."""

from __future__ import annotations

import uuid
from http import HTTPStatus
from typing import TYPE_CHECKING

from fastapi.responses import JSONResponse

if TYPE_CHECKING:
    from fastapi import Request
    from fastapi.exceptions import RequestValidationError
    from starlette.exceptions import HTTPException as StarletteHTTPException


def _problem(status: int, detail: object, request: Request, *, title: str | None = None) -> JSONResponse:
    try:
        phrase = HTTPStatus(status).phrase
    except ValueError:
        phrase = "Error"
    return JSONResponse(
        status_code=status,
        media_type="application/problem+json",
        content={
            "type": "about:blank",
            "title": title or phrase,
            "status": status,
            "detail": detail,
            "instance": request.url.path,
            "correlation_id": str(uuid.uuid4()),
        },
    )


async def http_exception_handler(request: Request, exc: StarletteHTTPException) -> JSONResponse:
    """Handle HTTP exceptions as RFC 7807 Problem Details."""
    return _problem(exc.status_code, exc.detail, request)


async def validation_exception_handler(request: Request, exc: RequestValidationError) -> JSONResponse:
    """Handle request validation errors as RFC 7807 Problem Details."""
    return _problem(422, exc.errors(), request, title="Validation Error")
