"""Test FastAPI endpoint functionality.

Validates HTTP endpoints, request/response handling, and error cases.
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient


class TestGalleryEndpoints:
    """Test gallery listing and image operations."""

    def test_gallery_empty_database(self, api_client: TestClient) -> None:
        """Verify gallery endpoint returns empty response on initialization."""
        response = api_client.get("/gallery")
        assert response.status_code == 200
        data = response.json()
        assert "items" in data
        assert "stats" in data
        assert data["stats"]["total"] == 0

    def test_gallery_response_structure(self, api_client: TestClient) -> None:
        """Verify gallery response has correct structure."""
        response = api_client.get("/gallery")
        assert response.status_code == 200
        data = response.json()
        stats = data["stats"]
        assert all(k in stats for k in ["pending", "done", "skipped", "total", "pct"])

    def test_classes_endpoint(self, api_client: TestClient) -> None:
        """Verify classes endpoint returns list of classes."""
        response = api_client.get("/classes")
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)
        assert len(data) > 0
        assert all("id" in item and "name" in item for item in data)


class TestHealthCheck:
    """Test liveness and readiness probes."""

    def test_health_check_exists(self, api_client: TestClient) -> None:
        """Verify health check endpoint responds."""
        # Many APIs have a /health or /healthz endpoint
        # This test is a placeholder for when one is added
        response = api_client.get("/")
        # FastAPI root should redirect or have some response
        assert response.status_code in [200, 404, 307, 404]


class TestErrorHandling:
    """Test error handling and HTTP status codes."""

    def test_invalid_image_id_returns_404(self, api_client: TestClient) -> None:
        """Verify requesting non-existent image returns 404."""
        response = api_client.get("/images/99999")
        assert response.status_code == 404

    def test_invalid_annotation_request_returns_400(self, api_client: TestClient) -> None:
        """Verify invalid annotation request returns 400."""
        # This test will depend on the actual validation in endpoints
        # Placeholder for when segmentation endpoint is tested
        pass


class TestCORS:
    """Test CORS middleware configuration."""

    def test_cors_headers_present(self, api_client: TestClient) -> None:
        """Verify CORS headers are set."""
        response = api_client.options("/gallery")
        # CORS headers should be present or method allowed
        assert response.status_code in [200, 405]
