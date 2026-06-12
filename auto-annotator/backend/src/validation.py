"""Validation gate for all request validation.

Single source of truth for input validation. Services assume
validated input; routers call validators before passing to services.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from src.api.schemas import SaveAnnotationsRequest, SegmentationPoint
    from src.db.repository import Repository
    from src.models import ClassInfo


class Validator(Protocol):
    """Interface for request validation."""

    def validate_segmentation_request(
        self,
        image_id: int,
        points: list[SegmentationPoint],
        classes: list[ClassInfo],
    ) -> None:
        """Validate segmentation request.

        Args:
            image_id: Image database ID.
            points: Click points.
            classes: Available classes.

        Raises:
            ValueError: If validation fails.
        """
        ...

    def validate_save_annotations_request(
        self,
        payload: SaveAnnotationsRequest,
        classes: list[ClassInfo],
    ) -> None:
        """Validate save annotations request.

        Args:
            payload: Save request.
            classes: Available classes.

        Raises:
            ValueError: If validation fails.
        """
        ...


class DefaultValidator:
    """Default validator implementation."""

    def __init__(self, repository: Repository):
        """Initialize validator with repository for class lookup.

        Args:
            repository: For checking if classes exist.
        """
        self.repository = repository

    def validate_segmentation_request(
        self,
        image_id: int,
        points: list[SegmentationPoint],
        classes: list[ClassInfo],
    ) -> None:
        """Validate segmentation request.

        Checks:
        - Image ID exists
        - At least one point provided
        - All classes referenced in points exist
        """
        if self.repository.images.get_by_id(image_id) is None:
            msg = f"Image {image_id} not found"
            raise ValueError(msg)

        if not points:
            msg = "At least one point required"
            raise ValueError(msg)

        class_names = {cls.name for cls in classes}
        for point in points:
            if point.class_name not in class_names:
                msg = f"Unknown class '{point.class_name}'"
                raise ValueError(msg)

    def validate_save_annotations_request(
        self,
        payload: SaveAnnotationsRequest,
        classes: list[ClassInfo],
    ) -> None:
        """Validate save annotations request.

        Checks:
        - At least one shape provided
        - Primary class exists
        - All referenced classes exist
        """
        if not payload.shapes:
            msg = "At least one shape required"
            raise ValueError(msg)

        class_names = {cls.name for cls in classes}
        primary_class = payload.shapes[0].class_name

        if primary_class not in class_names:
            msg = f"Unknown primary class '{primary_class}'"
            raise ValueError(msg)

        for shape in payload.shapes:
            if shape.class_name not in class_names:
                msg = f"Unknown class '{shape.class_name}'"
                raise ValueError(msg)
