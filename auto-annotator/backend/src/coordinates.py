"""Unified point and coordinate conversion with explicit coordinate spaces.

Defines three coordinate spaces:
- NormalizedCoords: (0.0 - 1.0), origin at top-left, used by frontend
- PixelCoords: (0 - width/height), origin at top-left, used by inference
- YOLOCoords: (0.0 - 1.0), normalized bounding box format for labels

Provides type-safe conversion functions between spaces.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NormalizedPoint:
    """Point in normalized space [0.0, 1.0], used by frontend.

    Origin at top-left. Values are already clamped to [0, 1].
    """

    x: float
    y: float

    def to_pixel(self, width: int, height: int) -> PixelPoint:
        """Convert to pixel coordinates given image dimensions, clamping to valid range."""
        x = max(0.0, min(1.0, self.x))
        y = max(0.0, min(1.0, self.y))
        return PixelPoint(x=int(x * (width - 1)), y=int(y * (height - 1)))

    def to_yolo(self) -> tuple[float, float]:
        """Convert to YOLO format (already normalized)."""
        return (self.x, self.y)


@dataclass(frozen=True)
class PixelPoint:
    """Point in pixel space, used by SAM model inference.

    Origin at top-left. Values are integers in range [0, width/height-1].
    """

    x: int
    y: int

    def to_normalized(self, width: int, height: int) -> NormalizedPoint:
        """Convert to normalized coordinates given image dimensions."""
        nx = self.x / (width - 1)
        ny = self.y / (height - 1)
        return NormalizedPoint(x=nx, y=ny)


@dataclass(frozen=True)
class YOLOPoint:
    """Point in YOLO format for label files.

    Normalized coordinates (0.0 - 1.0) for bounding boxes and polygons.
    """

    x: float
    y: float

    def to_normalized(self) -> NormalizedPoint:
        """Convert from YOLO to normalized (clamp to valid range)."""
        return NormalizedPoint(
            x=max(0.0, min(1.0, self.x)),
            y=max(0.0, min(1.0, self.y)),
        )

    def to_pixel(self, width: int, height: int) -> PixelPoint:
        """Convert from YOLO to pixel coordinates."""
        return self.to_normalized().to_pixel(width, height)


def yolo_coords_to_polygon(coords: list[float]) -> list[YOLOPoint]:
    """Parse a YOLO polygon (alternating x, y floats) into YOLOPoints.

    Args:
        coords: Flat list of [x1, y1, x2, y2, ...] YOLO coordinates.

    Returns:
        List of YOLOPoint objects.
    """
    return [YOLOPoint(x=coords[i], y=coords[i + 1]) for i in range(0, len(coords), 2)]


def yolo_bbox_to_corners(xc: float, yc: float, w: float, h: float) -> list[YOLOPoint]:
    """Convert YOLO bounding box center + size to four corner YOLOPoints.

    Args:
        xc: Box center X (normalized)
        yc: Box center Y (normalized)
        w: Box width (normalized)
        h: Box height (normalized)

    Returns:
        List of 4 YOLOPoints at corners: [TL, TR, BR, BL].
    """
    half_w = w / 2
    half_h = h / 2
    return [
        YOLOPoint(x=xc - half_w, y=yc - half_h),  # Top-left
        YOLOPoint(x=xc + half_w, y=yc - half_h),  # Top-right
        YOLOPoint(x=xc + half_w, y=yc + half_h),  # Bottom-right
        YOLOPoint(x=xc - half_w, y=yc + half_h),  # Bottom-left
    ]
