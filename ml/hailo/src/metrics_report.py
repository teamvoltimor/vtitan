"""Validated JSON transport form of eval.metrics.MetricsResult.

Host-only: ``eval/`` must stay free of third-party deps beyond numpy/PIL so
it runs unmodified inside the Hailo AI Software Suite container, which
doesn't guarantee pydantic is installed. This module lives in ``src/``
instead, which never ships to the container, for whichever host script needs
to move a MetricsResult across the docker exec boundary as JSON.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, field_serializer, field_validator

if TYPE_CHECKING:
    from eval.metrics import MetricsResult


class MetricsReport(BaseModel):
    """Validated, JSON-round-trippable mirror of :class:`eval.metrics.MetricsResult`.

    ``confusion``'s ``(true_class, predicted_class)`` tuple keys aren't valid
    JSON object keys, so they're encoded as ``"true:predicted"`` strings on
    the wire and decoded back to tuples on load.
    """

    model_config = ConfigDict(frozen=True)

    mAP50: float
    mAP75: float
    mAP50_95: float
    per_class: dict[int, float]
    per_class_5095: dict[int, float]
    confusion: dict[tuple[int, int], int]

    @field_serializer("confusion", when_used="json")
    def _serialize_confusion(self, value: dict[tuple[int, int], int]) -> dict[str, int]:
        return {f"{true_class}:{pred_class}": count for (true_class, pred_class), count in value.items()}

    @field_validator("confusion", mode="before")
    @classmethod
    def _parse_confusion(cls, value: object) -> object:
        if isinstance(value, dict) and all(isinstance(key, str) for key in value):
            return {tuple(int(part) for part in key.split(":")): count for key, count in value.items()}
        return value

    @classmethod
    def from_result(cls, result: MetricsResult) -> MetricsReport:
        """Build a validated report from a computed ``MetricsResult``."""
        return cls(
            mAP50=result.mAP50,
            mAP75=result.mAP75,
            mAP50_95=result.mAP50_95,
            per_class=result.per_class,
            per_class_5095=result.per_class_5095,
            confusion=result.confusion,
        )

    def to_result(self) -> MetricsResult:
        """Reconstruct the plain, container-safe ``MetricsResult`` for host-side use."""
        from eval.metrics import MetricsResult  # noqa: PLC0415

        return MetricsResult(
            mAP50=self.mAP50,
            mAP75=self.mAP75,
            mAP50_95=self.mAP50_95,
            per_class=self.per_class,
            per_class_5095=self.per_class_5095,
            confusion=self.confusion,
        )
