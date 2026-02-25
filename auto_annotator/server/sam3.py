"""server.sam3 – SAM 3 model loading and inference helpers."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from server.context import ServerContext


# ── Interactive predictor (adapts Sam3TrackerModel to SAM2 interface) ──────────


class SAM3Predictor:
    """Wrap Sam3TrackerModel as a SAM2ImagePredictor-compatible object."""

    def __init__(self, model, processor) -> None:
        self.model = model
        self.processor = processor
        self._pil = None
        self._orig_hw: tuple[int, int] | None = None

    def set_image(self, image: np.ndarray) -> None:
        from PIL import Image as _PIL  # noqa: PLC0415

        self._pil = _PIL.fromarray(image)
        self._orig_hw = image.shape[:2]

    def predict(
        self,
        point_coords,
        point_labels,
        mask_input=None,
        multimask_output: bool = True,
    ):
        import torch  # noqa: PLC0415

        if self._pil is None:
            msg = "set_image() must be called before predict()"
            raise RuntimeError(msg)

        input_points = [[[[float(c[0]), float(c[1])] for c in point_coords]]]
        input_labels = [[[int(l) for l in point_labels]]]

        proc_kwargs = dict(
            images=self._pil,
            input_points=input_points,
            input_labels=input_labels,
            return_tensors="pt",
        )
        if mask_input is not None:
            try:
                proc_kwargs["input_masks"] = [[mask_input]]
            except Exception:  # noqa: BLE001
                pass

        inputs = self.processor(**proc_kwargs).to(self.model.device)

        with torch.inference_mode():
            outputs = self.model(**inputs)

        try:
            masks_t = self.processor.post_process_masks(
                outputs.pred_masks,
                inputs["original_sizes"],
                inputs["reshaped_input_sizes"],
            )[0]
        except Exception:  # noqa: BLE001
            import torch.nn.functional as _F  # noqa: PLC0415, N812

            H, W = self._orig_hw
            raw = outputs.pred_masks[0, :, 0:1].float()
            masks_t = (_F.interpolate(raw, (H, W), mode="bilinear")[:, 0] > 0)

        if masks_t.ndim == 4:
            masks_t = masks_t[:, 0]

        masks_np = masks_t.cpu().bool().numpy()

        if hasattr(outputs, "iou_scores"):
            scores_np = outputs.iou_scores[0].flatten().cpu().float().numpy()
        elif hasattr(outputs, "pred_scores"):
            scores_np = outputs.pred_scores[0].flatten().cpu().float().numpy()
        else:
            scores_np = np.ones(len(masks_np), dtype=np.float32)

        logits = outputs.pred_masks[0].cpu().numpy() if hasattr(outputs, "pred_masks") else None
        return masks_np, scores_np, logits


# ── Text segmenter ─────────────────────────────────────────────────────────────


class SAM3TextSegmenter:
    """Use Sam3Model for text-prompted auto-annotation."""

    def __init__(self, hf_repo: str, device: str) -> None:
        try:
            from transformers import Sam3Model, Sam3Processor  # type: ignore  # noqa: PLC0415

            self.processor = Sam3Processor.from_pretrained(hf_repo)
            self.model = Sam3Model.from_pretrained(hf_repo).to(device)
        except (ImportError, AttributeError):
            from transformers import AutoModel, AutoProcessor  # type: ignore  # noqa: PLC0415

            self.processor = AutoProcessor.from_pretrained(hf_repo)
            self.model = AutoModel.from_pretrained(hf_repo).to(device)
        self.device = device

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        import torch  # noqa: PLC0415
        from PIL import Image as _PIL  # noqa: PLC0415

        pil = _PIL.fromarray(image)
        H, W = image.shape[:2]
        results = []

        for class_name in class_names:
            try:
                inputs = self.processor(
                    images=pil,
                    text=[class_name],
                    return_tensors="pt",
                ).to(self.device)

                with torch.inference_mode():
                    outputs = self.model(**inputs)

                try:
                    masks_t = self.processor.post_process_masks(
                        outputs.pred_masks,
                        inputs["original_sizes"],
                        inputs["reshaped_input_sizes"],
                    )[0]
                except Exception:  # noqa: BLE001
                    import torch.nn.functional as _F  # noqa: PLC0415, N812

                    raw = outputs.pred_masks[0, :, 0:1].float()
                    masks_t = (_F.interpolate(raw, (H, W), mode="bilinear")[:, 0] > 0)

                if masks_t.ndim == 4:
                    masks_t = masks_t[:, 0]
                masks_np = masks_t.cpu().bool().numpy()

                scores = (
                    outputs.iou_scores[0].flatten().cpu().float().tolist()
                    if hasattr(outputs, "iou_scores")
                    else [1.0] * len(masks_np)
                )

                boxes: list = []
                if hasattr(outputs, "pred_boxes"):
                    for box in outputs.pred_boxes[0].cpu().float().numpy():
                        cx, cy, bw, bh = box * np.array([W, H, W, H])
                        boxes.append(
                            [
                                float(cx - bw / 2),
                                float(cy - bh / 2),
                                float(cx + bw / 2),
                                float(cy + bh / 2),
                            ]
                        )

                results.append(
                    {
                        "class_name": class_name,
                        "masks": [masks_np[i] for i in range(len(masks_np))],
                        "scores": scores,
                        "boxes": boxes,
                    }
                )
            except Exception as e:  # noqa: BLE001
                results.append(
                    {
                        "class_name": class_name,
                        "masks": [],
                        "scores": [],
                        "boxes": [],
                        "error": str(e),
                    }
                )

        return results


# ── Loader entry point ─────────────────────────────────────────────────────────


def load_sam3(cfg: dict, ctx: "ServerContext") -> None:
    """Load SAM 3 interactive + optional text segmenter into *ctx*."""
    try:
        from transformers import Sam3TrackerModel, Sam3TrackerProcessor  # type: ignore  # noqa: PLC0415

        tracker = Sam3TrackerModel.from_pretrained(cfg["hf_repo"]).to(ctx.device)
        tracker_proc = Sam3TrackerProcessor.from_pretrained(cfg["hf_repo"])
    except (ImportError, AttributeError):
        from transformers import AutoModel, AutoProcessor  # type: ignore  # noqa: PLC0415

        tracker = AutoModel.from_pretrained(cfg["hf_repo"]).to(ctx.device)
        tracker_proc = AutoProcessor.from_pretrained(cfg["hf_repo"])

    ctx.predictor = SAM3Predictor(tracker, tracker_proc)
    ctx.text_seg = None

    if cfg.get("supports_text"):
        try:
            ctx.text_seg = SAM3TextSegmenter(cfg["hf_repo"], ctx.device)
        except Exception:  # noqa: BLE001
            pass
