"""Configuration dataclasses for CLI commands and their handlers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.constants import DEFAULT_CALIB_NAME
from src.registry import SHARED_WITH_DOCKER, ModelName

if TYPE_CHECKING:
    from src.enums import Backend, EvalTarget, HWArch, Task


@dataclass(slots=True, frozen=True)
class ExportConfig:
    """Parameters for a single model export run.

    Args:
        model: Registry key identifying the model variant.
        imgsz: Square input resolution passed to ``YOLO.export()``.
        opset: Override the per-model default ONNX opset when set.
        no_simplify: When ``True``, suppress the ``simplify=True`` flag
            even if the model registry requests it.
    """

    model: ModelName
    imgsz: int
    opset: int | None
    no_simplify: bool


@dataclass(slots=True, frozen=True)
class DownloadConfig:
    """Parameters for the COCO calibration dataset download.

    Args:
        samples: Number of images to pull from the COCO 2017 validation split.
        output: Destination directory for the raw image files.
    """

    samples: int
    output: str


@dataclass(slots=True, frozen=True)
class ConvertConfig:
    """Parameters for converting raw images to float32 ``.npy`` arrays.

    Args:
        input: Directory containing ``.jpg``/``.png`` calibration images.
        output: Directory where ``.npy`` files will be written.
        size: Square resize target matching the model's input resolution.
    """

    input: str
    output: str
    size: int


@dataclass(slots=True, frozen=True)
class TestConfig:
    """Parameters for an inference test run.

    Args:
        model: Path to the ``.pt`` or ``.onnx`` model file.
        backend: Which inference engine to use.
        task: Detection or segmentation. Inferred from filename when ``None``.
        input: Directory of test images.
        output: Directory where annotated images are written.
        conf: Confidence threshold; detections below this are discarded.
    """

    model: str
    backend: Backend
    task: Task | None
    input: str
    output: str
    conf: float


@dataclass(slots=True, frozen=True)
class StageConfig:
    """Parameters for staging files into ``shared_with_docker/``.

    Args:
        model: Registry key of the model whose ONNX should be staged.
        calib: Local calibration image directory to copy across.
            Pass ``None`` to skip copying calibration data.
        shared_dir: Host path of the Docker-shared volume
            (default: ``shared_with_docker``).
        calib_name: Subdirectory name for the staged calibration images.
            Give a per-model name to keep domain-specific calibration sets
            from overwriting each other.
        labels: YOLO label directory to flatten alongside the images, so the
            evaluators can score inside the container (which mounts nothing
            else). Pass ``None`` to skip.
        labels_name: Subdirectory name for the staged labels.
    """

    model: str
    calib: str | None
    shared_dir: str = SHARED_WITH_DOCKER
    calib_name: str = DEFAULT_CALIB_NAME
    labels: str | None = None
    labels_name: str = "calib_labels"


@dataclass(slots=True, frozen=True)
class HailoMZConfig:
    """Shared fields for Hailo Model Zoo commands (compile / eval / profile).

    Args:
        model: Registry key used to resolve the zoo name and related artifacts.
        zoo_name: Override the Hailo Model Zoo identifier.
        docker: Docker container name for ``docker exec``. When ``None`` the
            command is printed instead of executed.
    """

    model: str
    zoo_name: str | None
    docker: str | None


@dataclass(slots=True, frozen=True)
class CompileConfig(HailoMZConfig):
    """Parameters for ``hailomz compile``.

    Args:
        model: Registry key used to resolve the zoo name and ONNX filename.
        zoo_name: Override the Hailo Model Zoo identifier. Required when the
            model has no ``zoo_name`` entry in the registry.
        hw: Target hardware architecture.
        calib_path: Calibration data path *inside Docker*
            (default: ``/local/shared_with_docker/calib_data``).
        docker: Docker container name for ``docker exec``.
            When ``None`` the command is printed instead of executed.
        classes: Detection class count for a retrained checkpoint. Falls back
            to the registry entry's ``classes`` when ``None``; when both are
            ``None`` the zoo model's own class count is used.
        model_script: Path *inside Docker* to an ``.alls`` model script. This
            replaces the zoo's own script rather than extending it, so a custom
            script must restate the model's normalization, output activations
            and ``nms_postprocess`` or the compiled HEF will be wrong. Its main
            use is raising ``calibset_size`` above the SDK's 64-entry default.
        performance: Emit ``--performance`` to compile at the highest
            optimization level. Mutually exclusive with ``model_script``.
    """

    hw: HWArch
    calib_path: str
    classes: int | None = None
    model_script: str | None = None
    performance: bool = False


@dataclass(slots=True, frozen=True)
class EvalConfig(HailoMZConfig):
    """Parameters for ``hailomz eval``.

    Args:
        model: Registry key used to resolve the zoo name and default HAR path.
        zoo_name: Override the Hailo Model Zoo identifier.
        har: HAR path *inside Docker*. Defaults to
            ``/local/shared_with_docker/<zoo_name>.har``.
        target: Evaluation target — ``"emulator"`` or ``"hailo8"``.
        data_count: Number of samples to evaluate.
        visualize: Emit ``--visualize`` flag.
        docker: Docker container name. ``None`` → print command.
    """

    har: str | None
    target: EvalTarget
    data_count: int
    visualize: bool


@dataclass(slots=True, frozen=True)
class ProfileConfig(HailoMZConfig):
    """Parameters for ``hailomz profile``.

    Args:
        model: Registry key used to resolve the zoo name and default HEF path.
        zoo_name: Override the Hailo Model Zoo identifier.
        hef: HEF path *inside Docker*. Defaults to
            ``/local/shared_with_docker/<zoo_name>.hef``.
        docker: Docker container name. ``None`` → print command.
    """

    hef: str | None
