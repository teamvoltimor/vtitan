"""Configuration dataclasses for CLI commands and their handlers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

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
    """

    model: str
    calib: str | None
    shared_dir: str = SHARED_WITH_DOCKER


@dataclass(slots=True, frozen=True)
class CompileConfig:
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
    """

    model: str
    zoo_name: str | None
    hw: HWArch
    calib_path: str
    docker: str | None


@dataclass(slots=True, frozen=True)
class EvalConfig:
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

    model: str
    zoo_name: str | None
    har: str | None
    target: EvalTarget
    data_count: int
    visualize: bool
    docker: str | None


@dataclass(slots=True, frozen=True)
class ProfileConfig:
    """Parameters for ``hailomz profile``.

    Args:
        model: Registry key used to resolve the zoo name and default HEF path.
        zoo_name: Override the Hailo Model Zoo identifier.
        hef: HEF path *inside Docker*. Defaults to
            ``/local/shared_with_docker/<zoo_name>.hef``.
        docker: Docker container name. ``None`` → print command.
    """

    model: str
    zoo_name: str | None
    hef: str | None
    docker: str | None
