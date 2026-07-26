"""Hailo YOLO pipeline CLI entry point.

Host-side commands
------------------
    uv run hailo export        --model yolo11s
    uv run hailo calib         download
    uv run hailo calib         convert
    uv run hailo inspect       --model data/yolo11s.onnx
    uv run hailo test          --model data/yolo11s.onnx --backend onnx
    uv run hailo stage         --model yolo11s [--calib ./calib_data]
    uv run hailo docker run    [--dry-run]

Docker-side commands (dispatched via docker exec, or printed for manual use)
-----------------------------------------------------------------------------
    uv run hailo compile  --model yolo11s [--docker hailo8_ai_sw_suite_2025-10_container]
    uv run hailo eval     --model yolo11s [--docker hailo8_ai_sw_suite_2025-10_container]
    uv run hailo profile  --model yolo11s [--docker hailo8_ai_sw_suite_2025-10_container]
"""

from __future__ import annotations

import argparse
import sys

from src import calib, export, graph, hailomz, test
from src.constants import (
    DEFAULT_CALIB_INPUT,
    DEFAULT_CALIB_OUTPUT,
    DEFAULT_COCO_SAMPLES,
    DEFAULT_CONFIDENCE,
    DEFAULT_IMG_SIZE,
)
from src.docker import (
    DOCKER_SHARED_MOUNT,
    DockerRunConfig,
    docker_run,
)
from src.enums import Backend, EvalTarget, HWArch, Task
from src.errors import HailoError
from src.log import configure_logging, get_logger
from src.registry import MODEL_REGISTRY, SHARED_WITH_DOCKER, ModelName
from src.settings import HailoSettings, load_settings

log = get_logger(__name__)


# Command adapters


def _cmd_export(args: argparse.Namespace) -> None:
    export.run(
        export.ExportConfig(
            model=ModelName(args.model),
            imgsz=args.imgsz,
            opset=args.opset,
            no_simplify=args.no_simplify,
        ),
    )


def _cmd_calib(args: argparse.Namespace) -> None:
    if args.calib_cmd == "download":
        calib.download(
            calib.DownloadConfig(samples=args.samples, output=args.output),
        )
    else:
        calib.convert(
            calib.ConvertConfig(input=args.input, output=args.output, size=args.size),
        )


def _cmd_inspect(args: argparse.Namespace) -> None:
    graph.inspect(args.model)


def _cmd_test(args: argparse.Namespace) -> None:
    test.run(
        test.TestConfig(
            model=args.model,
            backend=Backend(args.backend),
            task=Task(args.task) if args.task else None,
            input=args.input,
            output=args.output,
            conf=args.conf,
        ),
    )


def _cmd_stage(args: argparse.Namespace) -> None:
    hailomz.stage(
        hailomz.StageConfig(
            model=args.model,
            calib=args.calib,
            shared_dir=args.shared_dir,
            calib_name=args.calib_name,
        ),
    )


def _cmd_compile(args: argparse.Namespace) -> None:
    hailomz.compile_model(
        hailomz.CompileConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            hw=HWArch(args.hw),
            calib_path=args.calib_path,
            docker=args.docker,
            classes=args.classes,
            model_script=args.model_script,
            performance=args.performance,
        ),
    )


def _cmd_eval(args: argparse.Namespace) -> None:
    hailomz.eval_model(
        hailomz.EvalConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            har=args.har,
            target=EvalTarget(args.target),
            data_count=args.data_count,
            visualize=args.visualize,
            docker=args.docker,
        ),
    )


def _cmd_profile(args: argparse.Namespace) -> None:
    hailomz.profile_model(
        hailomz.ProfileConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            hef=args.hef,
            docker=args.docker,
        ),
    )


def _cmd_docker(args: argparse.Namespace) -> None:
    if args.docker_cmd == "run":
        docker_run(
            DockerRunConfig(
                shared_dir=args.shared_dir,
                container=args.container,
                image=args.image,
                host_uid=args.host_uid,
                video_gid=args.video_gid,
                display=args.display,
                dry_run=args.dry_run,
                compile_only=args.compile_only,
                gpu=not args.no_gpu,
                cuda_device=args.cuda_device,
            ),
        )


def _make_docker_parent() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--docker",
        default=None,
        metavar="CONTAINER",
        help="Docker container name for `docker exec`. Omit to print the command instead.",
    )
    parser.add_argument(
        "--zoo-name",
        default=None,
        metavar="ZOO_NAME",
        help=(
            "Override the Hailo Model Zoo identifier (e.g. yolov11s). "
            "Required when the model has no zoo_name in the registry."
        ),
    )
    return parser


def _add_export_parser(sub: argparse._SubParsersAction) -> None:
    parser = sub.add_parser("export", help="Export a YOLO .pt checkpoint to ONNX")
    parser.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
        help=f"Model variant — one of: {list(MODEL_REGISTRY)}",
    )
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMG_SIZE)
    parser.add_argument("--opset", type=int, default=None, help="Override ONNX opset")
    parser.add_argument("--no-simplify", action="store_true")
    parser.set_defaults(func=_cmd_export)


def _add_calib_parser(sub: argparse._SubParsersAction) -> None:
    parser = sub.add_parser("calib", help="Calibration data management")
    calib_sub = parser.add_subparsers(dest="calib_cmd", required=True)

    p_dl = calib_sub.add_parser("download", help="Download COCO 2017 validation images")
    p_dl.add_argument("--samples", type=int, default=DEFAULT_COCO_SAMPLES)
    p_dl.add_argument("--output", default=DEFAULT_CALIB_INPUT)

    p_cv = calib_sub.add_parser(
        "convert",
        help="Convert images to float32 .npy (HWC, [0,1]) for hailomz --calib-path",
    )
    p_cv.add_argument("--input", default=DEFAULT_CALIB_INPUT)
    p_cv.add_argument("--output", default=DEFAULT_CALIB_OUTPUT)
    p_cv.add_argument("--size", type=int, default=DEFAULT_IMG_SIZE)

    parser.set_defaults(func=_cmd_calib)


def _add_inspect_parser(sub: argparse._SubParsersAction) -> None:
    parser = sub.add_parser("inspect", help="Print ONNX graph, inputs, and outputs")
    parser.add_argument("--model", required=True, help="Path to .onnx file")
    parser.set_defaults(func=_cmd_inspect)


def _add_test_parser(sub: argparse._SubParsersAction) -> None:
    parser = sub.add_parser("test", help="Run inference on images and save annotated results")
    parser.add_argument("--model", required=True, help="Path to .pt or .onnx model")
    parser.add_argument(
        "--backend",
        required=True,
        choices=[b.value for b in Backend],
        help="pt | onnx | ultraonnx  (onnx requires an NMS-embedded model; "
        "use ultraonnx for the registered nms=False exports)",
    )
    parser.add_argument(
        "--task",
        choices=[t.value for t in Task],
        default=None,
        help='Override task (inferred from filename by default — "seg" → segment)',
    )
    parser.add_argument("--input", default=DEFAULT_CALIB_INPUT)
    parser.add_argument("--output", default="./test_output")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    parser.set_defaults(func=_cmd_test)


def _add_stage_parser(sub: argparse._SubParsersAction) -> None:
    parser = sub.add_parser(
        "stage",
        help=(
            f"Copy model ONNX [+ calibration data] into {SHARED_WITH_DOCKER}/ "
            f"(mounted at {DOCKER_SHARED_MOUNT}/ inside Docker)"
        ),
    )
    parser.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
        help=f"Model to stage — one of: {list(MODEL_REGISTRY)}",
    )
    parser.add_argument(
        "--calib",
        default=None,
        metavar="DIR",
        help="Local calibration image directory to copy (optional, searched recursively)",
    )
    parser.add_argument(
        "--shared-dir",
        default=SHARED_WITH_DOCKER,
        help=f"Host path of the Docker-shared volume (default: {SHARED_WITH_DOCKER})",
    )
    parser.add_argument(
        "--calib-name",
        default="calib_data",
        metavar="NAME",
        help="Subdirectory to stage calibration images into (default: calib_data)",
    )
    parser.set_defaults(func=_cmd_stage)


def _add_compile_parser(
    sub: argparse._SubParsersAction,
    docker_parent: argparse.ArgumentParser,
) -> None:
    parser = sub.add_parser(
        "compile",
        parents=[docker_parent],
        help="Run `hailomz compile` inside the Hailo AI Software Suite container",
    )
    parser.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
    )
    parser.add_argument(
        "--hw",
        default=HWArch.HAILO8.value,
        choices=[h.value for h in HWArch],
    )
    parser.add_argument(
        "--calib-path",
        default=f"{DOCKER_SHARED_MOUNT}/calib_data",
        help=f"Calibration path inside Docker (default: {DOCKER_SHARED_MOUNT}/calib_data)",
    )
    parser.add_argument(
        "--classes",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Detection class count for a retrained checkpoint. Defaults to the "
            "registry entry's class count, or the zoo model's when unset."
        ),
    )
    quality = parser.add_mutually_exclusive_group()
    quality.add_argument(
        "--model-script",
        default=None,
        metavar="PATH",
        help=(
            "Path inside Docker to an .alls model script, replacing the zoo's own. "
            "Use to raise calibset_size above the SDK's 64-entry default; the script "
            "must restate normalization, output activations and nms_postprocess."
        ),
    )
    quality.add_argument(
        "--performance",
        action="store_true",
        help="Compile at the highest optimization level (much slower; needs a GPU)",
    )
    parser.set_defaults(func=_cmd_compile)


def _add_eval_parser(
    sub: argparse._SubParsersAction,
    docker_parent: argparse.ArgumentParser,
) -> None:
    parser = sub.add_parser(
        "eval",
        parents=[docker_parent],
        help="Run `hailomz eval` inside the Hailo AI Software Suite container",
    )
    parser.add_argument("--model", required=True, choices=list(MODEL_REGISTRY), metavar="MODEL")
    parser.add_argument(
        "--har",
        default=None,
        help="HAR path inside Docker (default: /local/shared_with_docker/<zoo_name>.har)",
    )
    parser.add_argument(
        "--target",
        default=EvalTarget.EMULATOR.value,
        choices=[t.value for t in EvalTarget],
        help="Evaluation target (default: emulator)",
    )
    parser.add_argument("--data-count", type=int, default=512)
    parser.add_argument("--visualize", action="store_true")
    parser.set_defaults(func=_cmd_eval)


def _add_profile_parser(
    sub: argparse._SubParsersAction,
    docker_parent: argparse.ArgumentParser,
) -> None:
    parser = sub.add_parser(
        "profile",
        parents=[docker_parent],
        help="Run `hailomz profile` inside the Hailo AI Software Suite container",
    )
    parser.add_argument("--model", required=True, choices=list(MODEL_REGISTRY), metavar="MODEL")
    parser.add_argument(
        "--hef",
        default=None,
        help="HEF path inside Docker (default: /local/shared_with_docker/<zoo_name>.hef)",
    )
    parser.set_defaults(func=_cmd_profile)


def _add_docker_parser(sub: argparse._SubParsersAction, settings: HailoSettings) -> None:
    parser = sub.add_parser("docker", help="Manage the Hailo AI Software Suite container")
    docker_sub = parser.add_subparsers(dest="docker_cmd", required=True)

    p_docker_run = docker_sub.add_parser(
        "run",
        help=f"Start the {settings.docker_image} container with all required mounts",
    )
    p_docker_run.add_argument(
        "--shared-dir",
        default=SHARED_WITH_DOCKER,
        help=f"Host path to mount as {DOCKER_SHARED_MOUNT} (default: {SHARED_WITH_DOCKER})",
    )
    p_docker_run.add_argument(
        "--container",
        default=settings.docker_container,
        help=f"Container name (default: {settings.docker_container}, override via HAILO_DOCKER_CONTAINER)",
    )
    p_docker_run.add_argument(
        "--image",
        default=settings.docker_image,
        help=f"Suite image repo:tag to launch (default: {settings.docker_image}, override via HAILO_DOCKER_IMAGE)",
    )
    p_docker_run.add_argument(
        "--host-uid",
        type=int,
        default=settings.host_uid,
        help=f"Host UID for XDG_RUNTIME_DIR (default: {settings.host_uid}, override via HAILO_HOST_UID)",
    )
    p_docker_run.add_argument(
        "--video-gid",
        type=int,
        default=settings.video_gid,
        help=f"Host video group GID for --group-add (default: {settings.video_gid}, override via HAILO_VIDEO_GID)",
    )
    p_docker_run.add_argument(
        "--display",
        default=settings.x11_display,
        help=f"X11 DISPLAY to forward (default: {settings.x11_display}, override via HAILO_X11_DISPLAY)",
    )
    p_docker_run.add_argument(
        "--compile-only",
        action="store_true",
        help=(
            "Start a minimal detached container with only the shared mount. "
            "Required on Docker Desktop, where the Linux device and X11 mounts "
            "do not exist. Sufficient for export/stage/compile; `eval --target "
            "hailo8` still needs the full Linux run for NPU access."
        ),
    )
    p_docker_run.add_argument(
        "--no-gpu",
        action="store_true",
        help="Drop --gpus all, for hosts without an NVIDIA GPU",
    )
    p_docker_run.add_argument(
        "--cuda-device",
        default=None,
        metavar="INDEX",
        help=(
            "Pin CUDA_VISIBLE_DEVICES. Without it the optimizer's GPU selector "
            "skips any GPU over 5%% utilised -- which a laptop GPU driving a "
            "desktop always is -- and silently drops to optimization level 0."
        ),
    )
    p_docker_run.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the docker run command without executing it",
    )

    parser.set_defaults(func=_cmd_docker)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hailo",
        description="Hailo YOLO pipeline: export → calib → inspect → test → stage → compile → eval → profile",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    sub = parser.add_subparsers(dest="command", required=True)

    settings = load_settings()
    docker_parent = _make_docker_parent()
    _add_export_parser(sub)
    _add_calib_parser(sub)
    _add_inspect_parser(sub)
    _add_test_parser(sub)
    _add_stage_parser(sub)
    _add_compile_parser(sub, docker_parent)
    _add_eval_parser(sub, docker_parent)
    _add_profile_parser(sub, docker_parent)
    _add_docker_parser(sub, settings)

    return parser


def main() -> None:
    """CLI entry point registered in pyproject.toml."""
    parser = _build_parser()
    args = parser.parse_args()
    configure_logging(args.log_level)

    try:
        args.func(args)
    except HailoError as exc:
        log.exception("%s", type(exc).__name__)
        sys.exit(1)


if __name__ == "__main__":
    main()
