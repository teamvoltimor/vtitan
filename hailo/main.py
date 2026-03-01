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

from src.common import (
    DOCKER_CONTAINER,
    DOCKER_IMAGE,
    DOCKER_SHARED_MOUNT,
    MODEL_REGISTRY,
    SHARED_WITH_DOCKER,
    Backend,
    HWArch,
    ModelName,
    Task,
    configure_logging,
    get_logger,
)
from src import calib, export, graph, hailomz, test

log = get_logger(__name__)


# Command adapters

def _cmd_export(args: argparse.Namespace) -> None:
    export.run(
        export.ExportConfig(
            model=ModelName(args.model),
            imgsz=args.imgsz,
            opset=args.opset,
            no_simplify=args.no_simplify,
        )
    )


def _cmd_calib(args: argparse.Namespace) -> None:
    if args.calib_cmd == "download":
        calib.download(
            calib.DownloadConfig(samples=args.samples, output=args.output)
        )
    else:
        calib.convert(
            calib.ConvertConfig(input=args.input, output=args.output, size=args.size)
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
        )
    )


def _cmd_stage(args: argparse.Namespace) -> None:
    hailomz.stage(
        hailomz.StageConfig(
            model=args.model,
            calib=args.calib,
            shared_dir=args.shared_dir,
        )
    )


def _cmd_compile(args: argparse.Namespace) -> None:
    hailomz.compile_model(
        hailomz.CompileConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            hw=HWArch(args.hw),
            calib_path=args.calib_path,
            docker=args.docker,
        )
    )


def _cmd_eval(args: argparse.Namespace) -> None:
    hailomz.eval_model(
        hailomz.EvalConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            har=args.har,
            target=args.target,
            data_count=args.data_count,
            visualize=args.visualize,
            docker=args.docker,
        )
    )


def _cmd_profile(args: argparse.Namespace) -> None:
    hailomz.profile_model(
        hailomz.ProfileConfig(
            model=args.model,
            zoo_name=args.zoo_name,
            hef=args.hef,
            docker=args.docker,
        )
    )


def _cmd_docker(args: argparse.Namespace) -> None:
    if args.docker_cmd == "run":
        hailomz.docker_run(
            hailomz.DockerRunConfig(
                shared_dir=args.shared_dir,
                container=args.container,
                display=args.display,
                dry_run=args.dry_run,
            )
        )


# Argument parser

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

    _docker_args = argparse.ArgumentParser(add_help=False)
    _docker_args.add_argument(
        "--docker",
        default=None,
        metavar="CONTAINER",
        help="Docker container name for `docker exec`. Omit to print the command instead.",
    )
    _docker_args.add_argument(
        "--zoo-name",
        default=None,
        metavar="ZOO_NAME",
        help="Override the Hailo Model Zoo identifier (e.g. yolov11s). "
             "Required when the model has no zoo_name in the registry.",
    )

    # export
    p_export = sub.add_parser("export", help="Export a YOLO .pt checkpoint to ONNX")
    p_export.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
        help=f"Model variant — one of: {list(MODEL_REGISTRY)}",
    )
    p_export.add_argument("--imgsz", type=int, default=640)
    p_export.add_argument("--opset", type=int, default=None, help="Override ONNX opset")
    p_export.add_argument("--no-simplify", action="store_true")
    p_export.set_defaults(func=_cmd_export)

    # calib
    p_calib = sub.add_parser("calib", help="Calibration data management")
    calib_sub = p_calib.add_subparsers(dest="calib_cmd", required=True)

    p_dl = calib_sub.add_parser("download", help="Download COCO 2017 validation images")
    p_dl.add_argument("--samples", type=int, default=2048)
    p_dl.add_argument("--output", default="./calib_data")

    p_cv = calib_sub.add_parser(
        "convert",
        help="Convert images to float32 .npy (HWC, [0,1]) for hailomz --calib-path",
    )
    p_cv.add_argument("--input", default="./calib_data")
    p_cv.add_argument("--output", default="./calib_data_npy")
    p_cv.add_argument("--size", type=int, default=640)

    p_calib.set_defaults(func=_cmd_calib)

    # inspect
    p_inspect = sub.add_parser("inspect", help="Print ONNX graph, inputs, and outputs")
    p_inspect.add_argument("--model", required=True, help="Path to .onnx file")
    p_inspect.set_defaults(func=_cmd_inspect)

    # test
    p_test = sub.add_parser(
        "test", help="Run inference on images and save annotated results"
    )
    p_test.add_argument("--model", required=True, help="Path to .pt or .onnx model")
    p_test.add_argument(
        "--backend",
        required=True,
        choices=[b.value for b in Backend],
        help="pt | onnx | ultraonnx",
    )
    p_test.add_argument(
        "--task",
        choices=[t.value for t in Task],
        default=None,
        help='Override task (inferred from filename by default — "seg" → segment)',
    )
    p_test.add_argument("--input", default="./calib_data")
    p_test.add_argument("--output", default="./test_output")
    p_test.add_argument("--conf", type=float, default=0.3)
    p_test.set_defaults(func=_cmd_test)

    # stage
    p_stage = sub.add_parser(
        "stage",
        help=(
            f"Copy model ONNX [+ calibration data] into {SHARED_WITH_DOCKER}/ "
            f"(mounted at {DOCKER_SHARED_MOUNT}/ inside Docker)"
        ),
    )
    p_stage.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
        help=f"Model to stage — one of: {list(MODEL_REGISTRY)}",
    )
    p_stage.add_argument(
        "--calib",
        default=None,
        metavar="DIR",
        help="Local calibration image directory to copy (optional)",
    )
    p_stage.add_argument(
        "--shared-dir",
        default=SHARED_WITH_DOCKER,
        help=f"Host path of the Docker-shared volume (default: {SHARED_WITH_DOCKER})",
    )
    p_stage.set_defaults(func=_cmd_stage)

    # compile  (hailomz compile inside Docker)
    p_compile = sub.add_parser(
        "compile",
        parents=[_docker_args],
        help="Run `hailomz compile` inside the Hailo AI Software Suite container",
    )
    p_compile.add_argument(
        "--model",
        required=True,
        choices=list(MODEL_REGISTRY),
        metavar="MODEL",
    )
    p_compile.add_argument(
        "--hw",
        default=HWArch.HAILO8.value,
        choices=[h.value for h in HWArch],
    )
    p_compile.add_argument(
        "--calib-path",
        default=f"{DOCKER_SHARED_MOUNT}/calib_data",
        help=f"Calibration path inside Docker (default: {DOCKER_SHARED_MOUNT}/calib_data)",
    )
    p_compile.set_defaults(func=_cmd_compile)

    # eval  (hailomz eval inside Docker)
    p_eval = sub.add_parser(
        "eval",
        parents=[_docker_args],
        help="Run `hailomz eval` inside the Hailo AI Software Suite container",
    )
    p_eval.add_argument("--model", required=True, choices=list(MODEL_REGISTRY), metavar="MODEL")
    p_eval.add_argument(
        "--har",
        default=None,
        help="HAR path inside Docker (default: /local/shared_with_docker/<zoo_name>.har)",
    )
    p_eval.add_argument(
        "--target",
        default="emulator",
        choices=["emulator", "hailo8"],
        help="Evaluation target (default: emulator)",
    )
    p_eval.add_argument("--data-count", type=int, default=512)
    p_eval.add_argument("--visualize", action="store_true")
    p_eval.set_defaults(func=_cmd_eval)

    # profile  (hailomz profile inside Docker)
    p_profile = sub.add_parser(
        "profile",
        parents=[_docker_args],
        help="Run `hailomz profile` inside the Hailo AI Software Suite container",
    )
    p_profile.add_argument("--model", required=True, choices=list(MODEL_REGISTRY), metavar="MODEL")
    p_profile.add_argument(
        "--hef",
        default=None,
        help="HEF path inside Docker (default: /local/shared_with_docker/<zoo_name>.hef)",
    )
    p_profile.set_defaults(func=_cmd_profile)

    # docker
    p_docker = sub.add_parser(
        "docker",
        help="Manage the Hailo AI Software Suite container",
    )
    docker_sub = p_docker.add_subparsers(dest="docker_cmd", required=True)

    p_docker_run = docker_sub.add_parser(
        "run",
        help=f"Start the {DOCKER_IMAGE} container with all required mounts",
    )
    p_docker_run.add_argument(
        "--shared-dir",
        default=SHARED_WITH_DOCKER,
        help=f"Host path to mount as {DOCKER_SHARED_MOUNT} (default: {SHARED_WITH_DOCKER})",
    )
    p_docker_run.add_argument(
        "--container",
        default=DOCKER_CONTAINER,
        help=f"Container name (default: {DOCKER_CONTAINER})",
    )
    p_docker_run.add_argument(
        "--display",
        default=":0",
        help="X11 DISPLAY to forward (default: :0)",
    )
    p_docker_run.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the docker run command without executing it",
    )

    p_docker.set_defaults(func=_cmd_docker)

    return parser


def main() -> None:
    """CLI entry point registered in pyproject.toml."""
    parser = _build_parser()
    args = parser.parse_args()
    configure_logging(args.log_level)

    try:
        args.func(args)
    except Exception as exc:
        log.error("%s: %s", type(exc).__name__, exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
