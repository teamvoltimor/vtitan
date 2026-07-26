"""Unit tests for src.docker's command construction."""

from __future__ import annotations

from src.docker import DOCKER_SHARED_MOUNT, DockerRunConfig, _compile_only_args, _gpu_args


def _config(**overrides: object) -> DockerRunConfig:
    base: dict[str, object] = {"shared_dir": "./shared_with_docker"}
    base.update(overrides)
    return DockerRunConfig(**base)  # type: ignore[arg-type]


def test_gpu_args_waive_the_images_stale_driver_requirement() -> None:
    # The 2025-10 image pins driver<471; without the waiver the NVIDIA runtime
    # refuses to start on any current driver.
    assert "NVIDIA_DISABLE_REQUIRE=1" in _gpu_args(_config())


def test_gpu_args_are_empty_when_gpu_disabled() -> None:
    assert _gpu_args(_config(gpu=False)) == []


def test_cuda_device_is_forwarded_when_set() -> None:
    args = _gpu_args(_config(cuda_device="0"))
    assert "CUDA_VISIBLE_DEVICES=0" in args


def test_cuda_device_is_omitted_when_unset() -> None:
    joined = " ".join(_gpu_args(_config()))
    assert "CUDA_VISIBLE_DEVICES" not in joined


def test_no_gpu_suppresses_cuda_device() -> None:
    # --no-gpu must win: pinning a device on a GPU-less run is meaningless.
    assert _gpu_args(_config(gpu=False, cuda_device="0")) == []


def test_compile_only_run_is_detached_and_stays_alive() -> None:
    args = _compile_only_args(_config(), "/host/shared")
    assert "-d" in args
    # The image's own command is an interactive shell, which exits immediately
    # when detached and would leave nothing for `docker exec` to attach to.
    assert args[-2:] == ["sleep", "infinity"]


def test_compile_only_run_mounts_only_the_shared_dir() -> None:
    args = _compile_only_args(_config(), "/host/shared")
    mounts = [args[i + 1] for i, a in enumerate(args) if a == "-v"]
    assert mounts == [f"/host/shared:{DOCKER_SHARED_MOUNT}:rw"]


def test_compile_only_run_omits_linux_only_flags() -> None:
    # These are exactly the flags that make the full run fail on Docker Desktop.
    joined = " ".join(_compile_only_args(_config(), "/host/shared"))
    for flag in ("--privileged", "/dev/dri", "/lib/modules", ".X11-unix", "--ipc=host"):
        assert flag not in joined
