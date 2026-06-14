# robot/libs — vendored wheels

Pre-built HailoRT Python wheels vendored here because they are not published to PyPI and require a Hailo Developer Zone account to download.

## Contents

| Platform | Python | File |
|---|---|---|
| `linux_aarch64` | 3.10 | `hailort-4.23.0-cp310-cp310-linux_aarch64.whl` |
| `linux_aarch64` | 3.12 | `hailort-4.23.0-cp312-cp312-linux_aarch64.whl` |
| `linux_x86_64` | 3.12 | `hailort-4.23.0-cp312-cp312-linux_x86_64.whl` |
| `win_amd64` | 3.12 | `hailort-4.23.0-cp312-cp312-win_amd64.whl` |

## Provenance

Downloaded from the [Hailo Developer Zone](https://hailo.ai/developer-zone/software-downloads/) — Software Downloads → HailoRT → Python API wheel.

Current version: **HailoRT 4.23.0**, targeting the **Hailo 8** NPU (M.2 / PCIe).

## Refresh procedure

1. Log in to the Hailo Developer Zone.
2. Download the wheel matching the target Python version and platform.
3. Replace the file in the corresponding subdirectory.
4. Update the version string in this README and in `pyproject.toml` (`hailort` dependency path).
5. Commit with message: `chore(libs): update HailoRT to x.y.z`.
