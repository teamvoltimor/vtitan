from pathlib import Path

import yaml


def load_class_map_from_yaml(yaml_path: str) -> dict[int, str]:
    """Load class map from YOLO data.yaml file."""
    path = Path(yaml_path)
    if path.exists():
        with Path.open(path) as f:
            data = yaml.safe_load(f)

        # The data.yaml file typically has a "names" field that maps class IDs to class names.
        names = data.get("names", {})
        return {int(k): v for k, v in names.items()}

    msg = f"data.yaml file not found at {yaml_path}"
    raise FileNotFoundError(msg)
