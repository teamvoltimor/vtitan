import os
import re

platform_dir = r"C:\Users\ralva\OneDrive\Documents\private\projects\archived\teamsteelbot\klevor-v2-platform\platform"

for root, dirs, files in os.walk(platform_dir):
    if "shared" in root.split(os.sep) or "site-packages" in root or ".venv" in root:
        continue
    for file in files:
        if file.endswith(".py"):
            path = os.path.join(root, file)
            try:
                with open(path, "r", encoding="utf-8") as f:
                    content = f.read()

                if (
                    "from shared.config.constants" in content
                    or "from shared.config.enums" in content
                ):
                    content = content.replace(
                        "from shared.config.constants", "from shared.config.constants"
                    )
                    content = content.replace(
                        "from shared.config.enums", "from shared.config.enums"
                    )
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(content)
                    print(f"Updated {path}")
            except Exception as e:
                print(f"Skipping {path}: {e}")
