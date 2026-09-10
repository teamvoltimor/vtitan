#!/usr/bin/env python3
"""Copy eval/ to shared_with_docker/eval/ with progress output."""
import pathlib
import shutil
import sys

src = pathlib.Path("eval")
dst = pathlib.Path("shared_with_docker/eval")
dst.mkdir(parents=True, exist_ok=True)

copied = 0
for p in sorted(src.glob("*.py")):
    d = dst / p.name
    shutil.copy2(p, d)
    print(f"  copied {p.name}", file=sys.stderr)
    copied += 1

print(f"✓ synced {copied} files to shared_with_docker/eval", file=sys.stderr)
