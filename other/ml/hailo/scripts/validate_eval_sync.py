#!/usr/bin/env python3
"""Check for drift between eval/ and shared_with_docker/eval/."""
import pathlib
import sys

src = pathlib.Path("eval")
dst = pathlib.Path("shared_with_docker/eval")
drifted = []

for p in sorted(src.glob("*.py")):
    d = dst / p.name
    if not d.exists():
        drifted.append(f"{p.name}: missing in container copy")
    elif p.read_text() != d.read_text():
        drifted.append(f"{p.name}: content mismatch")

if drifted:
    print("✗ Drift detected:", file=sys.stderr)
    for msg in drifted:
        print(f"  {msg}", file=sys.stderr)
    sys.exit(1)

print("✓ No drift detected", file=sys.stderr)
