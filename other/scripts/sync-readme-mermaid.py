#!/usr/bin/env python3
"""Keep the README's inlined Mermaid blocks identical to their .mmd sources.

The diagrams live in schemes/flowcharts/*/mermaid/*.mmd, which is also what
`task docs:diagrams` renders to WebP and what GitHub renders when the .mmd is
opened directly. The README inlines a copy so a judge sees the diagram without
leaving the page; this script regenerates those copies so the .mmd stays the
single source of truth.

Each block is anchored by an HTML comment naming its source:

    <!-- mermaid-src: schemes/flowcharts/open/mermaid/maquina-estados.mmd -->
    ```mermaid
    ...
    ```

%% lines are dropped on the way in. They are maintenance notes (code
references, sweep numbers) that never render and do not belong in the
document the judges read.

Usage:
    python other/scripts/sync-readme-mermaid.py            # rewrite
    python other/scripts/sync-readme-mermaid.py --check    # exit 1 if stale
"""

import argparse
import io
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
README = REPO / "README.md"

BLOCK = re.compile(
    r"(?P<anchor><!-- mermaid-src: (?P<src>[^\s]+?\.mmd) -->\n)"
    r"```mermaid\n(?P<body>.*?)\n```",
    re.S,
)


def render(src: pathlib.Path) -> str:
    """Return the .mmd body with maintenance comments and blank edges removed."""
    lines = [l for l in src.read_text(encoding="utf-8").split("\n")
             if not l.lstrip().startswith("%%")]
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="report stale blocks and exit 1 instead of rewriting")
    args = parser.parse_args()

    text = README.read_text(encoding="utf-8")
    stale: list[str] = []
    missing: list[str] = []

    def replace(match: re.Match[str]) -> str:
        src = REPO / match.group("src")
        if not src.exists():
            missing.append(match.group("src"))
            return match.group(0)
        body = render(src)
        if body != match.group("body"):
            stale.append(match.group("src"))
        return f"{match.group('anchor')}```mermaid\n{body}\n```"

    rebuilt, count = BLOCK.subn(replace, text)

    if missing:
        for src in missing:
            print(f"source missing: {src}", file=sys.stderr)
        return 2

    if args.check:
        for src in stale:
            print(f"stale: {src}", file=sys.stderr)
        print(f"{count} blocks checked, {len(stale)} stale")
        return 1 if stale else 0

    if stale:
        README.write_text(rebuilt, encoding="utf-8", newline="\n")
    print(f"{count} blocks checked, {len(stale)} updated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
