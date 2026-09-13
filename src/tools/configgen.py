"""Config schema tooling for the JSON Schemas under ``src/model``.

Commands:

* ``check``: for every TOML under ``src/config`` with a ``#:schema`` directive,
  verify each key has a described schema entry and every ``x-journal`` reference
  resolves.
* ``generate-go``: walk ``src/model`` and run go-jsonschema with a
  ``--schema-output``/``--schema-package`` pair per schema, mirroring the config
  tree into Go subpackages.
* ``scaffold``: infer a mirrored schema plus a directive from a TOML.
* ``fill``: add a placeholder description to any leaf that lacks one.

Python is used because the Task shell (gosh) has no JSON tooling and
go-jsonschema cannot map a directory to per-schema outputs on its own.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

Command = Literal["check", "generate-go", "scaffold", "fill"]

_SCALAR_TYPES = {bool: "boolean", int: "integer", float: "number", str: "string"}


class ConfigError(Exception):
    """Base error for config tooling failures."""


@dataclass(frozen=True, slots=True)
class GoOutput:
    """Where one schema's generated Go code goes.

    Attributes:
        schema_path: Source schema file.
        schema_id: The schema's ``$id``.
        path: Destination ``.gen.go`` file.
        package: Go package name for that file.
    """

    schema_path: Path
    schema_id: str
    path: Path
    package: str


# --- pure helpers ----------------------------------------------------------


def pascal(text: str) -> str:
    """Convert a snake/kebab path to PascalCase.

    Args:
        text: Identifier text, e.g. ``hardware_camera_config``.

    Returns:
        The PascalCase form, e.g. ``HardwareCameraConfig``.
    """
    return "".join(part.capitalize() for part in text.replace("-", "_").split("_") if part)


def humanize(key: str) -> str:
    """Turn a key into a placeholder sentence.

    Args:
        key: A leaf key such as ``text_thickness``.

    Returns:
        ``"Text thickness."``
    """
    words = key.replace("_", " ").strip()
    return (words[:1].upper() + words[1:]) + "."


def infer_type(value: object) -> dict:
    """Infer a minimal JSON Schema type for a TOML value.

    Args:
        value: Any value parsed from TOML.

    Returns:
        A JSON Schema fragment (``{"type": ...}``), with ``items`` for arrays.
    """
    for py_type, json_type in _SCALAR_TYPES.items():
        if isinstance(value, py_type):
            return {"type": json_type}
    if isinstance(value, list):
        return {"type": "array", "items": infer_type(value[0])} if value else {"type": "array"}
    if isinstance(value, dict):
        return {"type": "object"}
    return {}


def schema_leaves(node: dict, prefix: str = "") -> dict[str, dict]:
    """Map every leaf key path to its schema property.

    Args:
        node: A schema object node.
        prefix: Dotted path accumulated so far.

    Returns:
        Mapping of dotted key path to leaf property.
    """
    leaves: dict[str, dict] = {}
    for key, value in node.get("properties", {}).items():
        path = f"{prefix}.{key}" if prefix else key
        if value.get("type") == "object" and "properties" in value:
            leaves |= schema_leaves(value, path)
        else:
            leaves[path] = value
    return leaves


def toml_leaves(node: dict, prefix: str = "") -> dict[str, object]:
    """Map every leaf key path in a TOML document to its value.

    Args:
        node: A TOML mapping node.
        prefix: Dotted path accumulated so far.

    Returns:
        Mapping of dotted key path to value.
    """
    leaves: dict[str, object] = {}
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            leaves |= toml_leaves(value, path)
        else:
            leaves[path] = value
    return leaves


def schema_directive(toml_path: Path) -> Path | None:
    """Read the ``#:schema`` directive at the top of a TOML file.

    Args:
        toml_path: The TOML document.

    Returns:
        The resolved schema path, or ``None`` if the file has no directive.
    """
    for line in toml_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#:schema"):
            return (toml_path.parent / stripped[len("#:schema") :].strip()).resolve()
        if not stripped.startswith("#"):
            return None
    return None


def journal_refs(prop: dict) -> list[str]:
    """Normalize a property's ``x-journal`` to a list of references.

    Args:
        prop: A schema property.

    Returns:
        The references (empty when absent).
    """
    ref = prop.get("x-journal")
    if ref is None:
        return []
    return [ref] if isinstance(ref, str) else list(ref)


def descriptions_from_text(text: str) -> dict[str, str]:
    """Collect the comment block preceding each key or table in a TOML file.

    Args:
        text: The raw TOML text.

    Returns:
        Mapping of dotted path to the joined comment text.
    """
    descriptions: dict[str, str] = {}
    table = ""
    pending: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            pending = []
            continue
        if line.startswith("#"):
            pending.append(line.lstrip("#").strip())
            continue
        if line.startswith("[") and line.endswith("]"):
            table = line.strip("[]")
            if pending:
                descriptions[table] = " ".join(pending)
            pending = []
            continue
        if "=" in line:
            key = line.split("=", 1)[0].strip()
            path = f"{table}.{key}" if table else key
            parts = list(pending)
            if "#" in line:
                parts.append(line.split("#", 1)[1].strip())
            if parts:
                descriptions[path] = " ".join(parts)
            pending = []
    return descriptions


def schema_from_data(value: object, prefix: str, descriptions: dict[str, str]) -> dict:
    """Build a JSON Schema recursively from parsed TOML data.

    Args:
        value: A parsed TOML node.
        prefix: Dotted path of this node.
        descriptions: Comment text keyed by dotted path.

    Returns:
        The schema node.
    """
    if not isinstance(value, dict):
        schema = infer_type(value)
        leaf = prefix.rsplit(".", 1)[-1]
        schema["description"] = descriptions.get(prefix) or humanize(leaf)
        return schema

    properties = {
        key: schema_from_data(child, f"{prefix}.{key}" if prefix else key, descriptions)
        for key, child in value.items()
    }
    schema: dict = {
        "type": "object",
        "additionalProperties": False,
        "properties": properties,
        "required": list(properties),
    }
    if prefix in descriptions:
        schema["description"] = descriptions[prefix]
    return schema


def fill_node(node: dict, key_name: str | None = None) -> int:
    """Add a placeholder description to leaves that lack one, in place.

    Args:
        node: A schema node.
        key_name: The leaf key that owns this node, if any.

    Returns:
        How many descriptions were added.
    """
    if node.get("type") == "object" and "properties" in node:
        return sum(fill_node(child, key) for key, child in node["properties"].items())
    if key_name and not node.get("description"):
        node["description"] = humanize(key_name)
        return 1
    return 0


def go_package(relative_dir: Path) -> str:
    """Derive a Go package name from a schema's directory.

    Args:
        relative_dir: Directory of the schema relative to ``src/model``.

    Returns:
        ``"generated"`` for top-level schemas, else a sanitized directory name.
    """
    if relative_dir == Path():
        return "generated"
    name = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in relative_dir.name.lower())
    return name if name[:1].isalpha() else f"pkg_{name}"


# --- tool ------------------------------------------------------------------


class ConfigTool:
    """Config schema checks and generation for one repository.

    Attributes:
        repo_root: Repository root.
        config_root: ``src/config``.
        model_dir: ``src/model``.
        go_out_dir: Generated Go config package root.
    """

    def __init__(self, repo_root: Path) -> None:
        """Initialize the tool.

        Args:
            repo_root: Repository root used to resolve all paths.
        """
        self.repo_root = repo_root
        self.config_root = repo_root / "src" / "config"
        self.model_dir = repo_root / "src" / "model"
        self.go_out_dir = repo_root / "src" / "go" / "internal" / "config" / "generated"

    def schemas(self) -> list[Path]:
        """List every config schema under ``src/model``.

        Returns:
            Sorted schema paths, excluding dotfiles.
        """
        return sorted(p for p in self.model_dir.rglob("*.schema.json") if not p.name.startswith("."))

    def _check_coverage(self, toml_path: Path, schema_path: Path) -> list[str]:
        """Check that every TOML key is described in its schema.

        Args:
            toml_path: The TOML document.
            schema_path: Its schema.

        Returns:
            One message per problem.
        """
        rel = toml_path.relative_to(self.repo_root)
        if not schema_path.exists():
            return [f"{rel}: schema directive points at missing {schema_path}"]

        schema = schema_leaves(json.loads(schema_path.read_text(encoding="utf-8")))
        with toml_path.open("rb") as fh:
            keys = toml_leaves(tomllib.load(fh))

        failures: list[str] = []
        missing = sorted(set(keys) - set(schema))
        undocumented = sorted(p for p, prop in schema.items() if not prop.get("description"))
        if missing:
            failures.append(f"{rel}: keys with no schema entry: {missing}")
        if undocumented:
            failures.append(f"{schema_path.name}: properties with no description: {undocumented}")
        return failures

    def _check_refs(self, schema_path: Path) -> list[str]:
        """Check that every ``x-journal`` reference resolves.

        Args:
            schema_path: The schema to inspect.

        Returns:
            One message per unresolved reference.
        """
        failures: list[str] = []
        for path, prop in schema_leaves(json.loads(schema_path.read_text(encoding="utf-8"))).items():
            for ref in journal_refs(prop):
                if ref.startswith("adr:"):
                    target = self.repo_root / "docs" / "adr" / f"{ref[4:]}.md"
                else:
                    target = self.repo_root / ref.split("#", 1)[0]
                if not target.exists():
                    failures.append(f"{schema_path.name}: {path}: {ref} -> {target}")
        return failures

    def check(self) -> int:
        """Run the coverage and reference checks.

        Returns:
            Process exit code.
        """
        failures: list[str] = []
        seen: set[Path] = set()
        for toml_path in sorted(self.config_root.rglob("*.toml")):
            schema_path = schema_directive(toml_path)
            if schema_path is None:
                continue
            failures += self._check_coverage(toml_path, schema_path)
            if schema_path.exists():
                seen.add(schema_path)
        for schema_path in sorted(seen):
            failures += self._check_refs(schema_path)

        if failures:
            print("\n".join(failures))
            return 1
        print(f"config docs OK ({len(seen)} schemas)")
        return 0

    def _go_outputs(self) -> list[GoOutput]:
        """Build the per-schema Go output mapping.

        Returns:
            One entry per schema with a ``$id``.
        """
        outputs: list[GoOutput] = []
        for schema_path in self.schemas():
            spec = json.loads(schema_path.read_text(encoding="utf-8"))
            schema_id = spec.get("$id")
            if not schema_id:
                print(f"[warn] {schema_path.name} has no $id; skipped for Go")
                continue
            stem = schema_path.name.removesuffix(".schema.json")
            rel = schema_path.relative_to(self.model_dir)
            outputs.append(
                GoOutput(
                    schema_path,
                    schema_id,
                    self.go_out_dir / rel.parent / f"{stem}.gen.go",
                    go_package(rel.parent),
                )
            )
        return outputs

    def generate_go(self) -> int:
        """Generate one Go file per config, mirroring the schema tree.

        Returns:
            Process exit code.

        Raises:
            ConfigError: If go-jsonschema fails.
        """
        for stale in self.go_out_dir.rglob("*.gen.go"):
            stale.unlink()

        outputs = self._go_outputs()
        if not outputs:
            print("[warn] no schemas to generate")
            return 0

        args = [
            "go",
            "run",
            "github.com/atombender/go-jsonschema@latest",
            "--struct-name-from-title",
            "--only-models",
            "--package",
            "generated",
        ]
        inputs: list[str] = []
        for output in outputs:
            output.path.parent.mkdir(parents=True, exist_ok=True)
            # go-jsonschema runs url.Parse on paths, so Windows absolute paths
            # fail; pass repo-relative POSIX paths and run from the repo root.
            rel_out = output.path.relative_to(self.repo_root).as_posix()
            args += ["--schema-output", f"{output.schema_id}={rel_out}"]
            args += ["--schema-package", f"{output.schema_id}={output.package}"]
            inputs.append(output.schema_path.relative_to(self.repo_root).as_posix())

        try:
            subprocess.run(args + inputs, check=True, cwd=self.repo_root)
        except subprocess.CalledProcessError as exc:
            msg = f"go-jsonschema failed: {exc}"
            raise ConfigError(msg) from exc
        print(f"[OK] generated {len(outputs)} Go config files")
        return 0

    def scaffold(self, root: str) -> int:
        """Infer mirrored schemas and directives for TOMLs under a subtree.

        Args:
            root: Subtree under ``src/config`` (empty for the whole tree).

        Returns:
            Process exit code.
        """
        base = self.config_root / root if root else self.config_root
        created = 0
        for toml_path in sorted(base.rglob("*.toml")):
            rel = toml_path.relative_to(self.config_root)
            if "profiles" in rel.parts or schema_directive(toml_path) is not None:
                continue
            rel_schema = rel.with_suffix(".schema.json")
            out = self.model_dir / rel_schema
            if out.exists():
                continue

            with toml_path.open("rb") as fh:
                data = tomllib.load(fh)
            descriptions = descriptions_from_text(toml_path.read_text(encoding="utf-8"))
            schema: dict = {
                "$schema": "https://json-schema.org/draft/2020-12/schema",
                "$id": f"https://vtitan.local/schemas/{rel_schema.as_posix()}",
                "title": pascal(rel.with_suffix("").as_posix().replace("/", "_")),
                **schema_from_data(data, "", descriptions),
            }
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(schema, indent=2) + "\n", encoding="utf-8")

            directive = "../" * len(rel.parts) + "model/" + rel_schema.as_posix()
            toml_path.write_text(
                f"#:schema {directive}\n" + toml_path.read_text(encoding="utf-8"), encoding="utf-8"
            )
            created += 1
            print(f"[scaffold] {rel} -> {rel_schema}")
        print(f"[OK] scaffolded {created} schema(s)")
        return 0

    def fill(self) -> int:
        """Add placeholder descriptions to leaves that lack one.

        Returns:
            Process exit code.
        """
        total = 0
        for schema_path in self.schemas():
            spec = json.loads(schema_path.read_text(encoding="utf-8"))
            filled = fill_node(spec)
            if filled:
                schema_path.write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
                total += filled
        print(f"[OK] filled {total} placeholder description(s)")
        return 0

    def run(self, command: Command, root: str = "") -> int:
        """Dispatch a subcommand.

        Args:
            command: The subcommand name.
            root: Subtree for ``scaffold``.

        Returns:
            Process exit code.
        """
        if command == "scaffold":
            return self.scaffold(root)
        return {"check": self.check, "generate-go": self.generate_go, "fill": self.fill}[command]()


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser.

    Returns:
        The configured parser.
    """
    parser = argparse.ArgumentParser(prog="configgen")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="Check TOML/schema coverage and x-journal refs")
    sub.add_parser("generate-go", help="Generate one Go file per config")
    scaffold_parser = sub.add_parser("scaffold", help="Infer mirrored schemas + directives from TOML")
    scaffold_parser.add_argument("--root", default="", help="Subtree under src/config, e.g. hardware")
    sub.add_parser("fill", help="Add a placeholder description to any leaf that lacks one")
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point.

    Args:
        argv: Argument list (defaults to ``sys.argv``).

    Returns:
        Process exit code.
    """
    args = build_parser().parse_args(argv)
    tool = ConfigTool(Path(__file__).resolve().parents[2])
    try:
        return tool.run(args.command, getattr(args, "root", ""))
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
