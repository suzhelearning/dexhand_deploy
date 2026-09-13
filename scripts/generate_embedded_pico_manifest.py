#!/usr/bin/env python3
"""Generate a deterministic manifest for the embedded legacy PICO source."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path, PurePosixPath
from typing import Iterable


EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    ".pixi",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "recordings",
    "runtime",
}


def _source_files(source_root: Path, output: Path) -> Iterable[tuple[str, Path]]:
    output_resolved = output.resolve()
    for path in sorted(source_root.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix in {".pyc", ".pyo"}:
            continue
        relative = path.relative_to(source_root)
        if any(part in EXCLUDED_DIRECTORY_NAMES for part in relative.parts):
            continue
        if path.resolve() == output_resolved or relative.as_posix() == "source_manifest.json":
            continue
        relative_posix = PurePosixPath(*relative.parts).as_posix()
        if PurePosixPath(relative_posix).is_absolute() or ".." in PurePosixPath(relative_posix).parts:
            raise ValueError(f"source file path escapes source root: {relative_posix}")
        yield relative_posix, path


def _git_commit(source_root: Path) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(source_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return "embedded-source"
    commit = completed.stdout.strip()
    return commit or "embedded-source"


def build_manifest(source_root: Path, output: Path, source_commit: str | None = None) -> dict:
    source_root = source_root.resolve()
    output = output.resolve()
    if not source_root.is_dir():
        raise ValueError(f"source root is not a directory: {source_root}")
    try:
        output.relative_to(source_root)
    except ValueError as exc:
        raise ValueError("manifest output must be inside source root") from exc

    files = []
    for relative, path in _source_files(source_root, output):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        files.append({"path": relative, "sha256": digest})

    return {
        "schema_version": 1,
        "source_commit": source_commit or _git_commit(source_root),
        "files": files,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--source-commit")
    args = parser.parse_args(argv)

    source_root = args.source_root.resolve()
    output = (args.output or source_root / "source_manifest.json").resolve()
    manifest = build_manifest(source_root, output, args.source_commit)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2, sort_keys=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
