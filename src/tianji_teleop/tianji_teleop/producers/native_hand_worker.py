"""Provenance and selection helpers for the optional complete Hand2 worker.

The native worker owns geometry, optimization, filtering, clamping and joint
permutation after startup.  This module is deliberately cold-path only: it
does not load a device SDK, spawn a worker, or publish a command.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def worker_path() -> Path:
    return ROOT / "build/hand-native/tianji_hand_native_worker"


def launcher_path() -> Path:
    return ROOT / "scripts/wuji_hand_native_launcher.py"


def recording_metadata(environment):
    """Return portable native-worker provenance, or ``None`` for Python.

    Selection is explicit and fail-closed.  A missing executable is reported
    before the recorder is opened, so a recording cannot claim a backend that
    was never available.
    """

    backend = environment.get("TIANJI_HAND_WORKER_BACKEND", "python")
    if backend not in ("python", "cpp"):
        raise ValueError("TIANJI_HAND_WORKER_BACKEND must be python or cpp")
    if backend == "python":
        return None

    worker = worker_path()
    launcher = launcher_path()
    optimizer = ROOT / "build/hand-native/libtianji_hand_optimizer.so"
    for path, label in ((worker, "native hand worker"),
                        (launcher, "native hand launcher"),
                        (optimizer, "native hand optimizer library")):
        if not path.is_file():
            raise RuntimeError(f"{label} missing; run pixi run build-native-hand-worker")
    if not worker.stat().st_mode & 0o111:
        raise RuntimeError("native hand worker is not executable")

    sources = {
        "worker": ROOT / "native/hand/worker.cpp",
        "geometry": ROOT / "native/hand/geometry.cpp",
        "filter": ROOT / "native/hand/lowpass.cpp",
        "optimizer": ROOT / "native/hand/optimizer.cpp",
        "launcher": launcher,
    }
    return {
        "backend": "cpp",
        "abi": 1,
        "protocol": "TJWI/TJHR",
        # <4sBBHQQII126d>: 8-byte flags/size header, 16-byte timing
        # envelope, 8-byte count/reserved fields, and 126 float64 points.
        "request_size_bytes": 1040,
        "response_size_bytes": 344,
        "worker_sha256": hashlib.sha256(worker.read_bytes()).hexdigest(),
        "optimizer_library_sha256": hashlib.sha256(optimizer.read_bytes()).hexdigest(),
        "source_sha256": {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in sources.items()
        },
    }
