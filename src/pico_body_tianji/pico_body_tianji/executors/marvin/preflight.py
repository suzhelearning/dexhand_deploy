"""受 launcher 授权的 Marvin real-capability provider。"""
from __future__ import annotations

import json
import os
from pathlib import Path

from ...sources.common.real_admission import RealCapabilityInput


def trusted_real_capability() -> RealCapabilityInput:
    """Read an operator/preflight attestation from a protected regular file.

    Environment variables may select the file path only; they cannot fabricate
    a passing result.  Missing, foreign-owned, writable, malformed, or
    incomplete attestations return a typed denied capability.
    """
    denied = RealCapabilityInput(1.0, 0.0, False, False)
    raw_path = os.environ.get("TIANJI_REAL_PREFLIGHT_FILE", "")
    if not raw_path:
        return denied
    path = Path(raw_path).expanduser()
    try:
        stat = path.stat()
        if path.is_symlink() or stat.st_uid != os.getuid() or stat.st_mode & 0o022:
            return denied
        value = json.loads(path.read_text(encoding="utf-8"))
        return RealCapabilityInput.from_mapping(value)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
        return denied


__all__ = ["trusted_real_capability"]
