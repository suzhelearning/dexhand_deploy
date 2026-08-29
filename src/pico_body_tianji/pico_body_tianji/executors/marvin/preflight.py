"""受 launcher 授权的 Marvin real-capability provider。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

from ...sources.common.real_admission import RealCapabilityInput


def trusted_real_capability() -> RealCapabilityInput:
    """Read a protected, run-bound preflight attestation.

    Environment variables may select the file path and expected run identity;
    they cannot fabricate a passing result. Missing, foreign-owned, writable,
    malformed, or identity-mismatched attestations return a typed denial.
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
        expected_digest = os.environ.get("TIANJI_REAL_PREFLIGHT_DIGEST", "")
        if not expected_digest or hashlib.sha256(path.read_bytes()).hexdigest() != expected_digest:
            return denied
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict) or set(value) != {
            "run_id", "router_zid", "validation_supervisor_instance_id",
            "launcher_nonce", "capability"
        }:
            return denied
        if value["run_id"] != os.environ.get("TIANJI_RUN_ID", ""):
            return denied
        if value["router_zid"] != os.environ.get("TIANJI_ROUTER_ZID", ""):
            return denied
        if value["validation_supervisor_instance_id"] != os.environ.get(
            "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID", ""
        ):
            return denied
        if value["launcher_nonce"] != os.environ.get("TIANJI_REAL_PREFLIGHT_NONCE", ""):
            return denied
        return RealCapabilityInput.from_mapping(value["capability"])
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
        return denied


__all__ = ["trusted_real_capability"]
