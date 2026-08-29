"""受 launcher 授权的 Marvin real-capability provider。"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
import stat
from typing import Any, Mapping

import yaml

from ...sources.common.real_admission import RealCapabilityInput


_SEAL_GET = getattr(fcntl, "F_GET_SEALS", 1034)
_SEAL_ALL = (
    getattr(fcntl, "F_SEAL_SEAL", 1)
    | getattr(fcntl, "F_SEAL_SHRINK", 2)
    | getattr(fcntl, "F_SEAL_GROW", 4)
    | getattr(fcntl, "F_SEAL_WRITE", 8)
)


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _read_fd(fd: int) -> bytes:
    descriptor = os.fstat(fd)
    size = int(descriptor.st_size)
    if size < 0:
        raise ValueError("invalid attestation size")
    payload = os.pread(fd, size, 0)
    if len(payload) != size:
        raise ValueError("short attestation read")
    return payload


def _scanner_capability(raw: bytes) -> tuple[str, RealCapabilityInput]:
    value = yaml.safe_load(raw.decode("utf-8"))
    if not isinstance(value, Mapping) or set(value) != {"scanner_id", "capability"}:
        raise ValueError("invalid scanner attestation")
    scanner_id = str(value["scanner_id"]).strip()
    if not scanner_id:
        raise ValueError("scanner identity is empty")
    return scanner_id, RealCapabilityInput.from_mapping(value["capability"])


def trusted_real_capability() -> RealCapabilityInput:
    """Read one sealed run-bound payload and its protected scanner source.

    The provider never opens a writable bundle path.  Hashing, integrity
    validation, and JSON parsing all operate on the same ``pread`` bytes from
    a sealed memfd.  The scanner descriptor is independently checked as a
    root-owned protected regular file and its bytes/capability must match the
    bound payload.
    """
    denied = RealCapabilityInput(1.0, 0.0, False, False)
    raw_attestation_fd = os.environ.get("TIANJI_REAL_PREFLIGHT_FD", "")
    raw_scanner_fd = os.environ.get("TIANJI_REAL_PREFLIGHT_SCANNER_FD", "")
    try:
        if not raw_attestation_fd.isdigit() or not raw_scanner_fd.isdigit():
            return denied
        attestation_fd = int(raw_attestation_fd)
        scanner_fd = int(raw_scanner_fd)
        if fcntl.fcntl(attestation_fd, _SEAL_GET) & _SEAL_ALL != _SEAL_ALL:
            return denied
        payload_bytes = _read_fd(attestation_fd)
        value = json.loads(payload_bytes.decode("utf-8"))
        if not isinstance(value, dict) or set(value) != {
            "run_id", "router_zid", "validation_supervisor_instance_id",
            "launcher_nonce", "scanner_id", "scanner_sha256",
            "scanner_device", "scanner_inode", "capability", "payload_sha256",
        }:
            return denied
        payload_digest = value.pop("payload_sha256")
        if not isinstance(payload_digest, str) or hashlib.sha256(_canonical_json(value)).hexdigest() != payload_digest:
            return denied
        scanner_stat = os.fstat(scanner_fd)
        if (
            not stat.S_ISREG(scanner_stat.st_mode)
            or scanner_stat.st_uid != 0
            or scanner_stat.st_mode & 0o022
        ):
            return denied
        scanner_bytes = _read_fd(scanner_fd)
        scanner_id, scanner_capability = _scanner_capability(scanner_bytes)
        if value["scanner_id"] != scanner_id:
            return denied
        if value["scanner_sha256"] != hashlib.sha256(scanner_bytes).hexdigest():
            return denied
        if value["scanner_device"] != scanner_stat.st_dev or value["scanner_inode"] != scanner_stat.st_ino:
            return denied
        capability = RealCapabilityInput.from_mapping(value["capability"])
        if capability != scanner_capability:
            return denied
        if value["run_id"] != os.environ.get("TIANJI_RUN_ID", ""):
            return denied
        if value["router_zid"] != os.environ.get("TIANJI_ROUTER_ZID", ""):
            return denied
        if value["validation_supervisor_instance_id"] != os.environ.get(
            "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID", ""
        ):
            return denied
        return capability
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
        return denied


__all__ = ["trusted_real_capability"]
