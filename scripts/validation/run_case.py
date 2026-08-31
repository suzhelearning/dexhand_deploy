#!/usr/bin/env python3
"""Run one Tianji validation case and create an auditable result bundle.

The command deliberately never synthesizes a passing device result.  ``--fake
--headless`` exercises bundle creation and preflight only; its operator result
is ``aborted`` until an operator records a real observation.
"""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import math
from dataclasses import dataclass
from datetime import datetime, timezone
import stat
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import sys
import time
import uuid
from typing import Any, Callable, Iterable, Mapping

import yaml

ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = ROOT / "src" / "pico_body_tianji"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pico_body_tianji.config_loader import DEFAULT_ROUTER_ENDPOINT, canonical_config_root
from pico_body_tianji.protocol.messages import ProtocolEnvelope, SafetyStopAck, SafetyStopRequest
from pico_body_tianji.sources.common.real_admission import RealCapabilityInput

MATRIX_PATH = canonical_config_root() / "validation" / "test_matrix.yaml"
MATRIX_SCHEMA = "tianji-validation-matrix"
BUNDLE_SCHEMA = "tianji-validation-run"
BUNDLE_VERSION = "1.0"
OUTCOMES = frozenset({"pass", "fail", "aborted"})
DANGEROUS_STOPS = frozenset({
    "wrong_direction_or_side", "physical_limit", "collision_risk", "feedback_stale",
    "tracking_threshold", "device_or_servo_error", "duplicate_authority",
    "router_zid_change", "emergency_stop",
})


def monotonic_ns() -> int:
    return time.monotonic_ns()
    
CASE_CONTRACTS = {
    "acquisition_live": {"profile": "acquisition_live", "producer": "acquisition", "ik_backend": None, "recordable": False, "source_capability": "simulation", "hand_mode": "disabled"},
    "pico_sim": {"profile": "pico_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "mocap_live_sim": {"profile": "mocap_live_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "h5_sim": {"profile": "h5_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "auto"},
    "ik_pinocchio_cpp": {"profile": "pico_sim", "producer": "ik", "ik_backend": "pinocchio_cpp", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "ik_pinocchio_qp": {"profile": "pico_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "ik_tianji_official": {"profile": "pico_sim", "producer": "ik", "ik_backend": "tianji_official", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "target_replay_sim": {"profile": "target_replay_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": False, "source_capability": "simulation", "hand_mode": "auto"},
    "joint_replay_sim": {"profile": "joint_replay_sim", "producer": "joint_replay", "ik_backend": None, "recordable": False, "source_capability": "simulation", "hand_mode": "direct"},
    "policy_hold_sim": {"profile": "pico_sim", "producer": "policy_hold", "ik_backend": None, "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "marvin_pico_real_10pct": {"profile": "pico_real", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "real", "hand_mode": "disabled"},
    "marvin_mocap_live_real_10pct": {"profile": "mocap_live_real", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "real", "hand_mode": "disabled"},
    "marvin_h5_real_10pct": {"profile": "h5_real", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "real", "hand_mode": "auto"},
    "wuji_retarget_dry": {"profile": "h5_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "retarget"},
    "wuji_retarget_real": {"profile": "h5_real", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "real", "hand_mode": "retarget"},
    "wuji_direct_real": {"profile": "wuji_direct_real", "producer": "joint_replay", "ik_backend": None, "recordable": False, "source_capability": "real", "hand_mode": "direct"},
    "fault_recovery_sim": {"profile": "pico_sim", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "simulation", "hand_mode": "disabled"},
    "fault_recovery_real": {"profile": "pico_real", "producer": "ik", "ik_backend": "pinocchio_qp", "recordable": True, "source_capability": "real", "hand_mode": "disabled"},
}


def build_session_contract(case_id: str, overrides: Mapping[str, Any] | None = None) -> dict[str, Any]:
    try:
        contract = dict(CASE_CONTRACTS[case_id])
    except KeyError as exc:
        raise ValueError(f"unknown validation case: {case_id}") from exc
    override = dict(overrides or {})
    requested = override.get("ik_backend")
    if requested is not None and requested != contract["ik_backend"]:
        raise ValueError(f"case {case_id} requires IK backend {contract['ik_backend']!r}, got {requested!r}")
    profile = _profile_config(contract["profile"])
    if profile:
        if profile.get("required_capability") != contract["source_capability"] and case_id != "wuji_direct_real":
            raise ValueError(f"case {case_id} source capability/profile mismatch")
        if contract["hand_mode"] != "auto" and profile.get("hand_mode") not in {contract["hand_mode"], "auto"}:
            raise ValueError(f"case {case_id} hand mode/profile mismatch")
    return contract


def validate_operator_finalization(outcome: str, events: Iterable[str], *, rc: int) -> str:
    if outcome not in OUTCOMES:
        raise ValueError(f"invalid operator outcome: {outcome}")
    events = tuple(str(event) for event in events)
    if outcome == "pass" and (rc != 0 or any(event in DANGEROUS_STOPS for event in events)):
        raise ValueError("dangerous stop or non-zero run cannot be finalized as pass")
    return outcome


class AlignedStreamObservation:
    """Fail-closed observer for the external acquisition aligned stream."""

    def __init__(self, *, min_samples: int = 2, min_rate_hz: float = 0.0) -> None:
        if min_samples < 1 or min_rate_hz < 0.0:
            raise ValueError("invalid aligned-stream observation thresholds")
        self.min_samples = int(min_samples)
        self.min_rate_hz = float(min_rate_hz)
        self.samples = 0
        self.complete = False
        self.stream_instance_id: str | None = None
        self.router_zid: str | None = None
        self.last_sequence: int | None = None
        self._first_timestamp_ns: int | None = None
        self._last_timestamp_ns: int | None = None

    @property
    def rate_hz(self) -> float:
        if self.samples < 2 or self._first_timestamp_ns is None or self._last_timestamp_ns is None:
            return 0.0
        elapsed = self._last_timestamp_ns - self._first_timestamp_ns
        return (self.samples - 1) / (elapsed / 1e9) if elapsed > 0 else 0.0

    def accept(self, row: Mapping[str, Any]) -> bool:
        required = ("stream_instance_id", "stream_sequence", "router_zid", "left_valid", "right_valid")
        if any(key not in row for key in required):
            return False
        instance = str(row["stream_instance_id"])
        router = str(row["router_zid"])
        sequence = row["stream_sequence"]
        if not instance or not router or isinstance(sequence, bool) or not isinstance(sequence, int):
            return False
        if self.stream_instance_id is None:
            self.stream_instance_id, self.router_zid = instance, router
        if instance != self.stream_instance_id or router != self.router_zid:
            return False
        if self.last_sequence is not None and sequence != self.last_sequence + 1:
            return False
        if not isinstance(row["left_valid"], bool) or not isinstance(row["right_valid"], bool):
            return False
        timestamp = row.get("timestamp_ns", row.get("time_ns"))
        if timestamp is not None:
            if isinstance(timestamp, bool) or not isinstance(timestamp, int):
                return False
            if self._last_timestamp_ns is not None and timestamp <= self._last_timestamp_ns:
                return False
        elif self.min_rate_hz > 0.0:
            return False
        self.last_sequence = sequence
        self.samples += 1
        if timestamp is not None:
            self._first_timestamp_ns = timestamp if self._first_timestamp_ns is None else self._first_timestamp_ns
            self._last_timestamp_ns = timestamp
        self.complete = self.samples >= self.min_samples and self.rate_hz >= self.min_rate_hz
        return True


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _memfd_create(name: str) -> int:
    creator = getattr(os, "memfd_create", None)
    if creator is not None:
        return int(creator(name, getattr(os, "MFD_ALLOW_SEALING", 0x0002)))
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.memfd_create
    function.argtypes = [ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    fd = int(function(name.encode("ascii"), 0x0002))
    if fd < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return fd


def _sealed_memfd(payload: bytes, *, name: str) -> int:
    fd = _memfd_create(name)
    try:
        offset = 0
        while offset < len(payload):
            offset += os.write(fd, payload[offset:])
        fcntl.fcntl(
            fd,
            getattr(fcntl, "F_ADD_SEALS", 1033),
            getattr(fcntl, "F_SEAL_SEAL", 1)
            | getattr(fcntl, "F_SEAL_SHRINK", 2)
            | getattr(fcntl, "F_SEAL_GROW", 4)
            | getattr(fcntl, "F_SEAL_WRITE", 8),
        )
        return fd
    except Exception:
        os.close(fd)
        raise


def _read_fd(fd: int) -> bytes:
    size = os.fstat(fd).st_size
    if size < 0:
        raise ValueError("invalid file descriptor size")
    payload = os.pread(fd, size, 0)
    if len(payload) != size:
        raise ValueError("short read from immutable file descriptor")
    return payload


@dataclass
class RealPreflightBinding:
    nonce: str
    digest: str
    payload: bytes
    attestation_fd: int
    scanner_fd: int

    def close(self) -> None:
        for name in ("attestation_fd", "scanner_fd"):
            fd = getattr(self, name)
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass
                setattr(self, name, -1)




def sha256_tree(path: Path, *, suffixes: Iterable[str] | None = None) -> str:
    digest = hashlib.sha256()
    if not path.exists():
        return "unavailable"
    allowed = set(suffixes) if suffixes is not None else None
    files = [item for item in path.rglob("*") if item.is_file()]
    for item in sorted(files):
        if allowed is not None and item.suffix not in allowed:
            continue
        relative = item.relative_to(path).as_posix().encode()
        digest.update(relative)
        digest.update(b"\0")
        with item.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def git_fingerprint(path: Path) -> dict[str, Any]:
    if not (path / ".git").exists():
        return {"commit": "unavailable", "dirty": None}
    try:
        commit = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        dirty = bool(subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True, stderr=subprocess.DEVNULL).strip())
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "unavailable", "dirty": None}


def load_matrix(path: Path = MATRIX_PATH) -> dict[str, Any]:
    try:
        matrix = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read validation matrix: {path}: {exc}") from exc
    if not isinstance(matrix, dict) or matrix.get("schema_name") != MATRIX_SCHEMA or matrix.get("schema_version") != BUNDLE_VERSION:
        raise ValueError("unsupported validation matrix schema")
    cases = matrix.get("cases")
    if not isinstance(cases, dict) or not cases:
        raise ValueError("validation matrix must contain cases")
    required = {"profile", "required_devices", "required_capability", "active_sides", "hand_mode", "velocity_ratio", "acceleration_ratio", "max_duration_s", "prerequisites", "stop_criteria"}
    for case_id, case in cases.items():
        if not isinstance(case_id, str) or not isinstance(case, dict) or set(case) != required:
            raise ValueError(f"invalid validation case schema: {case_id}")
        if case["required_capability"] not in {"simulation", "real"}:
            raise ValueError(f"invalid required capability for {case_id}")
        if not isinstance(case["required_devices"], list) or not isinstance(case["active_sides"], list):
            raise ValueError(f"invalid device or sides list for {case_id}")
        if not isinstance(case["prerequisites"], list) or float(case["max_duration_s"]) <= 0:
            raise ValueError(f"invalid duration or prerequisites for {case_id}")
        stops = case["stop_criteria"]
        if not isinstance(stops, dict) or not stops.get("dangerous") or not stops.get("controlled"):
            raise ValueError(f"invalid stop criteria for {case_id}")
    return matrix


def _payload(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        return value
    if hasattr(value, "to_dict"):
        result = value.to_dict()
        if isinstance(result, Mapping):
            return result
    raise TypeError("safety ack must be a mapping or protocol message")


@dataclass(frozen=True)
class SafetyStopResult:
    accepted: bool
    locked: bool
    reason: str
    request: SafetyStopRequest
    acked_executor_ids: tuple[str, ...]
    ack_timestamps_ns: tuple[int, ...] = ()


class SafetyStopSupervisor:
    """Issue one authorized, latched stop and require every executor ack.

    ``publish`` and ``wait_ack`` are injected so the same strict logic can be
    used by a Zenoh process and by deterministic fake-device tests.  Missing or
    mismatched acknowledgements never become a pass; the supervisor remains
    locked and instructs the operator to maintain physical E-stop.
    """

    def __init__(self, run_id: str, supervisor_instance_id: str, router_zid: str, *, clock: Callable[[], int] = monotonic_ns) -> None:
        if not run_id or not supervisor_instance_id or not router_zid:
            raise ValueError("run_id, supervisor_instance_id and router_zid are required")
        self.run_id = run_id
        self.supervisor_instance_id = supervisor_instance_id
        self.router_zid = router_zid
        self.clock = clock
        self.sequence = 0
        self.locked = False
        self.last_request: SafetyStopRequest | None = None

    def issue(
        self,
        reason: str,
        executor_ids: Iterable[str],
        *,
        publish: Callable[[SafetyStopRequest], Any],
        wait_ack: Callable[[SafetyStopRequest], Mapping[str, Any] | Iterable[Any]],
    ) -> SafetyStopResult:
        if reason not in DANGEROUS_STOPS and not reason.strip():
            raise ValueError(f"invalid safety stop reason: {reason!r}")
        expected = tuple(dict.fromkeys(str(item) for item in executor_ids if str(item)))
        if not expected:
            raise ValueError("at least one executor is required for safety stop")
        self.sequence += 1
        request = SafetyStopRequest(
            ProtocolEnvelope(1, self.supervisor_instance_id, self.router_zid, self.sequence, int(self.clock())),
            self.run_id,
            reason,
            True,
        )
        self.last_request = request
        self.locked = True
        try:
            publish(request)
            raw_acks = wait_ack(request)
        except Exception as exc:  # fail closed; physical E-stop remains required
            return SafetyStopResult(False, True, f"safety stop ack unavailable: {exc}", request, ())
        if isinstance(raw_acks, Mapping):
            values = raw_acks.items()
        else:
            values = ((None, value) for value in raw_acks)
        valid: dict[str, Any] = {}
        errors: list[str] = []
        for key, value in values:
            try:
                payload = _payload(value)
                executor_id = str(payload.get("executor_id", key or ""))
                if executor_id not in expected:
                    errors.append(f"unexpected executor {executor_id}")
                    continue
                ack = value if isinstance(value, SafetyStopAck) else SafetyStopAck.from_dict(payload)
                ack.validate_for(executor_id, self.run_id)
                if ack.envelope.publisher_instance_id != executor_id:
                    raise ValueError("ack publisher instance does not match expected executor")
                if ack.envelope.router_zid != self.router_zid or ack.envelope.sequence != request.envelope.sequence:
                    raise ValueError("ack sequence/router mismatch")
                valid[executor_id] = ack
            except (TypeError, ValueError, KeyError) as exc:
                errors.append(str(exc))
        missing = [executor_id for executor_id in expected if executor_id not in valid]
        if missing:
            errors.append("missing ack: " + ", ".join(missing))
        if errors:
            return SafetyStopResult(False, True, "safety stop ack failure: " + "; ".join(errors), request, tuple(sorted(valid)), tuple(int(valid[item].envelope.timestamp_ns) for item in sorted(valid)))
        return SafetyStopResult(True, True, "all executor safety-stop acknowledgements received", request, tuple(sorted(valid)), tuple(int(valid[item].envelope.timestamp_ns) for item in sorted(valid)))
class ZenohSafetyTransport:
    """Small transport adapter for an explicit managed safety stop.

    The adapter is only constructed after the managed session has started.
    It never emits return or clear messages and closes after collecting the
    matching acknowledgements (or timeout).
    """

    def __init__(self, endpoint: str, *, timeout_s: float = 3.0) -> None:
        import threading
        from pico_body_tianji.zenoh_util import open_session

        self._threading = threading
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0:
            raise ValueError("safety ack timeout must be positive")
        self.session = open_session(endpoint)
        self.publisher = self.session.declare_publisher("tianji/safety/stop")
        self._acks: list[Mapping[str, Any]] = []
        self._event = threading.Event()
        self.subscriber = self.session.declare_subscriber("tianji/safety/ack/**", self._on_ack)

    def _on_ack(self, sample: Any) -> None:
        try:
            payload = sample.payload.to_bytes()
            value = json.loads(payload.decode("utf-8"))
            if isinstance(value, Mapping):
                self._acks.append(value)
                self._event.set()
        except (AttributeError, UnicodeDecodeError, json.JSONDecodeError, TypeError):
            # A malformed ack is not accepted; the supervisor will report a
            # missing/mismatched acknowledgement and remain locked.
            return

    def publish(self, request: SafetyStopRequest) -> None:
        self.publisher.put(json.dumps(request.to_dict(), separators=(",", ":")).encode("utf-8"), encoding="application/json")

    def wait_ack(
        self,
        request: SafetyStopRequest,
        expected_executor_ids: Iterable[str] = (),
    ) -> Mapping[str, Any]:
        del request
        expected = {str(item) for item in expected_executor_ids if str(item)}
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            self._event.wait(timeout=max(0.0, min(0.1, deadline - time.monotonic())))
            values = {str(row.get("executor_id", "")): row for row in self._acks}
            if values and (not expected or expected.issubset(values)):
                return values
        return {str(row.get("executor_id", "")): row for row in self._acks}

    def close(self) -> None:
        for item in (getattr(self, "subscriber", None), getattr(self, "publisher", None), getattr(self, "session", None)):
            try:
                if item is not None:
                    item.close() if hasattr(item, "close") else item.undeclare()
            except Exception:
                pass


def _source_type(profile: str) -> str:
    values = {
        "acquisition_live": "mocap_live",
        "pico_sim": "pico_controller",
        "pico_real": "pico_controller",
        "mocap_live_sim": "mocap_live",
        "mocap_live_real": "mocap_live",
        "h5_sim": "h5_replay",
        "h5_real": "h5_replay",
        "target_replay_sim": "target_replay",
        "joint_replay_sim": "joint_replay",
        "wuji_direct_real": "joint_replay",
        "diagnostic_mocap_calibration_sim": "diagnostic_mocap_calibration",
    }
    return values.get(profile, "pico_controller")
def _profile_config(profile: str) -> dict[str, Any]:
    path = canonical_config_root() / "sessions" / f"{profile}.yaml"
    if not path.is_file():
        return {}
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return value if isinstance(value, dict) else {}


def _hashes() -> dict[str, str]:
    config_root = canonical_config_root()
    acl = Path("/home/current/syz/mocap/acquisition/config/zenohd_acl.yaml")
    return {
        "config_sha256": sha256_tree(config_root, suffixes={".yaml", ".yml"}),
        "runtime_sha256": sha256_tree(ROOT / "runtime"),
        "acl_sha256": sha256_file(acl) if acl.is_file() else "unavailable",
    }
def _bind_real_preflight(
    source_path: Path,
    bound_path: Path,
    *,
    run_id: str,
    supervisor: str,
    router_zid: str,
) -> RealPreflightBinding:
    """Bind one protected scanner result and keep its immutable bytes for children."""
    scanner_fd = -1
    attestation_fd = -1
    audit_created = False
    try:
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
        scanner_fd = os.open(str(source_path), flags)
        scanner_stat = os.fstat(scanner_fd)
        if (
            not stat.S_ISREG(scanner_stat.st_mode)
            or scanner_stat.st_uid != 0
            or scanner_stat.st_mode & 0o022
        ):
            raise PermissionError("real preflight source must be a root-owned protected regular file")
        scanner_bytes = _read_fd(scanner_fd)
        value = yaml.safe_load(scanner_bytes.decode("utf-8"))
        if not isinstance(value, Mapping) or set(value) != {"scanner_id", "capability"} or not str(value["scanner_id"]).strip():
            raise PermissionError("real preflight source is not a scanner attestation")
        capability = RealCapabilityInput.from_mapping(value["capability"])
        nonce = uuid.uuid4().hex
        base = {
            "run_id": run_id,
            "router_zid": router_zid,
            "validation_supervisor_instance_id": supervisor,
            "launcher_nonce": nonce,
            "scanner_id": str(value["scanner_id"]).strip(),
            "scanner_sha256": hashlib.sha256(scanner_bytes).hexdigest(),
            "scanner_device": int(scanner_stat.st_dev),
            "scanner_inode": int(scanner_stat.st_ino),
            "capability": {
                "speed": float(capability.speed),
                "yaw_deg": float(capability.yaw_deg),
                "deadman_available": capability.deadman_available,
                "preflight_passed": capability.preflight_passed,
            },
        }
        payload = dict(base)
        payload["payload_sha256"] = hashlib.sha256(_canonical_json(base)).hexdigest()
        payload_bytes = _canonical_json(payload)
        audit_fd = os.open(
            str(bound_path),
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o400,
        )
        audit_created = True
        try:
            offset = 0
            while offset < len(payload_bytes):
                offset += os.write(audit_fd, payload_bytes[offset:])
            os.fsync(audit_fd)
            os.fchmod(audit_fd, 0o400)
        finally:
            os.close(audit_fd)
        attestation_fd = _sealed_memfd(payload_bytes, name="tianji-real-preflight")
        return RealPreflightBinding(
            nonce=nonce,
            digest=hashlib.sha256(payload_bytes).hexdigest(),
            payload=payload_bytes,
            attestation_fd=attestation_fd,
            scanner_fd=scanner_fd,
        )
    except Exception:
        if attestation_fd >= 0:
            os.close(attestation_fd)
        if scanner_fd >= 0:
            os.close(scanner_fd)
        if audit_created:
            try:
                bound_path.unlink()
            except OSError:
                pass
        raise


def _wait_for_launcher_startup(
    log_path: Path,
    *,
    timeout_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Any] = time.sleep,
    process: Any | None = None,
) -> bool:
    """Wait for run_session's explicit post-launch marker, not elapsed time."""
    if timeout_s <= 0:
        return False
    deadline = clock() + float(timeout_s)
    while clock() < deadline:
        try:
            text = log_path.read_text(encoding="utf-8")
        except OSError:
            text = ""
        if any(line.startswith("session_startup_complete ") for line in text.splitlines()):
            return True
        if process is not None and process.poll() is not None:
            return False
        sleep(min(0.05, max(0.0, deadline - clock())))
    return False


def _launcher_startup_timeout(manifest: Mapping[str, Any]) -> float:
    components = sum(
        1 for entry in manifest.get("authority_contract", [])
        if entry.get("component_role") in {
            "source", "producer_arm", "producer_hand", "coordinator_arm",
            "executor_arm", "executor_hand", "recorder",
        }
    )
    return max(15.0, float(max(1, components) * 3))
def _executor_ready_timeout(manifest: Mapping[str, Any]) -> float:
    """Bound the readiness wait from the profile's safety configuration."""
    profile = _profile_config(str(manifest.get("profile", "")))
    values = [float(_launcher_startup_timeout(manifest))]
    config_root = canonical_config_root()
    for config_name in (profile.get("coordinator_config"), profile.get("arm_executor_config"), "executors/wuji_hand2.yaml"):
        if not config_name:
            continue
        try:
            config = yaml.safe_load((config_root / str(config_name)).read_text(encoding="utf-8")) or {}
            for key in ("home_minimum_duration_s", "enable_timeout_s", "connection_wait_s", "hand_return_timeout_s"):
                value = float(config.get(key, 0.0))
                if math.isfinite(value) and value > 0.0:
                    values.append(value)
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            continue
    # Include a scheduling margin and one extra status period per configured
    # component; there is no fixed post-marker five-second shortcut.
    return max(values) + max(2.0, float(len(manifest.get("authority_contract", []))) * 0.5)


def _latest_ready_executor(rows: list[Mapping[str, Any]], executor_id: str, manifest: Mapping[str, Any]) -> bool:
    expected = [
        entry for entry in manifest.get("authority_contract", [])
        if str(entry.get("publisher_instance_id", "")) == executor_id
        and entry.get("component_role") in {"executor_arm", "executor_hand"}
    ]
    candidates = [
        row for row in rows
        if str(row.get("publisher_instance_id", "")) == executor_id
        and row.get("component_role") in {"executor_arm", "executor_hand"}
    ]
    if not candidates or not expected:
        return False
    latest = max(candidates, key=lambda row: int(row.get("sequence", -1)) if isinstance(row.get("sequence"), int) else -1)
    authority = expected[0]
    diagnostics = latest.get("diagnostics")
    side = diagnostics.get("side") if isinstance(diagnostics, Mapping) else latest.get("side")
    if (
        latest.get("component_role") != authority.get("component_role")
        or latest.get("component_id") != authority.get("logical_id")
        or (authority.get("side") is not None and side != authority.get("side"))
    ):
        return False
    timestamp = latest.get("timestamp_ns")
    if not isinstance(timestamp, int) or isinstance(timestamp, bool):
        return False
    now = monotonic_ns()
    if timestamp > now or now - timestamp > int(_executor_ready_timeout(manifest) * 1e9):
        return False
    return (
        latest.get("router_zid") == manifest.get("router_zid")
        and latest.get("ready") is True
        and latest.get("healthy") is True
    )



def _write_status(stream: Any, *, event: str, component: str, supervisor: str, run_id: str, **fields: Any) -> None:
    record = {
        "schema_version": 1,
        "timestamp_ns": monotonic_ns(),
        "run_id": run_id,
        "component": component,
        "event": event,
        "publisher_instance_id": supervisor,
        **fields,
    }
    stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
    stream.flush()


def _write_operator_event(path: Path, supervisor: str, run_id: str, event: str, details: str = "") -> None:
    if not event.strip():
        raise ValueError("operator event cannot be empty")
    record = {
        "schema_version": 1,
        "time_ns": monotonic_ns(),
        "run_id": run_id,
        "publisher_instance_id": supervisor,
        "event": event.strip(),
        "details": details,
    }
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def _write_checksums(bundle: Path, *, immutable_files: Mapping[str, bytes] | None = None) -> None:
    names = [
        "manifest.yaml", "status.jsonl", "operator_events.jsonl",
        "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml",
    ]
    if (bundle / "real-preflight.json").is_file():
        names.insert(1, "real-preflight.json")
    if (bundle / "session.h5").is_file():
        names.insert(1, "session.h5")
    names += sorted(path.relative_to(bundle).as_posix() for path in (bundle / "logs").glob("*") if path.is_file())
    immutable = dict(immutable_files or {})
    with (bundle / "checksums.sha256").open("w", encoding="utf-8") as stream:
        for name in names:
            payload = immutable.get(name)
            digest = hashlib.sha256(payload).hexdigest() if payload is not None else sha256_file(bundle / name)
            stream.write(f"{digest}  {name}\n")


def _create_empty_session(path: Path, source_type: str, router_zid: str) -> None:
    # Empty is intentional: fake/headless mode proves the schema and safety
    # plumbing only. It is never reported as a successful device run.
    try:
        from pico_body_tianji.recording.session_h5 import SessionH5Writer
    except ModuleNotFoundError as exc:
        # ``validation-run --list`` and local preflight must work without the
        # optional Zenoh wheel.  Keep a minimal HDF5 artifact for that mode;
        # the Pixi runtime uses the canonical SessionH5Writer below.
        if exc.name != "zenoh":
            raise
        import h5py
        path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(path, "w") as file:
            file.attrs.update(
                schema_name="tianji-teleop-session",
                schema_version="1.0",
                source_type=source_type,
                robot_model="unknown-unverified",
                router_zid=router_zid,
                complete=True,
            )
            for group in ("raw", "target", "joint", "meta"):
                file.create_group(group)
        return
    with SessionH5Writer(path, source_type=source_type, robot_model="unknown-unverified", router_zid=router_zid):
        pass


class ManagedEvidenceCapture:
    """Capture real wire/status/liveliness evidence while a session is online."""

    def __init__(self, endpoint: str, bundle: Path, run_id: str) -> None:
        from pico_body_tianji.zenoh_util import open_session

        self.bundle = bundle
        self.run_id = run_id
        self.session = open_session(endpoint)
        self._streams: list[Any] = []
        self._lock = __import__("threading").Lock()
        self._streams.append(self.session.declare_subscriber("tianji/**", self._on_data))
        try:
            self._streams.append(self.session.liveliness().declare_subscriber("tj/live/**", self._on_liveliness))
        except (AttributeError, TypeError):
            self._streams.append(self.session.declare_subscriber("tj/live/**", self._on_liveliness))

    @staticmethod
    def _key(sample: Any) -> str:
        value = getattr(sample, "key_expr", "")
        return str(value)

    @staticmethod
    def _payload(sample: Any) -> dict[str, Any] | None:
        try:
            raw = bytes(sample.payload)
            value = json.loads(raw.decode("utf-8"))
        except (AttributeError, UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
            return None
        return dict(value) if isinstance(value, Mapping) else None

    def _append(self, filename: str, row: Mapping[str, Any]) -> None:
        with self._lock, (self.bundle / filename).open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(row), ensure_ascii=False, separators=(",", ":")) + "\n")
            stream.flush()

    def _on_data(self, sample: Any) -> None:
        payload = self._payload(sample)
        if payload is None:
            return
        # Protocol records must already carry their complete wire envelope.
        # The capture layer may add only local run/topic metadata; it must
        # never invent schema or router identity for malformed samples.
        required = {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"}
        if not required.issubset(payload) or payload.get("schema_version") != 1:
            return
        try:
            ProtocolEnvelope.from_dict({key: payload[key] for key in required})
        except (TypeError, ValueError):
            return
        router = str(payload.get("router_zid", ""))
        expected_router = str(os.environ.get("TIANJI_ROUTER_ZID", ""))
        if not router or (expected_router and router != expected_router):
            return
        key = self._key(sample)
        payload["run_id"] = self.run_id
        payload["topic"] = key
        if key.endswith("/status"):
            self._append("status.jsonl", payload)
        else:
            self._append("protocol.jsonl", payload)

    def _on_liveliness(self, sample: Any) -> None:
        key = self._key(sample)
        parts = key.split("/")
        row = {
            "schema_version": 1,
            "run_id": self.run_id,
            "timestamp_ns": monotonic_ns(),
            "key_expr": key,
            "publisher_instance_id": parts[-1] if parts else "",
            "router_zid": os.environ.get("TIANJI_ROUTER_ZID", ""),
        }
        self._append("liveliness.jsonl", row)

    def close(self) -> None:
        for stream in self._streams:
            try:
                stream.undeclare() if hasattr(stream, "undeclare") else stream.close()
            except Exception:
                pass
        try:
            self.session.close()
        except Exception:
            pass

def _prerequisites_passed(root: Path, prerequisites: Iterable[str]) -> tuple[bool, list[str]]:
    """Accept only bundles validated by analyzer, never directory-name hints."""
    missing: list[str] = []
    if not prerequisites:
        return True, missing
    try:
        from scripts.validation.analyze_runs import AnalysisError, analyze_bundle
    except ImportError as exc:
        raise RuntimeError(f"cannot load prerequisite analyzer: {exc}") from exc
    matrix = load_matrix()
    for prerequisite in prerequisites:
        found = False
        if root.is_dir():
            for candidate in sorted(path for path in root.iterdir() if path.is_dir() and (path / "manifest.yaml").is_file()):
                try:
                    manifest = yaml.safe_load((candidate / "manifest.yaml").read_text(encoding="utf-8")) or {}
                    if manifest.get("case_id") != prerequisite:
                        continue
                    report = analyze_bundle(candidate, matrix)
                    found = report.get("operator_outcome") == "pass"
                except (AnalysisError, OSError, yaml.YAMLError, ValueError):
                    found = False
                if found:
                    break
        if not found:
            missing.append(prerequisite)
    return not missing, missing


def _h5_contains_hand_joints(path: Path | None, active_sides: Iterable[str] = ("right",)) -> bool:
    """Return true only when every active side has valid direct-joint frames."""
    if path is None or not path.is_file():
        return False
    try:
        from pico_body_tianji.sources.mocap.h5 import load_mocap_h5
        recording = load_mocap_h5(path)
        import numpy as np
        sides = tuple(str(side) for side in active_sides)
        if not sides:
            return False
        for side in sides:
            if side not in {"left", "right"}:
                return False
            joints = recording.hands[side].wuji2_joints
            valid = recording.hands[side].valid
            if joints is None or joints.shape != (recording.frame_count, 20) or not bool(valid.any()):
                return False
            if not bool(np.isfinite(joints[valid]).all()):
                return False
        return True
    except (OSError, ValueError, ModuleNotFoundError, ImportError):
        return False


def _resolved_hand_mode(case: Mapping[str, Any], profile: str, input_path: Path | None) -> str:
    mode = str(case.get("hand_mode", "disabled"))
    if mode != "auto":
        return mode
    if profile == "target_replay_sim":
        return "retarget"
    return "direct" if _h5_contains_hand_joints(input_path, case.get("active_sides", ("right",))) else "retarget"


def _instance_map(manifest: Mapping[str, Any], key: str) -> dict[str, str]:
    value = manifest.get("publisher_instance_ids", {}).get(key, {})
    return dict(value) if isinstance(value, Mapping) else {}

def instance_handoff_environment(manifest: Mapping[str, Any]) -> dict[str, str]:
    """Return exact preallocated IDs for run_session child processes."""
    ids = manifest.get("publisher_instance_ids", {})
    environment: dict[str, str] = {
        "TIANJI_RUN_ID": str(manifest["run_id"]),
        "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID": str(ids.get("validation_supervisor", "")),
        "TIANJI_COORDINATOR_INSTANCE_ID": str(ids.get("coordinator_arm", "")),
        "TIANJI_SOURCE_INSTANCE_ID": str(ids.get("source", "")),
        "TIANJI_ARM_PRODUCER_INSTANCE_ID": str(ids.get("producer_arm", "")),
        "TIANJI_ARM_EXECUTOR_INSTANCE_ID": str(ids.get("executor_arm", "")),
        "TIANJI_RECORDER_INSTANCE_ID": str(ids.get("recorder", "")),
    }
    producer = _instance_map(manifest, "hand_producer_instances")
    executor = _instance_map(manifest, "hand_executor_instances")
    if producer:
        environment["TIANJI_HAND_PRODUCER_INSTANCES"] = ",".join(f"{side}={value}" for side, value in sorted(producer.items()))
    if executor:
        environment["TIANJI_HAND_EXECUTOR_INSTANCES"] = ",".join(f"{side}={value}" for side, value in sorted(executor.items()))
    return {key: value for key, value in environment.items() if value}


def _authority_contract(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Expand manifest IDs into role/logical/side/topic/live identities."""
    ids = manifest.get("publisher_instance_ids", {})
    router = str(manifest.get("router_zid", ""))
    profile = str(manifest.get("profile", ""))
    source = _source_type(profile)
    producer = str(manifest.get("producer", ""))
    source_logical = "acquisition" if profile == "acquisition_live" else source
    entries: list[dict[str, Any]] = []

    def add(role: str, logical_id: str, instance: Any, side: str | None, topics: list[str], live_role: str) -> None:
        if not instance or str(instance) in {"disabled", "None"}:
            return
        entries.append({
            "component_role": role,
            "logical_id": logical_id,
            "side": side,
            "publisher_instance_id": str(instance),
            "router_zid": router,
            "topics": topics,
            "liveliness": f"tj/live/{live_role}/{logical_id}/{instance}",
        })

    add("source", source_logical, ids.get("source"), None, ["tianji/source/status", "tianji/session/intent", f"tianji/raw/{source}", f"tianji/target/arm/{{side}}", f"tianji/target/hand/{{side}}", "tianji/diagnostics/h5/frame0_hand_skeleton"], "source")
    if ids.get("producer_arm"):
        producer_logical = "joint_replay" if producer == "joint_replay" else producer
        if producer_logical == "ik":
            producer_logical = "arm_ik_producer"
        add("producer_arm", producer_logical, ids.get("producer_arm"), None, ["tianji/producer/status", "tianji/proposal/arm/{side}", "tianji/producer/arm/{side}/solved_pose"], "producer/arm")
    add("coordinator_arm", "arm", ids.get("coordinator_arm"), None, ["tianji/coordinator/status", "tianji/session/state", "tianji/coordinator/at_home", "tianji/coordinator/return_complete", "tianji/command/arm/{side}"], "coordinator/arm")
    executor_config = _profile_config(profile).get("arm_executor_config", "")
    executor_logical = "marvin" if str(executor_config).endswith("marvin.yaml") else "mujoco"
    add("executor_arm", executor_logical, ids.get("executor_arm"), None, ["tianji/executor/status", "tianji/state/arm"], "executor/arm")
    hand_producers = _instance_map(manifest, "hand_producer_instances")
    hand_executors = _instance_map(manifest, "hand_executor_instances")
    for side, instance in sorted(hand_producers.items()):
        logical = "h5_direct" if manifest.get("resolved_hand_mode") == "direct" and profile in {"h5_sim", "h5_real"} else (
            "joint_replay" if source == "joint_replay" else f"wuji_retarget_{side}"
        )
        add("producer_hand", logical, instance, side, ["tianji/producer/status", f"tianji/command/hand/{side}"], "producer/hand")
    for side, instance in sorted(hand_executors.items()):
        add("executor_hand", f"wuji_{side}", instance, side, [f"tianji/executor/hand/{side}/status", f"tianji/state/hand/{side}"], "executor/hand")
    if ids.get("recorder"):
        add("recorder", "session_recorder", ids.get("recorder"), None, ["tianji/recorder/status"], "recorder")
    return entries


def _build_manifest(case_id: str, case: Mapping[str, Any], profile: str, run_id: str, supervisor: str, router_zid: str, started: str, args: argparse.Namespace) -> dict[str, Any]:
    profile_config = _profile_config(profile)
    source = profile_config.get("source_config", "")
    contract = build_session_contract(case_id, {"ik_backend": args.ik_backend})
    backend = contract["ik_backend"]
    input_path = Path(args.input).expanduser() if args.input else None
    actual_hand_mode = _resolved_hand_mode(case, profile, input_path)
    instance_ids: dict[str, Any] = {"validation_supervisor": supervisor}
    if not args.fake:
        instance_ids["source"] = str(uuid.uuid4())
        if case_id != "acquisition_live":
            instance_ids.update({
                "producer_arm": str(uuid.uuid4()),
                "coordinator_arm": str(uuid.uuid4()),
                "executor_arm": str(uuid.uuid4()),
            })
            if contract["recordable"]:
                instance_ids["recorder"] = str(uuid.uuid4())
            hand_sides = list(case["active_sides"]) if actual_hand_mode != "disabled" else []
            if hand_sides:
                producer_instances: dict[str, str] = {}
                executor_instances: dict[str, str] = {}
                for side in hand_sides:
                    executor_instances[side] = str(uuid.uuid4())
                    if actual_hand_mode == "direct" and profile in {"h5_sim", "h5_real"}:
                        producer_instances[side] = instance_ids["source"]
                    elif profile in {"joint_replay_sim", "wuji_direct_real"}:
                        producer_instances[side] = instance_ids["producer_arm"]
                    else:
                        producer_instances[side] = str(uuid.uuid4())
                instance_ids["hand_producer_instances"] = producer_instances
                instance_ids["hand_executor_instances"] = executor_instances
                instance_ids["producer_hand"] = producer_instances[hand_sides[0]]
                instance_ids["executor_hand"] = executor_instances[hand_sides[0]]
    repositories = {"teleop": git_fingerprint(ROOT), "acquisition": git_fingerprint(Path("/home/current/syz/mocap/acquisition"))}
    hashes = _hashes()
    manifest: dict[str, Any] = {
        "schema_name": BUNDLE_SCHEMA,
        "schema_version": BUNDLE_VERSION,
        "run_id": run_id,
        "case_id": case_id,
        "profile": profile,
        "source_type": _source_type(profile),
        "producer": contract["producer"],
        "required_capability": case["required_capability"],
        "active_sides": list(case["active_sides"]),
        "hand": {"sides": list(case["active_sides"] if actual_hand_mode != "disabled" else []), "mode": actual_hand_mode},
        "hand_sides": list(case["active_sides"] if actual_hand_mode != "disabled" else []),
        "hand_mode": case["hand_mode"],
        "resolved_hand_mode": actual_hand_mode,
        "robot_ip": args.robot_ip or "unrecorded",
        "robot_model": args.robot_model or "unverified",
        "motive_rigid_ids": list(args.motive_rigid_id or []),
        "h5_input_sha256": sha256_file(input_path) if input_path and input_path.is_file() else None,
        "ik_backend": backend,
        "safety": {"velocity_ratio": float(case["velocity_ratio"]), "acceleration_ratio": float(case["acceleration_ratio"])},
        "velocity_ratio": float(case["velocity_ratio"]),
        "acceleration_ratio": float(case["acceleration_ratio"]),
        "max_duration_s": float(case["max_duration_s"]),
        "repositories": repositories,
        "teleop_commit": repositories["teleop"]["commit"],
        "teleop_dirty": repositories["teleop"]["dirty"],
        "acquisition_commit": repositories["acquisition"]["commit"],
        "acquisition_dirty": repositories["acquisition"]["dirty"],
        "hashes": hashes,
        "config_sha256": hashes["config_sha256"],
        "runtime_sha256": hashes["runtime_sha256"],
        "acl_sha256": hashes["acl_sha256"],
        "router": {"endpoint": os.environ.get("TIANJI_ROUTER_ENDPOINT", DEFAULT_ROUTER_ENDPOINT), "zid": router_zid},
        "router_endpoint": os.environ.get("TIANJI_ROUTER_ENDPOINT", DEFAULT_ROUTER_ENDPOINT),
        "router_zid": router_zid,
        "publisher_instance_ids": instance_ids,
        "machine": {"hostname": socket.gethostname(), "os": platform.platform(), "python": platform.python_version()},
        "hostname": socket.gethostname(),
        "os": platform.platform(),
        "source_config": source,
        "real_preflight_file": "real-preflight.json" if contract["source_capability"] == "real" else "",
        "headless": bool(args.headless),
        "started_at": started,
        "ended_at": None,
        "exit_reason": "not_finished",
    }
    manifest["authority_contract"] = _authority_contract(manifest)
    return manifest


def _parse_event(value: str) -> tuple[str, str]:
    if "=" in value:
        event, details = value.split("=", 1)
    elif ":" in value:
        event, details = value.split(":", 1)
    else:
        event, details = value, ""
    return event.strip(), details.strip()


def _write_operator_result(path: Path, *, outcome: str, emergency_stop: bool = False, abnormal_direction: bool = False, jitter: bool = False, noise: bool = False, collision_risk: bool = False, notes: str = "") -> None:
    if outcome not in OUTCOMES:
        raise ValueError(f"invalid operator outcome: {outcome}")
    data = {
        "schema_version": BUNDLE_VERSION,
        "outcome": outcome,
        "emergency_stop": bool(emergency_stop),
        "abnormal_direction": bool(abnormal_direction),
        "jitter": bool(jitter),
        "noise": bool(noise),
        "collision_risk": bool(collision_risk),
        "notes": notes,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _managed_stop(
    bundle: Path,
    manifest: Mapping[str, Any],
    status: Any,
    args: argparse.Namespace,
    *,
    launcher_log: Path | None = None,
    process: Any | None = None,
) -> SafetyStopResult | None:
    if not args.danger_stop:
        return None
    ids = [entry["publisher_instance_id"] for entry in manifest.get("authority_contract", []) if entry.get("component_role") == "executor_arm"]
    ids += [
        entry["publisher_instance_id"]
        for entry in manifest.get("authority_contract", [])
        if entry.get("component_role") == "executor_hand"
    ]
    if not ids:
        ids = [str(manifest["publisher_instance_ids"]["executor_arm"])]
    supervisor_id = str(manifest["publisher_instance_ids"]["validation_supervisor"])
    supervisor = SafetyStopSupervisor(str(manifest["run_id"]), supervisor_id, str(manifest["router_zid"]))
    def _records(name: str) -> list[dict[str, Any]]:
        try:
            return [
                json.loads(line)
                for line in (bundle / name).read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        except (OSError, json.JSONDecodeError, TypeError):
            return []
    # A danger stop is meaningful only after the launcher has completed its
    # full component sequence.  The explicit marker avoids a fixed five-second
    # race with hand executors that are started last.
    before_status_rows = _records("status.jsonl")
    startup_ready = launcher_log is None or _wait_for_launcher_startup(
        launcher_log,
        timeout_s=_launcher_startup_timeout(manifest),
        process=process,
    )
    ready_deadline = time.monotonic() + _executor_ready_timeout(manifest)
    executor_ready = False
    if startup_ready:
        while time.monotonic() < ready_deadline:
            before_status_rows = _records("status.jsonl")
            executor_ready = all(
                _latest_ready_executor(before_status_rows, executor_id, manifest)
                for executor_id in ids
            )
            if executor_ready:
                break
            if process is not None and process.poll() is not None:
                break
            time.sleep(0.05)
    if not executor_ready:
        request = SafetyStopRequest(
            ProtocolEnvelope(1, supervisor_id, str(manifest["router_zid"]), 1, monotonic_ns()),
            str(manifest["run_id"]), args.danger_stop, True,
        )
        result = SafetyStopResult(False, True, "session is not ready; safety stop was not published", request, ())
        _write_status(
            status, event="safety_stop", component="validation", supervisor=supervisor_id,
            run_id=str(manifest["run_id"]), reason=args.danger_stop,
            expected_executor_ids=ids, acked_executor_ids=[], ack_complete=False,
            new_motion_commands_after_stop=None, lockout=False,
            executor_safety_evidence={"same_tick_ack": False, "unhealthy": False, "no_motion_commands": False},
        )
        return result
    transport: ZenohSafetyTransport | None = None
    try:
        transport = ZenohSafetyTransport(str(manifest["router"]["endpoint"]))
        result = supervisor.issue(args.danger_stop, ids, publish=transport.publish, wait_ack=lambda request: transport.wait_ack(request, ids))
    except Exception as exc:
        request = SafetyStopRequest(
            ProtocolEnvelope(1, supervisor_id, str(manifest["router_zid"]), supervisor.sequence + 1, monotonic_ns()),
            str(manifest["run_id"]), args.danger_stop, True,
        )
        supervisor.locked = True
        result = SafetyStopResult(False, True, f"safety stop transport unavailable: {exc}", request, ())
    finally:
        if transport is not None:
            transport.close()
    # Give executors at most one control tick plus scheduling margin.  A
    # fixed 200ms window would incorrectly classify several control ticks as
    # same-tick evidence.
    profile_config = _profile_config(str(manifest.get("profile", "")))
    rates: list[float] = []
    for config_name in (profile_config.get("arm_executor_config"), "executors/wuji_hand2.yaml"):
        if not config_name:
            continue
        try:
            config = yaml.safe_load((canonical_config_root() / str(config_name)).read_text(encoding="utf-8")) or {}
            rate = float(config.get("rate_hz", 0.0))
            if rate > 0.0 and math.isfinite(rate):
                rates.append(rate)
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            continue
    max_rate = max(rates, default=60.0)
    tick_window_ns = int(1.5e9 / max_rate)
    time.sleep(max(0.05, 2.0 * tick_window_ns / 1e9))
    status_rows = _records("status.jsonl")
    protocol_rows = _records("protocol.jsonl")
    request_time = int(result.request.envelope.timestamp_ns)
    executor_rows = {
        executor_id: [
            row for row in status_rows
            if str(row.get("publisher_instance_id", "")) == executor_id
            and row.get("component_role") in {"executor_arm", "executor_hand"}
        ]
        for executor_id in ids
    }
    def _status_time(row: Mapping[str, Any]) -> int | None:
        value = row.get("timestamp_ns")
        return int(value) if isinstance(value, int) and not isinstance(value, bool) else None
    def _locked(row: Mapping[str, Any]) -> bool:
        diagnostics = row.get("diagnostics")
        diagnostics = diagnostics if isinstance(diagnostics, Mapping) else {}
        return bool(
            diagnostics.get("safety_locked") is True
            or (row.get("tracking_allowed") is False and row.get("healthy") is False)
        )
    def _within(row: Mapping[str, Any]) -> bool:
        timestamp = _status_time(row)
        return timestamp is not None and request_time <= timestamp <= request_time + tick_window_ns
    unhealthy = all(any(_within(row) and row.get("healthy") is False for row in rows) for rows in executor_rows.values())
    locked = all(any(_within(row) and _locked(row) for row in rows) for rows in executor_rows.values())
    ack_times = result.ack_timestamps_ns
    same_tick_ack = bool(
        result.accepted and len(ack_times) == len(ids)
        and all(request_time <= timestamp <= request_time + tick_window_ns for timestamp in ack_times)
    )
    # Coordinator command wire traffic is not SDK motion evidence.  Require
    # each executor's command counter before/after the stop to prove no motion.
    before_counts: dict[str, int] = {}
    after_counts: dict[str, int] = {}
    for executor_id, rows in executor_rows.items():
        before = [
            int(row["diagnostics"]["commands_sent"])
            for row in before_status_rows
            if str(row.get("publisher_instance_id", "")) == executor_id
            and isinstance(row.get("diagnostics"), Mapping)
            and isinstance(row["diagnostics"].get("commands_sent"), int)
        ]
        after = [
            int(row["diagnostics"]["commands_sent"])
            for row in rows
            if _within(row)
            and isinstance(row.get("diagnostics"), Mapping)
            and isinstance(row["diagnostics"].get("commands_sent"), int)
        ]
        if before:
            before_counts[executor_id] = max(before)
        if after:
            after_counts[executor_id] = max(after)
    no_motion = bool(
        len(before_counts) == len(ids) == len(after_counts)
        and all(after_counts[item] == before_counts[item] for item in ids)
    )
    evidence = {
        "same_tick_ack": same_tick_ack,
        "unhealthy": unhealthy,
        "no_motion_commands": no_motion,
        "ack_timestamps_ns": list(ack_times),
        "request_timestamp_ns": request_time,
    }
    post_stop_motion = not no_motion
    _write_status(
        status,
        event="safety_stop",
        component="validation",
        supervisor=supervisor_id,
        run_id=str(manifest["run_id"]),
        reason=args.danger_stop,
        expected_executor_ids=ids,
        acked_executor_ids=list(result.acked_executor_ids),
        ack_complete=result.accepted,
        new_motion_commands_after_stop=post_stop_motion,
        lockout=locked,
        executor_safety_evidence=evidence,
    )
    if not result.accepted:
        _write_operator_event(
            bundle / "operator_events.jsonl",
            supervisor_id,
            str(manifest["run_id"]),
            "physical_estop_required",
            result.reason,
        )
    return result

def _run_fake(bundle: Path, manifest: dict[str, Any], status: Any, args: argparse.Namespace) -> int:
    _write_status(status, event="preflight_passed", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], mode="fake_headless", physical_validation=False)
    capture = {
        "schema_version": 1,
        "timestamp_ns": monotonic_ns(),
        "run_id": manifest["run_id"],
        "publisher_instance_id": manifest["publisher_instance_ids"]["validation_supervisor"],
        "mode": "fake_headless",
    }
    if manifest["profile"] not in {"target_replay_sim", "joint_replay_sim", "acquisition_live"} and not (bundle / "session.h5").exists():
        _create_empty_session(bundle / "session.h5", _source_type(manifest["profile"]), manifest["router_zid"])
    for raw in args.operator_event or []:
        event, details = _parse_event(raw)
        _write_operator_event(bundle / "operator_events.jsonl", manifest["publisher_instance_ids"]["validation_supervisor"], manifest["run_id"], event, details)
    if args.danger_stop:
        expected = [
            entry["publisher_instance_id"]
            for entry in manifest.get("authority_contract", [])
            if entry.get("component_role") == "executor_arm"
        ]
        expected += [
            entry["publisher_instance_id"]
            for entry in manifest.get("authority_contract", [])
            if entry.get("component_role") == "executor_hand"
        ]
        expected = list(dict.fromkeys(item for item in expected if item))
        if not expected:
            expected = ["fake_arm_executor"]
        supervisor = SafetyStopSupervisor(manifest["run_id"], manifest["publisher_instance_ids"]["validation_supervisor"], manifest["router_zid"])
        stop = supervisor.issue(args.danger_stop, expected, publish=lambda _: None, wait_ack=lambda _: {
            executor: {"schema_version": 1, "publisher_instance_id": executor, "router_zid": manifest["router_zid"], "sequence": 1, "timestamp_ns": monotonic_ns(), "executor_id": executor, "run_id": manifest["run_id"], "latched": True, "reason": args.danger_stop}
            for executor in expected
        })
        _write_status(status, event="safety_stop", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], reason=args.danger_stop, expected_executor_ids=expected, acked_executor_ids=list(stop.acked_executor_ids), ack_complete=stop.accepted, new_motion_commands_after_stop=False, lockout=True, executor_safety_evidence={"same_tick_ack": stop.accepted, "unhealthy": stop.accepted, "no_motion_commands": stop.accepted})
        if not stop.accepted:
            _write_operator_event(bundle / "operator_events.jsonl", manifest["publisher_instance_ids"]["validation_supervisor"], manifest["run_id"], "physical_estop_required", stop.reason)
    _write_status(status, event="finished", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], exit_reason="fake_headless_only", physical_validation=False)
    return 0


def _run_acquisition(bundle: Path, manifest: dict[str, Any], status: Any, args: argparse.Namespace) -> int:
    """Observe the acquisition-owned aligned stream; never manufacture a sample."""
    try:
        capture = ManagedEvidenceCapture(str(manifest["router"]["endpoint"]), bundle, manifest["run_id"])
        observation = AlignedStreamObservation(min_samples=3, min_rate_hz=50.0)

        def on_aligned(sample: Any) -> None:
            payload = capture._payload(sample)
            if payload is None:
                return
            hands = payload.get("hands", {})
            if not isinstance(hands, Mapping):
                return
            row = dict(payload)
            row["run_id"] = manifest["run_id"]
            row["topic"] = "mocap/aligned/hands"
            row["publisher_instance_id"] = str(payload.get("stream_instance_id", ""))
            row["router_zid"] = str(payload.get("router_zid", ""))
            row["left_valid"] = bool(isinstance(hands.get("left"), Mapping) and hands["left"].get("valid", False))
            row["right_valid"] = bool(isinstance(hands.get("right"), Mapping) and hands["right"].get("valid", False))
            if observation.accept(row):
                capture._append("protocol.jsonl", row)

        capture._streams.append(capture.session.declare_subscriber("mocap/aligned/hands", on_aligned))
    except Exception as exc:
        _write_status(status, event="acquisition_capture_unavailable", component="acquisition", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], healthy=False, complete=False, error=str(exc))
        return 1
    deadline = time.monotonic() + min(float(manifest["max_duration_s"]), float(args.duration))
    try:
        while time.monotonic() < deadline and not observation.complete:
            time.sleep(0.05)
    finally:
        capture.close()
    (bundle / "logs" / "acquisition.log").write_text(
        f"source=mocap/aligned/hands samples={observation.samples} stream_instance_id={observation.stream_instance_id or ''} router_zid={observation.router_zid or ''}\n",
        encoding="utf-8",
    )
    _write_status(status, event="acquisition_observation", component="acquisition", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], healthy=observation.complete, complete=observation.complete, samples=observation.samples, stream_instance_id=observation.stream_instance_id, router_zid=observation.router_zid)
    if not observation.complete:
        _write_status(status, event="acquisition_observation_missing", component="acquisition", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], healthy=False, complete=False, error="mocap/aligned/hands produced no valid sample")
        return 1
    return 0


def _run_session(bundle: Path, manifest: dict[str, Any], status: Any, args: argparse.Namespace) -> int:
    profile = manifest["profile"]
    if profile == "acquisition_live":
        return _run_acquisition(bundle, manifest, status, args)
    contract = CASE_CONTRACTS[manifest["case_id"]]
    command = ["bash", str(ROOT / "scripts" / "run_session.sh"), "--profile", profile]
    if args.headless:
        command.append("--headless")
    if contract["recordable"]:
        command += ["--record", str(bundle / "session.h5")]
    if args.input:
        command += ["--input", args.input]
    if args.confirm_real:
        command.append("--confirm-real")
    if args.extra:
        command += ["--", *args.extra]
    env = os.environ.copy()
    env.update(instance_handoff_environment(manifest))
    env.update({
        "TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID": manifest["publisher_instance_ids"]["validation_supervisor"],
        "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID": manifest["publisher_instance_ids"]["validation_supervisor"],
        "TIANJI_ROUTER_ZID": manifest["router_zid"],
        "TIANJI_VALIDATION_CASE_ID": manifest["case_id"],
        "TIANJI_VALIDATION_HAND_MODE": str(manifest.get("resolved_hand_mode") or ""),
        "TIANJI_IK_BACKEND": str(manifest.get("ik_backend") or ""),
        "TIANJI_VALIDATION_IK_BACKEND": str(manifest.get("ik_backend") or ""),
        "TIANJI_VALIDATION_PRODUCER": str(manifest.get("producer") or ""),
        "TIANJI_REAL_PREFLIGHT_FD": str(getattr(args, "real_preflight_fd", "") or ""),
        "TIANJI_REAL_PREFLIGHT_SCANNER_FD": str(getattr(args, "real_preflight_scanner_fd", "") or ""),
    })
    log_path = bundle / "logs" / "session.log"
    with log_path.open("w", encoding="utf-8") as log:
        _write_status(status, event="session_starting", component="run_session", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], command=command)
        try:
            evidence_capture: ManagedEvidenceCapture | None = ManagedEvidenceCapture(
                str(manifest["router"]["endpoint"]), bundle, manifest["run_id"]
            )
        except Exception as exc:
            evidence_capture = None
            _write_status(status, event="capture_unavailable", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], healthy=False, error=str(exc))
        pass_fds = tuple(
            int(fd) for fd in (
                getattr(args, "real_preflight_fd", -1),
                getattr(args, "real_preflight_scanner_fd", -1),
            ) if isinstance(fd, int) and fd >= 0
        )
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            pass_fds=pass_fds,
        )
        stop_result = _managed_stop(
            bundle,
            manifest,
            status,
            args,
            launcher_log=log_path,
            process=process,
        )
        deadline = time.monotonic() + float(manifest["max_duration_s"])
        exit_reason = "operator_interrupt"
        try:
            while process.poll() is None and time.monotonic() < deadline:
                if stop_result is not None:
                    exit_reason = "danger_stop"
                    process.terminate()
                    break
                time.sleep(0.2)
            if process.poll() is None:
                if stop_result is None:
                    exit_reason = "max_duration_reached"
                    process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            elif exit_reason == "operator_interrupt":
                exit_reason = f"session_exit_{process.returncode}"
        except KeyboardInterrupt:
            exit_reason = "operator_interrupt"
            process.terminate()
            process.wait(timeout=5)
        _write_status(status, event="session_finished", component="run_session", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], exit_reason=exit_reason, process_returncode=process.returncode)
        if evidence_capture is not None:
            evidence_capture.close()
    runtime_dir = Path(env.get("PICO_TIANJI_RUNTIME_DIR", os.environ.get("PICO_TIANJI_RUNTIME_DIR", "/tmp")))
    for child_log in runtime_dir.glob(f"{manifest['run_id']}-*.log"):
        destination = bundle / "logs" / f"{child_log.stem}.log"
        try:
            shutil.copy2(child_log, destination)
        except OSError:
            # Missing child log is evidence of failed capture; retain the
            # validation log and let analyzer/checksum expose what is present.
            continue
    if CASE_CONTRACTS[manifest["case_id"]]["recordable"] and not (bundle / "session.h5").exists():
        raise RuntimeError("session recorder did not produce session.h5")
    if stop_result is not None and not stop_result.accepted:
        return 1
    if process.returncode not in (0, -2, -15):
        return int(process.returncode or 1)
    return 0


def run_case(args: argparse.Namespace) -> int:
    matrix = load_matrix()
    cases = matrix["cases"]
    if args.list:
        print("\n".join(cases.keys()))
        return 0
    if not args.case:
        raise ValueError("--case is required (or use --list)")
    if args.case not in cases:
        raise ValueError(f"unknown validation case: {args.case}")
    case = cases[args.case]
    contract = build_session_contract(args.case, {"ik_backend": args.ik_backend})
    if contract["profile"] != case["profile"]:
        raise ValueError(f"matrix profile mismatch for {args.case}: {case['profile']} != {contract['profile']}")
    capability = case["required_capability"]
    if args.case in {"wuji_retarget_dry", "wuji_retarget_real"} and _h5_contains_hand_joints(Path(args.input).expanduser() if args.input else None):
        raise ValueError("retarget validation rejects H5 wuji2_joints input; use direct profile")
    if capability == "real" and not args.confirm_real:
        raise PermissionError("real validation requires explicit --confirm-real")
    prerequisite_root = Path(args.prerequisite_root or args.output).expanduser()
    prerequisites_ok, missing = _prerequisites_passed(prerequisite_root, case["prerequisites"])
    if capability == "real" and not prerequisites_ok:
        raise PermissionError("real validation prerequisites not passed: " + ", ".join(missing))
    if capability == "real" and args.fake:
        raise PermissionError("--fake cannot be used for real validation")
    if not args.fake and capability == "real" and not args.robot_ip:
        raise PermissionError("real validation requires --robot-ip during preflight")
    if not args.fake and capability == "real":
        if not args.real_preflight_file:
            raise PermissionError("real validation requires --real-preflight-file")
        attestation = Path(args.real_preflight_file).expanduser()
        if attestation.is_symlink() or not attestation.is_file():
            raise PermissionError("real preflight attestation must be a regular file")
    output_root = Path(args.output).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    started = utc_now()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"_{args.case}_{uuid.uuid4().hex[:8]}"
    bundle = output_root / run_id
    bundle.mkdir()
    (bundle / "logs").mkdir()
    supervisor = str(uuid.uuid4())
    router_zid = "fake-router-unverified" if args.fake else os.environ.get("TIANJI_ROUTER_ZID", "")
    if not router_zid:
        raise RuntimeError("router ZID unavailable; run the managed router before validation")
    preflight_binding: RealPreflightBinding | None = None
    if capability == "real":
        bound_attestation = bundle / "real-preflight.json"
        preflight_binding = _bind_real_preflight(
            Path(args.real_preflight_file).expanduser(),
            bound_attestation,
            run_id=run_id,
            supervisor=supervisor,
            router_zid=router_zid,
        )
        args.real_preflight_file = str(bound_attestation)
        args.real_preflight_nonce = preflight_binding.nonce
        args.real_preflight_digest = preflight_binding.digest
        args.real_preflight_fd = preflight_binding.attestation_fd
        args.real_preflight_scanner_fd = preflight_binding.scanner_fd
    manifest = _build_manifest(args.case, case, case["profile"], run_id, supervisor, router_zid, started, args)
    (bundle / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    (bundle / "operator_events.jsonl").touch()
    (bundle / "liveliness.jsonl").touch()
    (bundle / "protocol.jsonl").touch()
    with (bundle / "status.jsonl").open("w", encoding="utf-8") as status, (bundle / "logs" / "validation.log").open("w", encoding="utf-8") as log:
        _write_status(status, event="preflight_started", component="validation", supervisor=supervisor, run_id=run_id, required_capability=capability, required_devices=case["required_devices"], physical_validation=not args.fake)
        log.write(f"run_id={run_id}\ncase={args.case}\nmode={'fake_headless' if args.fake else 'managed_session'}\n")
        try:
            rc = _run_fake(bundle, manifest, status, args) if args.fake else _run_session(bundle, manifest, status, args)
        finally:
            if preflight_binding is not None:
                preflight_binding.close()
    manifest["ended_at"] = utc_now()
    manifest["exit_reason"] = "fake_headless_only" if args.fake else "operator_or_session_exit"
    (bundle / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    if not args.fake:
        for raw in args.operator_event or []:
            event, details = _parse_event(raw)
            _write_operator_event(bundle / "operator_events.jsonl", supervisor, run_id, event, details)
    requested_outcome = "aborted" if args.fake else (args.operator_outcome or "aborted")
    operator_event_names = [_parse_event(raw)[0] for raw in (args.operator_event or [])]
    if args.danger_stop:
        operator_event_names.append(args.danger_stop)
    try:
        requested_outcome = validate_operator_finalization(requested_outcome, operator_event_names, rc=rc)
    except ValueError as exc:
        requested_outcome = "fail"
        args.operator_notes = (args.operator_notes or "") + f" {exc}"
        rc = max(rc, 1)
    if requested_outcome in {"pass", "fail"} and not (args.operator_event or []):
        requested_outcome = "fail"
        args.operator_notes = (args.operator_notes or "") + " operator outcome requires at least one explicit operator event"
        rc = max(rc, 1)
    if requested_outcome == "pass" and rc != 0:
        requested_outcome = "fail"
        args.operator_notes = (args.operator_notes or "") + " session exited non-zero"
    _write_operator_result(
        bundle / "operator_result.yaml",
        outcome=requested_outcome,
        notes=args.operator_notes or ("No physical acceptance claim; record operator result after completing the runbook." if requested_outcome == "aborted" else "Operator recorded outcome with explicit event."),
    )
    immutable_files = {"real-preflight.json": preflight_binding.payload} if preflight_binding is not None else None
    _write_checksums(bundle, immutable_files=immutable_files)
    print(bundle)
    return rc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list fixed validation case IDs")
    parser.add_argument("--case")
    parser.add_argument("--output", default="validation-results")
    parser.add_argument("--confirm-real", action="store_true")
    parser.add_argument("--fake", action="store_true", help="bundle/preflight-only fake mode; never reports pass")
    parser.add_argument("--headless", action="store_true", help="request headless execution (required with --fake for clarity)")
    parser.add_argument("--input", "--h5", dest="input")
    parser.add_argument("--prerequisite-root")
    parser.add_argument("--robot-ip")
    parser.add_argument("--real-preflight-file", help="protected run-bound typed real-capability attestation")
    parser.add_argument("--robot-model")
    parser.add_argument("--motive-rigid-id", action="append")
    parser.add_argument("--ik-backend")
    parser.add_argument("--operator-outcome", choices=sorted(OUTCOMES), help="explicit operator result; fake mode remains aborted")
    parser.add_argument("--operator-notes", default="")
    parser.add_argument("--operator-event", "--event", action="append")
    parser.add_argument("--danger-stop", choices=sorted(DANGEROUS_STOPS), help="explicitly issue a latched danger stop; never automatic")
    parser.add_argument("--duration", type=float, default=10.0, help="acquisition observation timeout in seconds")
    parser.add_argument("--extra", nargs=argparse.REMAINDER)
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        return run_case(build_parser().parse_args(argv))
    except PermissionError as exc:
        print(f"preflight rejected: {exc}", file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, OSError, yaml.YAMLError) as exc:
        print(f"validation-run failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
