"""CLI for the receive-only Manus/PICO observation profile."""
from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any

from ..config_loader import load_component_config, require_finite_positive
from ..protocol import topics
from ..protocol.messages import ComponentStatus
from ..zenoh_util import ZenohJsonSub, open_session, require_single_router
from ..recording.session_h5 import EXTENDED_SCHEMA_VERSION, SessionH5Writer
from .runtime import (
    LegacyPicoUdpReceiver,
    ObservationRuntime,
    PicoTcpReceiver,
    ZenohObservationPublisher,
)


LOG = logging.getLogger("hand_tracking_observation")
_CONFIG_KEYS = frozenset({"input_profile", "observation_only", "receiver_instance_id", "pico", "manus", "legacy_pico", "print_interval_s"})
_PICO_KEYS = frozenset({"host", "port", "reconnect_seconds", "adb_path", "adb_serial", "adb_device_port", "auto_adb_forward"})
_MANUS_KEYS = frozenset({"topics"})
_LEGACY_PICO_KEYS = frozenset({"host", "port", "recv_timeout_seconds", "use_corrected_skeleton"})


def _port(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
        raise ValueError(f"{field} must be an integer in 1..65535")
    return value


def _host(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _load_config(path: str | os.PathLike[str]) -> dict[str, Any]:
    config = load_component_config(path, allowed_keys=_CONFIG_KEYS, required_keys={"input_profile", "observation_only", "receiver_instance_id"})
    if config["observation_only"] is not True:
        raise ValueError("observation_only must be exactly true")
    if config["input_profile"] not in {"pico", "manus"}:
        raise ValueError("input_profile must be exactly 'pico' or 'manus'")
    if not isinstance(config["receiver_instance_id"], str) or not config["receiver_instance_id"]:
        raise ValueError("receiver_instance_id must be a non-empty string")
    if "pico" in config:
        if not isinstance(config["pico"], dict) or set(config["pico"]) - _PICO_KEYS:
            raise ValueError("invalid pico observation config")
    if "manus" in config:
        if not isinstance(config["manus"], dict) or set(config["manus"]) - _MANUS_KEYS:
            raise ValueError("invalid manus observation config")
    if "legacy_pico" in config:
        if not isinstance(config["legacy_pico"], dict) or set(config["legacy_pico"]) - _LEGACY_PICO_KEYS:
            raise ValueError("invalid legacy_pico observation config")
    inactive_keys = ("manus", "legacy_pico") if config["input_profile"] == "pico" else ("pico",)
    if any(key in config for key in inactive_keys):
        inactive = ", ".join(inactive_keys)
        raise ValueError(
            f"input_profile={config['input_profile']!r} cannot include inactive config section(s): {inactive}"
        )
    interval = require_finite_positive(config.get("print_interval_s", 1.0), "print_interval_s")
    config["print_interval_s"] = interval
    if config["input_profile"] == "pico":
        pico = dict(config.get("pico", {}))
        pico.setdefault("host", "127.0.0.1")
        pico.setdefault("port", 10002)
        pico.setdefault("reconnect_seconds", 1.0)
        pico.setdefault("adb_path", "adb")
        pico.setdefault("adb_serial", None)
        pico.setdefault("adb_device_port", None)
        pico.setdefault("auto_adb_forward", True)
        pico["host"] = _host(pico["host"], "pico.host")
        pico["port"] = _port(pico["port"], "pico.port")
        pico["reconnect_seconds"] = require_finite_positive(pico["reconnect_seconds"], "pico.reconnect_seconds")
        pico["adb_path"] = _host(pico["adb_path"], "pico.adb_path")
        if pico["adb_serial"] is not None:
            pico["adb_serial"] = _host(pico["adb_serial"], "pico.adb_serial")
        if pico["adb_device_port"] is not None:
            pico["adb_device_port"] = _port(pico["adb_device_port"], "pico.adb_device_port")
        if not isinstance(pico["auto_adb_forward"], bool):
            raise ValueError("pico.auto_adb_forward must be boolean")
        config["pico"] = pico
    else:
        manus = dict(config.get("manus", {}))
        topics = manus.setdefault("topics", {
            "left": "manus/raw_skeleton/left_hand",
            "right": "manus/raw_skeleton/right_hand",
        })
        if not isinstance(topics, dict) or set(topics) != {"left", "right"} or not all(isinstance(value, str) and value for value in topics.values()):
            raise ValueError("manus.topics must map left/right to non-empty Zenoh keys")
        config["manus"] = manus
        legacy_pico = dict(config.get("legacy_pico", {}))
        legacy_pico.setdefault("host", "127.0.0.1")
        legacy_pico.setdefault("port", 15000)
        legacy_pico.setdefault("recv_timeout_seconds", 1.0)
        legacy_pico.setdefault("use_corrected_skeleton", True)
        legacy_pico["host"] = _host(legacy_pico["host"], "legacy_pico.host")
        legacy_pico["port"] = _port(legacy_pico["port"], "legacy_pico.port")
        legacy_pico["recv_timeout_seconds"] = require_finite_positive(
            legacy_pico["recv_timeout_seconds"], "legacy_pico.recv_timeout_seconds"
        )
        if not isinstance(legacy_pico["use_corrected_skeleton"], bool):
            raise ValueError("legacy_pico.use_corrected_skeleton must be boolean")
        config["legacy_pico"] = legacy_pico
    return config


class _RateReporter:
    def __init__(self, interval_s: float) -> None:
        self.interval_s = interval_s
        self.started = time.monotonic()
        self.last_report = self.started
        self.frames = 0

    def observe(self, label: str) -> None:
        self.frames += 1
        now = time.monotonic()
        if now - self.last_report < self.interval_s:
            return
        rate = self.frames / max(now - self.started, 1.0e-9)
        print(f"[observation] profile={label} frames={self.frames} rate={rate:.1f}Hz control=disabled", flush=True)
        self.last_report = now


class _StatusReporter:
    """Publish source health without creating any control authority."""

    def __init__(self, publish: Any, *, publisher_instance_id: str, router_zid: str, profile: str, clock: Any = time.monotonic_ns) -> None:
        if not callable(publish) or not publisher_instance_id or not router_zid:
            raise ValueError("status reporter requires a publisher and identities")
        self._publish = publish
        self._publisher_instance_id = publisher_instance_id
        self._router_zid = router_zid
        self._profile = profile
        self._clock = clock
        self._lock = threading.Lock()
        self._sequence = 0
        self._last_publish_monotonic = 0.0
        self._frames: dict[str, int] = {}
        self._last_received_timestamp_ns: int | None = None
        self._last_error: str | None = None

    def publish(self, *, phase: str, ready: bool, healthy: bool, error: str | None = None, force: bool = False) -> None:
        now_monotonic = time.monotonic()
        with self._lock:
            if not force and now_monotonic - self._last_publish_monotonic < 0.5:
                return
            self._sequence += 1
            self._last_publish_monotonic = now_monotonic
            status = ComponentStatus(
                schema_version=1,
                sequence=self._sequence,
                timestamp_ns=int(self._clock()),
                component_role="source",
                component_id="hand_tracking_observation",
                phase=phase,
                ready=bool(ready),
                healthy=bool(healthy),
                capabilities=["simulation"],
                error=error,
                diagnostics={
                    "observation_only": True,
                    "input_profile": self._profile,
                    "frames_by_stream": dict(self._frames),
                    "last_received_timestamp_ns": self._last_received_timestamp_ns,
                    "last_error": self._last_error,
                },
                publisher_instance_id=self._publisher_instance_id,
                router_zid=self._router_zid,
            )
            self._publish(topics.SOURCE_STATUS, status.to_dict())

    def frame(self, stream: str, received_timestamp_ns: int) -> None:
        with self._lock:
            self._frames[stream] = self._frames.get(stream, 0) + 1
            self._last_received_timestamp_ns = int(received_timestamp_ns)
            self._last_error = None
            ready = True
        self.publish(phase="receiving", ready=ready, healthy=True)

    def error(self, error: Exception) -> None:
        message = str(error) or type(error).__name__
        with self._lock:
            self._last_error = message
            ready = bool(self._frames)
        self.publish(phase="degraded", ready=ready, healthy=False, error=message, force=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Receive-only Manus/PICO hand tracking publisher")
    parser.add_argument("--config", required=True, help="canonical hand-tracking observation YAML")
    parser.add_argument("--record", help="optional schema-1.1 session HDF5 path")
    parser.add_argument("--no-adb-forward", action="store_true", help="PICO: use an externally managed TCP forward")
    parser.add_argument(
        "--suppress-status",
        action="store_true",
        help="do not publish source status (used when a target bridge owns source authority)",
    )
    parser.add_argument("--print", dest="print_frames", action="store_true", help="print receive rate periodically")
    parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds (diagnostic/testing)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = _load_config(args.config)
    required_profile = os.environ.get("TIANJI_REQUIRED_OBSERVATION_PROFILE")
    if required_profile is not None and config["input_profile"] != required_profile:
        raise ValueError(
            f"observation entry requires input_profile={required_profile!r}, "
            f"got {config['input_profile']!r}"
        )
    if args.duration is not None and (args.duration <= 0.0 or not __import__("math").isfinite(args.duration)):
        raise ValueError("duration must be finite and positive")
    if args.record and Path(args.record).exists():
        raise FileExistsError(f"refusing to overwrite existing recording: {args.record}")

    stop_event = threading.Event()
    old_handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    for signum in old_handlers:
        signal.signal(signum, request_stop)

    session: Any = None
    zenoh_publisher: ZenohObservationPublisher | None = None
    status_reporter: _StatusReporter | None = None
    writer: SessionH5Writer | None = None
    resources: list[Any] = []
    receivers: list[Any] = []
    receiver_threads: list[threading.Thread] = []
    duration_thread: threading.Thread | None = None
    normal_exit = False
    try:
        session = open_session()
        router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID") or None)
        zenoh_publisher = ZenohObservationPublisher(session)
        if not args.suppress_status:
            status_reporter = _StatusReporter(
                zenoh_publisher,
                publisher_instance_id=os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "hand-tracking-observation"),
                router_zid=router_zid,
                profile=config["input_profile"],
            )
            status_reporter.publish(phase="waiting_for_input", ready=False, healthy=True, force=True)
        if args.record:
            use_corrected_legacy_palm = bool(
                config.get("legacy_pico", {}).get("use_corrected_skeleton", True)
            )
            if config["input_profile"] == "pico":
                mapping_versions = {
                    "pico_hand": "pico26_to_mediapipe21_v1",
                    "pico_arm": "pico_head_current_v1",
                }
                reference_frames = {
                    "pico_hand": "pico_tracking_initial_wrist_relative",
                    "pico_arm": "pico_head_current",
                }
            else:
                mapping_versions = {
                    "manus_hand": "manus25_to_mediapipe21_v1",
                    "legacy_pico_arm": (
                        "legacy_pico_corrected_palm_v1"
                        if use_corrected_legacy_palm
                        else "legacy_pico_pose_v1"
                    ),
                }
                reference_frames = {
                    "manus_hand": "manus_local_vuh_y_flipped_wrist_relative",
                    "legacy_pico_arm": "legacy_pico_tracking",
                }
            writer = SessionH5Writer(
                args.record,
                source_type="hand_tracking_observation",
                robot_model=os.environ.get("TIANJI_ROBOT_MODEL", "observation_only"),
                router_zid=router_zid,
                schema_version=EXTENDED_SCHEMA_VERSION,
                metadata={
                    "observation_only": True,
                    "input_profile": config["input_profile"],
                    "receiver_instance_id": config["receiver_instance_id"],
                    "mapping_versions": mapping_versions,
                    "reference_frames": reference_frames,
                    "config": config,
                },
            )
        runtime = ObservationRuntime(
            publish=zenoh_publisher,
            publisher_instance_id=os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "hand-tracking-observation"),
            router_zid=router_zid,
            session_writer=writer,
        )
        reporter = _RateReporter(config["print_interval_s"])

        def on_error(error: Exception) -> None:
            LOG.warning("observation receiver error: %s", error)
            if status_reporter is not None:
                status_reporter.error(error)

        if args.duration is not None:
            def stop_after_duration() -> None:
                if stop_event.wait(args.duration):
                    return
                stop_event.set()

            duration_thread = threading.Thread(
                target=stop_after_duration,
                name="hand-tracking-duration",
                daemon=True,
            )
            duration_thread.start()

        if config["input_profile"] == "pico":
            pico = config["pico"]
            receiver = PicoTcpReceiver(
                host=pico["host"],
                port=int(pico["port"]),
                receiver_instance_id=config["receiver_instance_id"],
                reconnect_seconds=float(pico["reconnect_seconds"]),
                adb_path=pico["adb_path"],
                adb_serial=pico["adb_serial"],
                adb_device_port=pico["adb_device_port"],
                auto_adb_forward=bool(pico["auto_adb_forward"]) and not args.no_adb_forward,
                on_error=on_error,
            )

            def on_pico(frame: Any) -> None:
                runtime.ingest_pico(frame)
                if status_reporter is not None:
                    status_reporter.frame("pico", frame.received_timestamp_ns)
                if args.print_frames:
                    reporter.observe("pico")

            receiver.stop_event = stop_event
            receivers.append(receiver)
            receiver.run(on_pico)
            normal_exit = True
        else:
            manus_topics = config["manus"]["topics"]

            def on_manus(payload: Any) -> None:
                try:
                    observation = runtime.ingest_manus_payload(
                        payload,
                        receiver_instance_id=config["receiver_instance_id"],
                    )
                    if status_reporter is not None:
                        status_reporter.frame("manus", observation.received_timestamp_ns)
                    if args.print_frames:
                        reporter.observe("manus")
                except Exception:
                    LOG.exception("invalid Manus observation payload")
                    if status_reporter is not None:
                        status_reporter.error(ValueError("invalid Manus observation payload"))

            legacy_config = config["legacy_pico"]

            def on_legacy_pico(frame: Any) -> None:
                try:
                    runtime.ingest_legacy_palm(
                        frame,
                        use_corrected_skeleton=bool(legacy_config["use_corrected_skeleton"]),
                    )
                    if status_reporter is not None:
                        status_reporter.frame("legacy_pico_palm", frame.received_timestamp_ns)
                    if args.print_frames:
                        reporter.observe("manus+legacy_pico")
                except Exception:
                    LOG.exception("invalid legacy PICO observation frame")
                    if status_reporter is not None:
                        status_reporter.error(ValueError("invalid legacy PICO observation frame"))

            for side in ("left", "right"):
                resources.append(ZenohJsonSub(session, manus_topics[side], on_manus))
            legacy_receiver = LegacyPicoUdpReceiver(
                host=legacy_config["host"],
                port=int(legacy_config["port"]),
                receiver_instance_id=config["receiver_instance_id"],
                recv_timeout_seconds=float(legacy_config["recv_timeout_seconds"]),
                on_error=on_error,
            )
            legacy_receiver.stop_event = stop_event
            receivers.append(legacy_receiver)
            legacy_thread = threading.Thread(
                target=legacy_receiver.run,
                args=(on_legacy_pico,),
                name="legacy-pico-udp-receiver",
                daemon=True,
            )
            receiver_threads.append(legacy_thread)
            legacy_thread.start()
            while not stop_event.wait(0.25):
                pass
            normal_exit = True
    finally:
        stop_event.set()
        for receiver in receivers:
            try:
                receiver.stop()
            except Exception:
                pass
        for thread in receiver_threads:
            thread.join(timeout=2.0)
        if duration_thread is not None:
            duration_thread.join(timeout=2.0)
        if status_reporter is not None:
            try:
                status_reporter.publish(
                    phase="stopped" if normal_exit else "fault",
                    ready=False,
                    healthy=normal_exit,
                    error=None if normal_exit else "observation process exited before normal shutdown",
                    force=True,
                )
            except Exception:
                # Status is diagnostic only; a telemetry failure must not
                # prevent receiver shutdown or leave an HDF5 file open.
                LOG.exception("failed to publish final observation status")
        for resource in resources:
            try:
                resource.close()
            except Exception:
                pass
        if zenoh_publisher is not None:
            zenoh_publisher.close()
        if writer is not None:
            if normal_exit:
                writer.close()
            else:
                writer.abort()
        if session is not None:
            session.close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return 0


__all__ = ["main"]


if __name__ == "__main__":
    raise SystemExit(main())
