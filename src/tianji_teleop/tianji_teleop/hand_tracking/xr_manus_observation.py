"""Managed XRoboToolkit + Manus rawviz observation publisher."""
from __future__ import annotations

import argparse
from collections.abc import Mapping
import logging
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any

from ..config_loader import load_component_config, require_finite_positive
from ..protocol import topics
from ..zenoh_util import open_session, require_single_router
from .manus_environment import manus_environment
from .observation_node import _StatusReporter
from .reference_manus_process import ReferenceManusProcess
from .runtime import ObservationRuntime, ZenohObservationPublisher
from .xr_input import XR_ARM_INPUTS, XRoboToolkitClient, XrBindingConfig, XrRoboToolkitSource
from .xr_manus_runtime import manus_callback_observations
from .xr_operator import XrControllerOperatorConfig, XrControllerOperatorPublisher


LOG = logging.getLogger("xr_manus_observation")
_CONFIG_KEYS = frozenset({
    "input_profile", "observation_only", "receiver_instance_id", "print_interval_s", "xr", "manus",
})
_XR_KEYS = frozenset({
    "host", "port", "reconnect_seconds", "arm_input", "tracker_serials", "elbow_tracker_serials", "controller_sides",
    "poll_hz", "reference_frame", "tracker_frame", "controller_frame", "tracked_to_wrist_pose",
    "start", "home", "clutch", "freshness_ns", "stable_ns",
})
_MANUS_KEYS = frozenset({
    "rawviz", "user", "library_dir", "right_glove", "left_glove", "sides", "callback_capacity",
})


def _nonempty(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value.strip()


def _port(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
        raise ValueError(f"{field} must be an integer in 1..65535")
    return value


def _load_config(path: str | os.PathLike[str]) -> dict[str, Any]:
    config = load_component_config(
        path,
        allowed_keys=_CONFIG_KEYS,
        required_keys={"input_profile", "observation_only", "receiver_instance_id", "xr"},
    )
    if config["input_profile"] != "xr_manus" or config["observation_only"] is not True:
        raise ValueError("XR/Manus observation requires input_profile=xr_manus and observation_only=true")
    config["receiver_instance_id"] = _nonempty(config["receiver_instance_id"], "receiver_instance_id")
    config["print_interval_s"] = require_finite_positive(config.get("print_interval_s", 1.0), "print_interval_s")

    xr = config["xr"]
    if not isinstance(xr, Mapping) or set(xr) - _XR_KEYS:
        raise ValueError("invalid xr observation config")
    xr = dict(xr)
    xr.setdefault("host", "127.0.0.1")
    xr.setdefault("port", 60061)
    xr.setdefault("reconnect_seconds", 1.0)
    xr.setdefault("arm_input", "xr_tracker")
    xr.setdefault("tracker_serials", {"left": "190058", "right": "190600"})
    xr.setdefault("elbow_tracker_serials", {"left": "190046", "right": "190023"})
    xr.setdefault("controller_sides", {"left": "left", "right": "right"})
    xr.setdefault("poll_hz", 90.0)
    xr.setdefault("reference_frame", "xr_tracking")
    xr.setdefault("tracker_frame", "wrist_tracker")
    xr.setdefault("controller_frame", "controller")
    xr.setdefault("tracked_to_wrist_pose", {
        "left": [0, 0, 0, 0, 0, 0, 1],
        "right": [0, 0, 0, 0, 0, 0, 1],
    })
    xr.setdefault("start", {"side": "right", "control": "grip", "threshold": 0.8})
    xr.setdefault("home", {"side": "left", "control": "grip", "threshold": 0.8})
    xr.setdefault("clutch", {"side": "right", "control": "trigger", "threshold": 0.5})
    xr["host"] = _nonempty(xr["host"], "xr.host")
    xr["port"] = _port(xr["port"], "xr.port")
    xr["reconnect_seconds"] = require_finite_positive(xr["reconnect_seconds"], "xr.reconnect_seconds")
    xr["poll_hz"] = require_finite_positive(xr["poll_hz"], "xr.poll_hz")
    xr["reference_frame"] = _nonempty(xr["reference_frame"], "xr.reference_frame")
    xr["tracker_frame"] = _nonempty(xr["tracker_frame"], "xr.tracker_frame")
    xr["controller_frame"] = _nonempty(xr["controller_frame"], "xr.controller_frame")
    binding = XrBindingConfig(
        arm_input=xr["arm_input"],
        tracker_serials=xr["tracker_serials"],
        controller_sides=xr["controller_sides"],
        elbow_tracker_serials=xr["elbow_tracker_serials"],
    )
    operator = XrControllerOperatorConfig.from_mapping({
        "start": xr["start"], "home": xr["home"], "clutch": xr["clutch"],
        "freshness_ns": xr.get("freshness_ns", 200_000_000),
        "stable_ns": xr.get("stable_ns", 800_000_000),
    })
    xr["binding"] = binding
    xr["operator_config"] = operator
    config["xr"] = xr

    manus = config.get("manus")
    if manus is not None:
        if not isinstance(manus, Mapping) or set(manus) - _MANUS_KEYS:
            raise ValueError("invalid manus observation config")
        manus = dict(manus)
        manus.setdefault("rawviz", "")
        manus.setdefault("user", "")
        manus.setdefault("library_dir", None)
        manus.setdefault("right_glove", None)
        manus.setdefault("left_glove", None)
        manus.setdefault("sides", ["left", "right"])
        manus.setdefault("callback_capacity", 256)
        if not isinstance(manus["sides"], (list, tuple)) or tuple(manus["sides"]) != ("left", "right"):
            raise ValueError("manus.sides must be exactly [left, right]")
        if type(manus["callback_capacity"]) is not int or not 0 < manus["callback_capacity"] <= 65536:
            raise ValueError("manus.callback_capacity must be in 1..65536")
        for field in ("rawviz", "user"):
            if manus[field] is not None and not isinstance(manus[field], str):
                raise ValueError(f"manus.{field} must be a string")
        for field in ("library_dir", "right_glove", "left_glove"):
            if manus[field] is not None and not isinstance(manus[field], str):
                raise ValueError(f"manus.{field} must be a string or null")
        if manus["right_glove"] and manus["right_glove"] == manus["left_glove"]:
            raise ValueError("manus glove bindings must differ")
        config["manus"] = manus
    return config


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="XRoboToolkit + Manus observation publisher")
    parser.add_argument("--config", required=True)
    parser.add_argument("--arm-input", choices=tuple(sorted(XR_ARM_INPUTS)))
    parser.add_argument("--manus-rawviz")
    parser.add_argument("--manus-user")
    parser.add_argument("--manus-library-dir")
    parser.add_argument("--right-glove")
    parser.add_argument("--left-glove")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--no-manus", action="store_true")
    parser.add_argument("--print", dest="print_frames", action="store_true")
    return parser


def _config_from_args(args: argparse.Namespace) -> dict[str, Any]:
    config = _load_config(args.config)
    if args.arm_input is not None:
        xr = dict(config["xr"])
        xr["arm_input"] = args.arm_input
        xr["binding"] = XrBindingConfig(
            arm_input=args.arm_input,
            tracker_serials=xr["tracker_serials"],
            controller_sides=xr["controller_sides"],
            elbow_tracker_serials=xr["elbow_tracker_serials"],
        )
        config["xr"] = xr
    if not args.no_manus:
        manus = dict(config.get("manus") or {})
        for argument, field in ((args.manus_rawviz, "rawviz"), (args.manus_user, "user"),
                                (args.manus_library_dir, "library_dir"),
                                (args.right_glove, "right_glove"), (args.left_glove, "left_glove")):
            if argument is not None:
                manus[field] = argument
        config["manus"] = manus
    return config


def validate_manus_runtime(manus: Mapping[str, Any]) -> dict[str, Any]:
    """Resolve and validate the external rawviz/SDK asset bundle.

    The canonical repository config intentionally contains no machine-local
    path.  A caller supplies the rawviz checkout at launch, and this function
    verifies the layout that the reference rawviz executable uses before any
    child process is started.
    """
    if not isinstance(manus, Mapping):
        raise ValueError("Manus runtime config must be a mapping")
    rawviz_value = manus.get("rawviz")
    if not isinstance(rawviz_value, str) or not rawviz_value.strip():
        raise ValueError("Manus rawviz path must be supplied with --manus-rawviz")
    rawviz = Path(rawviz_value).expanduser().resolve()
    if not rawviz.is_file() or not os.access(rawviz, os.X_OK):
        raise ValueError(f"Manus rawviz is missing or not executable: {rawviz}")
    user = manus.get("user")
    if not isinstance(user, str) or not user.isalnum():
        raise ValueError("Manus user must be supplied with --manus-user and be alphanumeric")
    calibration = rawviz.parent / "calibration"
    for side in ("Left", "Right"):
        path = calibration / f"{user}{side}MetaglovePro.mcal"
        if not path.is_file():
            raise ValueError(f"Manus calibration is missing: {path}")
    library_dir_value = manus.get("library_dir")
    library_dir = (
        Path(library_dir_value).expanduser().resolve()
        if isinstance(library_dir_value, str) and library_dir_value.strip()
        else rawviz.parent / "ManusSDK" / "lib"
    )
    library = library_dir / "libManusSDK_Integrated.so"
    if not library.is_file():
        raise ValueError(
            f"Manus SDK library is missing: {library}; supply --manus-library-dir when using a custom layout"
        )
    return {"rawviz": rawviz, "user": user, "library_dir": library_dir, "library": library}


def _make_manus_process(
    manus: Mapping[str, Any], *, receiver_instance_id: str, raw_line_sink: Any,
) -> ReferenceManusProcess:
    """Start the reference rawviz parser with its assets as the cwd.

    Some rawviz builds resolve ``calibration/`` and other companion assets
    relative to the process working directory.  Keeping that directory at the
    external rawviz checkout makes the launcher independent of the current
    teleop repository, while ``manus_environment`` still scopes the shared
    library path to this child only.
    """
    assets = validate_manus_runtime(manus)
    environment, _ = manus_environment(
        assets["rawviz"], library_dir=assets["library_dir"]
    )
    return ReferenceManusProcess(
        command=[str(assets["rawviz"]), "--user", assets["user"]],
        receiver_instance_id=receiver_instance_id,
        sides=("left", "right"),
        capacity=manus["callback_capacity"],
        cwd=str(assets["rawviz"].parent),
        env=environment,
        right_glove=manus.get("right_glove"),
        left_glove=manus.get("left_glove"),
        raw_line_sink=raw_line_sink,
    )


def _make_operator_publisher(
    *, publisher_instance_id: str, epoch: int,
    config: XrControllerOperatorConfig,
) -> XrControllerOperatorPublisher:
    """Bind controller observations to the publisher identity of this run.

    The XR receiver identity is intentionally independent from the Zenoh
    publisher identity.  The target-side admission check authorizes the
    latter, so using the YAML receiver ID here would make all controller
    observations look foreign in a managed session.
    """
    return XrControllerOperatorPublisher.from_config(
        source_instance_id=publisher_instance_id,
        epoch=epoch,
        config=config,
    )


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.duration is not None and (args.duration <= 0 or not __import__("math").isfinite(args.duration)):
        raise ValueError("duration must be finite and positive")
    config = _config_from_args(args)
    stop_event = threading.Event()
    old_handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in old_handlers:
        signal.signal(signum, lambda *_: stop_event.set())
    session = None
    manus_process = None
    source = None
    duration_thread = None
    try:
        session = open_session()
        router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID") or None)
        publisher_instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", config["receiver_instance_id"])
        zenoh_publisher = ZenohObservationPublisher(session)
        runtime = ObservationRuntime(
            publish=zenoh_publisher,
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            run_id=os.environ.get("TIANJI_RUN_ID") or None,
        )
        status = _StatusReporter(
            zenoh_publisher,
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            profile="xr_manus",
        )
        status.publish(phase="waiting_for_input", ready=False, healthy=True, force=True)

        xr = config["xr"]
        binding: XrBindingConfig = xr["binding"]
        source = XrRoboToolkitSource(
            client=XRoboToolkitClient(xr["host"], xr["port"]),
            binding=binding,
            receiver_instance_id=config["receiver_instance_id"],
        )
        operator_publisher = None
        rawviz_line_sequence = 0

        def raw_line_sink(text: str, timestamp_ns: int) -> None:
            nonlocal rawviz_line_sequence
            rawviz_line_sequence += 1
            runtime.publish_manus_rawviz_line(
                text,
                line_sequence=rawviz_line_sequence,
                received_timestamp_ns=timestamp_ns,
            )

        if not args.no_manus:
            manus = config.get("manus") or {}
            manus_process = _make_manus_process(
                manus,
                receiver_instance_id=config["receiver_instance_id"] + "-manus",
                raw_line_sink=raw_line_sink,
            )

        if args.duration is not None:
            duration_thread = threading.Thread(target=lambda: (stop_event.wait(args.duration), stop_event.set()), daemon=True)
            duration_thread.start()

        period = 1.0 / float(xr["poll_hz"])
        last_report = time.monotonic()
        xr_frames = 0
        manus_frames = 0

        def publish_manus_callback(callback) -> None:
            nonlocal manus_frames
            runtime.publish_manus_callback(callback)
            observations = manus_callback_observations(callback)
            for observation in observations.values():
                runtime.publish_hand_observation(observation)
            manus_frames += 1

        def drain_manus_callbacks() -> None:
            if manus_process is None:
                return
            for callback in manus_process.drain_pending():
                publish_manus_callback(callback)

        while not stop_event.is_set():
            if not source.initialized:
                if not source.initialize():
                    status.publish(phase="waiting_for_input", ready=False, healthy=True,
                                   error="XRoboToolkit SDK is not connected")
                    stop_event.wait(float(xr["reconnect_seconds"]))
                    continue
                operator_publisher = _make_operator_publisher(
                    publisher_instance_id=publisher_instance_id,
                    epoch=source.connection_generation,
                    config=xr["operator_config"],
                )
                status.publish(phase="receiving", ready=True, healthy=True, force=True)
            try:
                frame = source.read_frame()
                runtime.ingest_xr(
                    frame,
                    binding,
                    tracked_frame=(xr["controller_frame"] if binding.arm_input == "xr_controller"
                                   else xr["tracker_frame"]),
                    reference_frame=xr["reference_frame"],
                )
                for payload in operator_publisher.payloads(frame, router_zid=router_zid):
                    zenoh_publisher(topics.XR_OPERATOR_OBSERVATION, payload)
                xr_frames += 1
            except (RuntimeError, OSError, ValueError) as exc:
                LOG.warning("XR input read failed; reconnecting: %s", exc)
                source.close()
                status.publish(phase="degraded", ready=bool(xr_frames), healthy=False, error=str(exc), force=True)
                stop_event.wait(float(xr["reconnect_seconds"]))
                continue

            if manus_process is not None:
                if manus_process.failure:
                    raise RuntimeError(manus_process.failure)
                while True:
                    callback = manus_process.try_read()
                    if callback is None:
                        break
                    publish_manus_callback(callback)
            if args.print_frames and time.monotonic() - last_report >= float(config["print_interval_s"]):
                print(f"[xr_manus_observation] xr_frames={xr_frames} manus_frames={manus_frames}", flush=True)
                last_report = time.monotonic()
            stop_event.wait(period)
        return 0
    finally:
        stop_event.set()
        if duration_thread is not None:
            duration_thread.join(timeout=1.0)
        if manus_process is not None:
            # Stop rawviz first, then publish callbacks already accepted by
            # the parser.  Without this final drain, the audit contains the
            # source line while the callback dataset misses its parsed result.
            manus_process.close(clear_pending=False)
            drain_manus_callbacks()
        if source is not None:
            source.close()
        if session is not None:
            session.close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


__all__ = [
    "_config_from_args", "_load_config", "_make_manus_process",
    "_make_operator_publisher", "main",
    "validate_manus_runtime",
]


if __name__ == "__main__":
    raise SystemExit(main())
