"""Opt-in live adapter for the native C++ session scheduler.

The device-facing PICO/TJVR and Manus acquisition processes remain unchanged.
This module is deliberately a cold-path bridge around ``NativeGatewayProcess``:
the C++ child owns TJVR ingress, the fixed-rate control loop, IK, MuJoCo and
the lifecycle state machine. Python supervises processes and terminal actions;
optional compatibility consumers translate snapshots only in the arms-only route.

The explicit bilateral Manus route requires native publication, recording and
viewer consumers. The Python Spark+Manus and PICO2 defaults remain unchanged.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import select
import signal
import subprocess
import threading
import time
import termios
import tty
from typing import Any, Mapping


from ...coordination.live_domain_guard import LiveDomainGuard
from ...hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from ...hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame
from ...protocol import topics
from ...protocol.bilateral import ArmBilateralCommand
from ...protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    ArmJointCommand,
    ArmJointState,
    ComponentStatus,
    LatchedBool,
    SessionState,
)
from ...recording.live_capture import LiveCycleSnapshot
from .coordinator_cycle import CycleResult
from .live_output import AsyncLiveOutput
from .native_gateway import (
    GatewayFrame,
    NativeGatewayProcess,
    build_native_gateway_manifest,
    write_native_gateway_manifest,
)


_FRAME_QUEUE_TIMEOUT_S = 0.1
_NATIVE_CLOSE_TIMEOUT_S = 10.0


def _check_native_health(*supervisors: Any) -> None:
    for supervisor in supervisors:
        if supervisor.failure:
            raise RuntimeError(supervisor.failure)


def _finish_native_outputs(consumer: Any, gateway: Any, guard: Any, output: Any) -> str | None:
    # A wire completion only means frames were enqueued, not consumed/published.
    consumer.close()
    output.close()
    process_failure = None
    if gateway.process is None:
        process_failure = 'native gateway process missing at completion'
    else:
        try:
            code = gateway.process.wait(timeout=_NATIVE_CLOSE_TIMEOUT_S)
            if code != 0:
                process_failure = f'native gateway exit status {code}'
        except subprocess.TimeoutExpired:
            process_failure = 'native gateway exit timeout'
    return (consumer.failure or gateway.failure or guard.failure or output.failure
            or process_failure or
            (None if consumer.home_return_completed else 'native Home completion not confirmed'))


def _authority(value: Mapping[str, Any]) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise ValueError("native authority must be an object")
    result = {
        "logical_id": value.get("logical"),
        "publisher_instance_id": value.get("instance"),
        "router_zid": value.get("router"),
    }
    if any(not isinstance(item, str) or not item for item in result.values()):
        raise ValueError("native authority fields must be nonempty strings")
    return result


def _command_mode(state: str) -> str:
    if state == "idle":
        return "idle"
    if state == "teleop":
        return "teleop"
    return "returning"


def _safe_reason(value: Any) -> str:
    return value if isinstance(value, str) and value else "native session"


def _frame_sample(cycle: Mapping[str, Any], *, receiver_instance_id: str):
    """Recreate the existing Python diagnostic envelope from native bytes.

    This is only for the passive overlay/recording boundary.  Native C++ has
    already decoded and gated the same packet before it appears in a cycle.
    A diagnostic parser mismatch is ignored by the caller; it must never
    manufacture a control input or stop an otherwise healthy arm session.
    """
    request = cycle.get("request")
    if not isinstance(request, Mapping):
        return None
    packet = request.get("packet")
    if not isinstance(packet, (bytes, bytearray)):
        return None
    source = cycle.get("source")
    revision = request.get("source_sequence")
    if type(revision) is not int or revision <= 0:
        revision = source.get("revision") if isinstance(source, Mapping) else 0
    if type(revision) is not int or revision <= 0:
        revision = 1
    received_ns = request.get("received_ns")
    if type(received_ns) is not int or received_ns <= 0:
        return None
    observation = parse_reference_tjvr_packet(
        bytes(packet),
        receiver_instance_id=receiver_instance_id,
        receiver_frame_sequence=revision,
        received_timestamp_ns=received_ns,
    )
    sample = ReceivedTjvrFrame(
        observation,
        bool(request.get("discontinuity", False)),
        int(request.get("generation", 0)),
    )
    return sample


def _typed_cycle(cycle: Mapping[str, Any], *, manifest: Mapping[str, Any],
                prefix: str, sample: ReceivedTjvrFrame | None) -> LiveCycleSnapshot:
    """Build the established Python recording/publication snapshot."""
    timestamp_ns = cycle["timestamp_ns"]
    state = cycle["state"]
    state_epoch = cycle["state_epoch"]
    authorities = {
        name: _authority(manifest[key])
        for name, key in (
            ("source", "source_authority"),
            ("producer", "producer_authority"),
            ("coordinator", "coordinator_authority"),
            ("executor", "executor_authority"),
        )
    }
    source = cycle["source"]
    result = cycle.get("result")
    command_values = cycle.get("command")
    feedback_values = cycle.get("feedback")
    if not isinstance(command_values, Mapping) or not isinstance(feedback_values, Mapping):
        raise ValueError("native cycle is missing bilateral command/feedback")

    result_tick = result.get("tick_id") if isinstance(result, Mapping) else None
    result_sequence = result.get("applied_sequence") if isinstance(result, Mapping) else None
    if type(result_tick) is not int or result_tick <= 0:
        result_tick = None
    if type(result_sequence) is not int or result_sequence < 0:
        result_sequence = None
    proposal_sequence = result_tick if state == "teleop" and result_tick is not None else None
    mode = _command_mode(state)
    command_sequence = cycle["ticks"]
    commands: dict[str, ArmJointCommand] = {}
    for side in ("left", "right"):
        values = command_values.get(side)
        if not isinstance(values, list) or len(values) != 7:
            raise ValueError("native cycle command has invalid side shape")
        commands[side] = ArmJointCommand(
            1, command_sequence, timestamp_ns, "coordinator", side, mode,
            proposal_sequence, result_sequence if proposal_sequence is not None else None,
            list(ARM_JOINT_NAMES[side]), values,
            authorities["coordinator"]["publisher_instance_id"],
            authorities["coordinator"]["router_zid"],
        )

    feedback = []
    for side in ("left", "right"):
        values = feedback_values.get(side)
        if not isinstance(values, list) or len(values) != 7:
            raise ValueError("native cycle feedback has invalid side shape")
        feedback.extend(values)
    arm_state = ArmJointState(
        1, command_sequence, timestamp_ns, "mujoco", list(ALL_ARM_JOINT_NAMES),
        feedback, None, authorities["executor"]["publisher_instance_id"],
        authorities["executor"]["router_zid"],
    )
    session_state = SessionState(
        1, command_sequence, timestamp_ns, state, _safe_reason(cycle.get("reason")),
        "coordinator", None, authorities["coordinator"]["publisher_instance_id"],
        authorities["coordinator"]["router_zid"],
    )

    healthy = state != "fault" and not cycle.get("capture_failed", False)
    source_ready = bool(source.get("accepted") or source.get("skeleton_valid") or healthy)
    source_status = ComponentStatus(
        1, max(0, int(source.get("sequence", 0))), timestamp_ns, "source", "tjvr",
        "ready" if healthy else "fault", source_ready and healthy, healthy,
        ["simulation"], None if healthy else _safe_reason(cycle.get("reason")),
        {"native_scheduler": "cpp", "accepted": bool(source.get("accepted")),
         "skeleton_valid": bool(source.get("skeleton_valid")),
         "rotations_valid": bool(source.get("rotations_valid")),
         "source_revision": source.get("revision", 0)},
        authorities["source"]["publisher_instance_id"], authorities["source"]["router_zid"],
    )
    producer_id = authorities["producer"]["logical_id"]
    producer_status = ComponentStatus(
        1, command_sequence, timestamp_ns, "producer_arm", producer_id,
        "ready" if healthy and (result is not None or source_ready) else "waiting_input",
        healthy and (result is not None or source_ready), healthy, ["simulation"],
        None if healthy else _safe_reason(cycle.get("reason")),
        {"algorithm": manifest["algorithm"], "state_source": "model_reference",
         "native_scheduler": "cpp", "native_ticks": result_tick or 0,
         "input_live": bool(result and result.get("input_live"))},
        authorities["producer"]["publisher_instance_id"], authorities["producer"]["router_zid"],
    )
    executor_status = ComponentStatus(
        1, command_sequence, timestamp_ns, "executor_arm", "mujoco", state,
        healthy, healthy, ["simulation"], None if healthy else _safe_reason(cycle.get("reason")),
        {"native_scheduler": "cpp", "model": manifest["model"]},
        authorities["executor"]["publisher_instance_id"], authorities["executor"]["router_zid"],
    )

    bilateral = ArmBilateralCommand(
        manifest["run_id"], state_epoch, proposal_sequence, commands["left"], commands["right"]
    ).to_dict()
    coordinator_receipt = None
    if result is not None:
        coordinator_receipt = {
            "schema_version": 1,
            "kind": "arm_bilateral_receipt",
            "run_id": manifest["run_id"],
            "execution_epoch": state_epoch,
            "tick_id": result["tick_id"],
            "timestamp_ns": timestamp_ns,
            "publisher_instance_id": authorities["coordinator"]["publisher_instance_id"],
            "router_zid": authorities["coordinator"]["router_zid"],
            "stage": "coordinator_command",
            "accepted": bool(cycle.get("ik_adopted")),
            "reason": "accepted" if cycle.get("ik_adopted") else _safe_reason(cycle.get("reason")),
            "command_position_rad": {
                side: list(commands[side].position_rad) for side in ("left", "right")
            },
        }
    at_home = mode == "idle" or all(
        abs(q - home) <= 1e-12
        for side_values, home_values in zip(command_values.values(), manifest["home"])
        for q, home in zip(side_values, home_values)
    )
    at_home_message = LatchedBool(
        1, command_sequence, timestamp_ns, at_home,
        authorities["coordinator"]["publisher_instance_id"], authorities["coordinator"]["router_zid"],
    )
    complete_message = LatchedBool(
        1, command_sequence, timestamp_ns, state == "idle" and cycle["ticks"] > 0,
        authorities["coordinator"]["publisher_instance_id"], authorities["coordinator"]["router_zid"],
    )
    native_attempt = None
    if sample is not None and result is not None:
        native_attempt = {
            "tick_id": result["tick_id"],
            "timestamp_ns": result["timestamp_ns"],
            "sample": sample.to_dict(),
        }
    cycle_result = CycleResult(
        commands, result, bool(cycle.get("ik_adopted")), native_attempt,
    )
    return LiveCycleSnapshot(
        result=cycle_result,
        source_status=source_status,
        producer_status=producer_status,
        executor_status=executor_status,
        arm_state=arm_state,
        session_state=session_state,
        hand_telemetry={}, hand_states={}, hand_commands={},
        execution_epoch=state_epoch,
        coordinator_receipt=coordinator_receipt,
        bilateral_command=bilateral,
        received_timestamp_ns=timestamp_ns,
    )


def _publish_native_snapshot(output: AsyncLiveOutput, snapshot: LiveCycleSnapshot) -> None:
    """Publish the same arm-side topics as the Python coordinator/executor."""
    output.put_json(topics.SOURCE_STATUS, snapshot.source_status.to_dict())
    output.put_json(topics.PRODUCER_STATUS, snapshot.producer_status.to_dict())
    output.put_json(topics.EXECUTOR_STATUS, snapshot.executor_status.to_dict())
    output.put_json(topics.ARM_STATE, snapshot.arm_state.to_dict())
    output.put_json(topics.SESSION_STATE, snapshot.session_state.to_dict())
    output.put_json(topics.COORDINATOR_STATUS, snapshot.producer_status.to_dict() | {
        "component_role": "coordinator_arm",
        "component_id": "arm",
        "publisher_instance_id": snapshot.bilateral_command["left"]["publisher_instance_id"],
        "diagnostics": {"native_scheduler": "cpp", "authority": "final_command_and_session_state"},
    })
    output.put_json(topics.AT_HOME, _latched_dict(snapshot, "at_home"))
    output.put_json(topics.RETURN_COMPLETE, _latched_dict(snapshot, "return_complete"))
    for side in ("left", "right"):
        output.put_json(topics.arm_command(side), snapshot.result.commands[side].to_dict())
    if snapshot.bilateral_command is not None:
        output.put_json("tianji/command/arm/bilateral", snapshot.bilateral_command)
    if snapshot.coordinator_receipt is not None:
        output.put_json("tianji/coordinator/arm/bilateral_receipt", snapshot.coordinator_receipt)


def _latched_dict(snapshot: LiveCycleSnapshot, name: str) -> dict[str, Any]:
    command = snapshot.result.commands
    home = name == "at_home" and all(
        command[side].mode == "idle" for side in ("left", "right")
    )
    value = home if name == "at_home" else home and snapshot.session_state.state == "idle"
    return {
        "schema_version": 1,
        "publisher_instance_id": snapshot.bilateral_command["left"]["publisher_instance_id"],
        "router_zid": snapshot.bilateral_command["left"]["router_zid"],
        "sequence": snapshot.session_state.sequence,
        "timestamp_ns": snapshot.session_state.timestamp_ns,
        "value": value,
    }


@dataclass
class _NativeSideState:
    """Thread-safe state produced by the gateway frame consumer."""

    latest: LiveCycleSnapshot | None = None
    phase: str = 'idle'
    epoch: int = 1
    complete: bool = False
    complete_payload: dict[str, Any] | None = None
    failure: str | None = None
    cycles: int = 0
    native_ticks: int = 0
    late_cycles: int = 0


class _NativeFrameConsumer:
    def __init__(self, gateway: NativeGatewayProcess, *, manifest: Mapping[str, Any],
                 prefix: str, output: AsyncLiveOutput, capture: Any,
                 overlay: Any, mapped_overlay: Any, receiver_instance_id: str) -> None:
        self.gateway = gateway
        self.manifest = manifest
        self.prefix = prefix
        self.output = output
        self.capture = capture
        self.overlay = overlay
        self.mapped_overlay = mapped_overlay
        self.receiver_instance_id = receiver_instance_id
        self._summary_only = (manifest.get('publication_backend') == 'cpp'
                              and manifest.get('viewer_backend') == 'cpp'
                              and capture is None and overlay is None and mapped_overlay is None)
        self._state = _NativeSideState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._reports: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=64)
        self._actions: dict[int, str] = {}
        self._pending_calibrations: set[int] = set()
        self._action_lock = threading.Lock()

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._state.failure

    @property
    def latest(self) -> LiveCycleSnapshot | None:
        with self._lock:
            return self._state.latest

    @property
    def control_status(self) -> tuple[str, int]:
        with self._lock:
            return self._state.phase, self._state.epoch

    @property
    def complete(self) -> bool:
        with self._lock:
            return self._state.complete

    @property
    def home_return_completed(self) -> bool:
        with self._lock:
            payload = self._state.complete_payload
            return bool(payload and payload.get('complete') is True and payload.get('state') == 'idle')

    @property
    def counters(self) -> dict[str, int]:
        with self._lock:
            return {"cycles": self._state.cycles,
                    "native_ticks": self._state.native_ticks,
                    "late_cycles": self._state.late_cycles}

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("native frame consumer already started")
        self._thread = threading.Thread(target=self._run, name="native-gateway-side-effects", daemon=True)
        self._thread.start()

    def map_action(self, command_id: int, action: str) -> None:
        with self._action_lock:
            self._actions[command_id] = action

    def send_action(self, gateway: NativeGatewayProcess, action: str, *, next_epoch: int = 0) -> int:
        command_id = gateway.reserve_command_id()
        self.map_action(command_id, action)
        try:
            # Do not hold the action lock during I/O: an immediate reply must
            # be consumable, even before send_action returns.
            gateway.send_action(action, next_epoch=next_epoch, command_id=command_id)
        except BaseException:
            with self._action_lock:
                self._actions.pop(command_id, None)
                self._pending_calibrations.discard(command_id)
            raise
        return command_id

    def poll_reports(self) -> list[dict[str, Any]]:
        result = []
        while True:
            try:
                result.append(self._reports.get_nowait())
            except queue.Empty:
                return result

    def _set_failure(self, reason: str) -> None:
        with self._lock:
            if self._state.failure is None:
                self._state.failure = str(reason)
        self._stop.set()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self.gateway.get_frame(_FRAME_QUEUE_TIMEOUT_S)
                if frame is None:
                    if self.gateway.failure:
                        self._set_failure(self.gateway.failure)
                    elif self.gateway.process is not None and self.gateway.process.poll() is not None and not self.complete:
                        self._set_failure("native gateway exited before completion")
                    continue
                self._consume(frame)
                if frame.kind == "complete":
                    return
        except Exception as exc:
            self._set_failure(f"native gateway side-effect worker failed: {type(exc).__name__}: {exc}")

    def _consume(self, frame: GatewayFrame) -> None:
        if frame.kind == 'summary':
            if not self._summary_only:
                raise RuntimeError('summary cannot replace required Python cycle consumers')
            with self._lock:
                self._state.phase = frame.payload['state']
                self._state.epoch = frame.payload['state_epoch']
                self._state.cycles = frame.payload['cycles']
                self._state.native_ticks = frame.payload['ticks']
                self._state.late_cycles = frame.payload['late_ticks']
            return
        if frame.kind == "reply":
            with self._action_lock:
                ident = frame.payload['id']
                action = self._actions.get(ident, 'unknown')
                if action == 'calibrate' and frame.payload['accepted'] and ident not in self._pending_calibrations:
                    # The first reply acknowledges sampling; keep its identity
                    # until commit/failure, rather than labelling that reply unknown.
                    self._pending_calibrations.add(ident)
                else:
                    self._actions.pop(ident, None)
                    self._pending_calibrations.discard(ident)
            if frame.payload['id'] == 0:
                action = 'auto_rearm'
            if action == 'unknown' and self.manifest.get('viewer_backend') == 'cpp' and frame.payload['id'] >= 2**63:
                action = {1:'start',2:'return',3:'shutdown',4:'rearm',5:'calibrate'}.get(frame.payload['id'] & 7,'unknown')
            report = {"kind": "operator_result", "action": action,
                      "accepted": frame.payload["accepted"], "reason": frame.payload["reason"]}
            try:
                self._reports.put_nowait(report)
            except queue.Full:
                self._set_failure("native operator report queue overflow")
            if self.capture:
                self.capture.audit("operator_result", report, frame.timestamp_ns or time.monotonic_ns())
            return
        if frame.kind == "failure":
            self._set_failure(frame.payload["reason"])
            return
        if frame.kind == "complete":
            with self._lock:
                self._state.phase = frame.payload['state']
                self._state.epoch = frame.payload['epoch']
                self._state.complete = True
                self._state.complete_payload = dict(frame.payload)
            return
        if frame.kind == "receipt":
            return
        if frame.kind == "raw":
            self._consume_raw(frame)
            return
        if frame.kind != "cycle":
            self._set_failure(f"unknown native gateway frame kind: {frame.kind}")
            return
        cycle = frame.payload
        if self._summary_only:
            self._update_cycle_status(cycle, None)
            return
        sample = None
        try:
            sample = _frame_sample(cycle, receiver_instance_id=self.receiver_instance_id)
        except (TypeError, ValueError, KeyError, IndexError):
            # Passive diagnostics are best effort.  The C++ source already
            # accepted/gated the packet, so a Python display parser mismatch
            # cannot be allowed to affect control or recording completion.
            sample = None
        snapshot = _typed_cycle(cycle, manifest=self.manifest, prefix=self.prefix, sample=sample)
        if self.mapped_overlay is not None:
            self.mapped_overlay.ingest_cycle(
                snapshot.result.native_result, snapshot.result.native_attempt,
                execution_epoch=snapshot.execution_epoch,
                active=snapshot.session_state.state == "teleop",
            )
        elif self.overlay is not None:
            if snapshot.result.native_result is not None:
                self.overlay.ingest_native(snapshot.result.native_result,
                                           execution_epoch=snapshot.execution_epoch)
        if self.capture:
            self.capture.cycle_snapshot(snapshot)
        if self.manifest.get('publication_backend', 'python') == 'python':
            _publish_native_snapshot(self.output, snapshot)
        self._update_cycle_status(cycle, snapshot)

    def _update_cycle_status(self, cycle, snapshot):
        with self._lock:
            self._state.latest = snapshot
            self._state.phase = cycle['state']
            self._state.epoch = int(cycle['state_epoch'])
            self._state.cycles += 1
            self._state.native_ticks = max(self._state.native_ticks,
                                           int(cycle.get("ticks", 0)))
            self._state.late_cycles = int(cycle.get("late_ticks", 0))

    def _consume_raw(self, frame: GatewayFrame) -> None:
        """Record every decoded packet before native stream-gate acceptance."""
        if self.capture is None and self.overlay is None:
            return
        payload = frame.payload
        packet = payload["packet"]
        observation = None
        try:
            observation = parse_reference_tjvr_packet(
                packet,
                receiver_instance_id=self.receiver_instance_id,
                receiver_frame_sequence=payload["sequence"],
                received_timestamp_ns=payload["received_ns"],
            )
        except (TypeError, ValueError, KeyError, IndexError) as exc:
            # A mismatch here means the passive Python diagnostic parser has
            # drifted from the C++ decoder.  Keep control alive when no
            # recording is requested, but make the discrepancy auditable.
            if self.capture:
                self.capture.audit("native_tjvr_ingress", {
                    "version": 1,
                    "receiver_instance_id": self.receiver_instance_id,
                    "receiver_frame_sequence": payload["sequence"],
                    "received_timestamp_ns": payload["received_ns"],
                    "accepted": payload["accepted"],
                    "parser_error": f"{type(exc).__name__}: {exc}",
                }, payload["received_ns"])
            return
        if self.capture:
            self.capture.tjvr(observation)
            self.capture.audit("native_tjvr_ingress", {
                "version": 1,
                "receiver_instance_id": self.receiver_instance_id,
                "receiver_frame_sequence": payload["sequence"],
                "received_timestamp_ns": payload["received_ns"],
                "accepted": payload["accepted"],
                "parser_error": None,
            }, payload["received_ns"])
        if self.overlay is not None and self.mapped_overlay is None:
            self.overlay.ingest_raw(observation)

    def close(self, *, drain: bool = True) -> None:
        if not drain:
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=30 if drain else 5)
            if self._thread.is_alive():
                self._set_failure("native gateway side-effect worker did not stop")
                self._thread.join(timeout=_FRAME_QUEUE_TIMEOUT_S * 2)


def _set_render_qpos(model: Any, data: Any, positions: Mapping[str, list[float]], mujoco: Any) -> None:
    for side in ("left", "right"):
        values = positions.get(side)
        if not isinstance(values, list) or len(values) != 7:
            continue
        for name, value in zip(ARM_JOINT_NAMES[side], values):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                data.qpos[model.jnt_qposadr[joint_id]] = value


def _declare_native_authorities(session: Any, manifest: Mapping[str, Any], stack: ExitStack):
    from ...zenoh_util import declare_component_liveliness
    tokens = set()
    roles=[(role,manifest[key]) for role,key in (("source", "source_authority"),
                      ("producer/arm", "producer_authority"),
                      ("coordinator/arm", "coordinator_authority"),
                      ("executor/arm", "executor_authority"))]
    if 'hand_authorities' in manifest:
        roles.extend([('source',manifest['manus_source_authority']),
            ('producer/hand',manifest['hand_authorities']['producer']),
            *[('executor/hand',manifest['hand_authorities'][side]) for side in ('left','right')]])
    for role, value in roles:
        authority = _authority(value)
        token = declare_component_liveliness(
            session, role=role, logical_id=authority["logical_id"],
            instance_id=authority["publisher_instance_id"],
        )
        if token is not None:
            stack.callback(token.undeclare)
        tokens.add(f"tj/live/{role}/{authority['logical_id']}/{authority['publisher_instance_id']}")
    return tokens


def _wait_for_guard(session: Any, guard: LiveDomainGuard) -> Any:
    """Inventory authorities and keep the subscriber for live supervision."""
    import zenoh
    subscriber = session.liveliness().declare_subscriber(
        "tj/live/**",
        lambda sample: guard.observe(str(sample.key_expr), present=sample.kind == zenoh.SampleKind.PUT),
        history=True,
    )
    keep = False
    try:
        for reply in session.liveliness().get("tj/live/**", timeout=.5):
            if not reply.ok:
                raise RuntimeError("live authority inventory failed")
            guard.observe(str(reply.result.key_expr), present=True)
        if guard.failure:
            raise RuntimeError(guard.failure)
        keep = True
        return subscriber
    finally:
        if not keep:
            subscriber.undeclare()


def run_live_native(root: Path, args: Any, resolved: Mapping[str, Any]) -> int:
    """Run the explicit C++ arm or bilateral arm/hand scheduler."""
    required = ("TIANJI_RUN_ID", "TIANJI_DUAL_INSTANCE_ID", "TIANJI_ROUTER_ZID")
    if os.environ.get("TIANJI_DUAL_MANAGED") != "1" or any(not os.environ.get(k) for k in required):
        raise RuntimeError("use the managed run_session launcher; explicit session identities required")
    run_id, instance_id, expected_router = (os.environ[k] for k in required)
    config = resolved["config"]
    hands=config.get('hands_enabled') is True
    if type(config.get('hands_enabled')) is not bool:
        raise RuntimeError('explicit hands_enabled required')
    if hands and (config.get('hand_input')!='manus' or
            sorted(config.get('active_hand_sides',[]))!=['left','right'] or
            getattr(args,'publication_backend','python')!='cpp' or getattr(args,'viewer_backend','python')!='cpp' or
            not args.viewer or getattr(args,'recording_adapter','python')!='cpp' or args.record is None):
        raise RuntimeError('native joint route requires bilateral Manus, C++ publication/viewer/recording and --record')
    backend = config["ik_backend"]
    prefix = "mapped_palm" if backend == "pico_ee_mapped_corrected_palm_velocity_qp" else "spark"
    height_enabled = bool(resolved.get("mapped_palm_height_calibration", False))
    from ...zenoh_util import open_session, require_single_router
    from .backend_assets import bilateral_assets

    stop = threading.Event()
    exit_trigger = "control_stop"
    keys: queue.Queue[str] = queue.Queue(maxsize=64)
    action_ids: dict[str, int] = {}
    shutdown_requested = False
    recorder = capture = None
    native_recording = getattr(args, 'recording_adapter', 'python') == 'cpp'
    native_viewer = getattr(args, 'viewer_backend', 'python') == 'cpp'
    gateway = None
    consumer = None
    viewer = None
    render_data = None
    overlay = mapped_overlay = None
    started = time.monotonic()
    exit_code = 1
    manifest_path = None
    manus = None

    def key_callback(value: Any) -> None:
        nonlocal exit_trigger
        if type(value) is not int:
            return
        key = "c" if height_enabled and value in (ord("c"), ord("C")) else None
        if key is None and 0 <= value < 128:
            candidate = chr(value).lower()
            if candidate in ("s", "h", "r", "q"):
                key = candidate
        if key is not None:
            try:
                keys.put_nowait(key)
            except queue.Full:
                exit_trigger = "operator_queue_overflow"
                stop.set()

    try:
        with ExitStack() as stack:
            session = open_session()
            stack.callback(session.close)
            router = require_single_router(session, expected_router)

            if args.record is not None:
                from ...recording.async_dual import AsyncDualRecorder
                from ...recording.live_capture import LiveCapture
                robot_model = "mapped_palm/marvin_m6_wuji2" if prefix == "mapped_palm" else "spark/marvin_m6_wuji2"
                recorder_type = AsyncDualRecorder
                recorder_options = {}
                if native_recording:
                    from ...recording.native_owner import NativeRecordingOwner
                    recorder_type = NativeRecordingOwner
                    recorder_options['root'] = root
                recorder = recorder_type(
                    args.record, router_zid=router,
                    robot_model=robot_model,
                    metadata=dict(run_id=run_id, resolved_configuration=dict(resolved),
                                  scheduler_backend="cpp", native_gateway_protocol_version=1,
                                  real_time_qualified=False, input_clock="host_monotonic_ns"),
                    **recorder_options,
                )
                if not native_recording:
                    capture = LiveCapture(recorder, run_id=run_id)
                    capture.audit("lifecycle", dict(stage="opening"))

            output = AsyncLiveOutput(session)
            stack.callback(output.close)
            hand_authorities={}
            if hands:
                from .native_hand_launch import hand_identities
                hand_authorities=hand_identities(instance_id,router)
            tokens = _declare_native_authorities(session, {
                "source_authority": {"logical": "tjvr", "instance": instance_id + "-source", "router": router},
                "producer_authority": {"logical": "ik_mapped_palm" if prefix == "mapped_palm" else "ik_spark_headroom",
                                        "instance": instance_id + "-" + prefix, "router": router},
                "coordinator_authority": {"logical": "arm", "instance": instance_id + "-coord", "router": router},
                "executor_authority": {"logical": "mujoco", "instance": instance_id + "-sim", "router": router},
                **hand_authorities,
            }, stack)
            guard = LiveDomainGuard(tokens)
            # The token declarations above are the native scheduler's explicit
            # authorities.  Observe the inventory before opening UDP.
            guard_subscriber = _wait_for_guard(session, guard)
            stack.callback(guard_subscriber.undeclare)

            manifest = build_native_gateway_manifest(
                Path(root), resolved, run_id=run_id, instance_id=instance_id,
                router_zid=router, tjvr_bind=args.tjvr_bind, tjvr_port=args.tjvr_port,
                native_hands=hands,
            )
            manifest['publication_backend'] = getattr(args, 'publication_backend', 'python')
            manifest['viewer_backend'] = 'cpp' if native_viewer else 'python'
            manifest['spark_overlay'] = bool(args.spark_overlay)
            manifest['diagnostic_transport'] = ('summary' if native_viewer and capture is None
                and manifest['publication_backend'] == 'cpp' else 'full')
            if native_recording:
                if recorder is None: raise ValueError('native recording requires --record')
                manifest['recording_fd'] = recorder.fileno()
                manifest['recording_origin_ns'] = recorder.origin_ns
            if manifest['publication_backend'] == 'cpp':
                endpoint = os.environ.get('TIANJI_ROUTER_ENDPOINT', '')
                if not endpoint:
                    raise ValueError('C++ publication requires explicit TIANJI_ROUTER_ENDPOINT')
                manifest['publication_endpoint'] = endpoint
            if hands:
                from ...hand_tracking.native_manus_process import NativeManusProcess
                from ...hand_tracking.manus_environment import manus_environment
                from .native_hand_launch import build_hand_manifest
                manus_env,_=manus_environment(args.manus_rawviz,library_dir=args.manus_library_dir)
                manus=NativeManusProcess(command=[str(args.manus_rawviz),'--user',args.manus_user],
                    cwd=os.path.dirname(os.path.abspath(os.fspath(args.manus_rawviz))),env=manus_env)
                stack.callback(manus.close)
                manifest.update(build_hand_manifest(root,run_id=run_id,instance_id=instance_id,router=router,
                    stdout_fd=manus.fileno(),right_glove=args.right_glove,left_glove=args.left_glove))
            runtime_dir = Path(os.environ.get("TELEOP_RUNTIME_DIR", str(Path(root) / ".runtime")))
            manifest_path = write_native_gateway_manifest(runtime_dir, manifest)
            gateway = NativeGatewayProcess(
                Path(root) / "build/control-native/tianji_native_session_gateway",
                manifest_path, prefix=prefix,
                startup_timeout_s=10., io_timeout_s=1., frame_capacity=4096,
                publication_backend=manifest['publication_backend'],
                recording_fd=recorder.fileno() if native_recording else None,
                viewer_backend=manifest['viewer_backend'],
                diagnostic_transport=manifest['diagnostic_transport'],
                manus_fd=manus.fileno() if manus else None,
            )
            try:
                gateway.start()
                if manus:manus.mark_transferred()
            finally:
                if native_recording: recorder.release_descriptor()
                manifest_path.unlink(missing_ok=True)
                manifest_path = None
            stack.callback(gateway.close, graceful=False)

            if args.spark_overlay and not native_viewer:
                from ...executors.mujoco.spark_overlay import SparkOverlay
                if prefix == "mapped_palm":
                    from ...executors.mujoco.mapped_palm_overlay import MappedPalmOverlay
                    mapped_overlay = MappedPalmOverlay(manifest["source_authority"]["instance"])
                    overlay = mapped_overlay
                else:
                    overlay = SparkOverlay(manifest["source_authority"]["instance"], algorithm=backend)

            consumer = _NativeFrameConsumer(
                gateway, manifest=manifest, prefix=prefix, output=output,
                capture=capture, overlay=overlay, mapped_overlay=mapped_overlay,
                receiver_instance_id=manifest["source_authority"]["instance"],
            )
            consumer.start()
            stack.callback(consumer.close, drain=False)
            gateway.wait_for_cycle(10.)
            if capture:
                capture.audit("lifecycle", dict(stage="native_gateway_ready", authorities={
                    key: manifest[key] for key in ("source_authority", "producer_authority",
                                                   "coordinator_authority", "executor_authority")}),
                              time.monotonic_ns())

            if args.viewer and not native_viewer:
                import mujoco
                from .native_passive_viewer import joined_passive_viewer
                model_path = bilateral_assets(Path(root), backend)["model"]
                model = mujoco.MjModel.from_xml_path(str(model_path))
                render_data = mujoco.MjData(model)
                mujoco.mj_forward(model, render_data)
                viewer = stack.enter_context(joined_passive_viewer(
                    model, render_data, key_callback=key_callback))

            print(
                f"Teleop ready: run_id={run_id}; IK={backend}; hands={hands}; "
                f"rate={config['rate_hz']} Hz; calibration={'XZ' if resolved.get('mapped_palm_xz_calibration') else ('Z' if height_enabled else 'off')}; "
                f"scheduler=cpp; publication={manifest['publication_backend']}; "
                f"recording_adapter={'cpp' if native_recording else 'python'}; viewer={manifest['viewer_backend']}; "
                f"diagnostics={manifest['diagnostic_transport']}; record={resolved.get('record_path')}",
                flush=True,
            )
            print("s: start (requires fresh input); h: return Home + automatic rearm; r: manual rearm at Home; "
                  "q: return and exit. Fault requires restart.", flush=True)
            if height_enabled:
                print(("c: hold both arms forward horizontally for 2 seconds (X/Z; robot J2=-90 degrees, others zero), then s to start."
                       if resolved.get('mapped_palm_xz_calibration') else
                       "c: hold both arms horizontal for 2 seconds (Z-only calibration), then s to start."),
                      flush=True)

            def stop_signal(signum: int, _frame: Any) -> None:
                nonlocal exit_trigger
                exit_trigger = signal.Signals(signum).name
                stop.set()

            for signum in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(signum, stop_signal)
                stack.callback(signal.signal, signum, previous)

            terminal_state = None
            if sys_stdin_is_tty := os.isatty(0):
                terminal_state = termios.tcgetattr(0)
                stack.callback(termios.tcsetattr, 0, termios.TCSANOW, terminal_state)
                tty.setcbreak(0)

            while not stop.is_set():
                _check_native_health(consumer, gateway, guard, output)
                if manus and manus.failure:
                    raise RuntimeError(manus.failure)
                for report in consumer.poll_reports():
                    if report["action"] == "shutdown" and report["accepted"]:
                        shutdown_requested = True
                    print(json.dumps(report, ensure_ascii=False), flush=True)
                if consumer.complete:
                    break
                if viewer is not None and not viewer.is_running():
                    exit_trigger = "viewer_closed"
                    if not shutdown_requested:
                        consumer.send_action(gateway, "shutdown")
                        shutdown_requested = True
                    stop.wait(.005)
                    continue
                if args.duration_s is not None and time.monotonic() - started >= args.duration_s:
                    exit_trigger = "duration_elapsed"
                    if not shutdown_requested:
                        consumer.send_action(gateway, "shutdown")
                        shutdown_requested = True
                if sys_stdin_is_tty and select.select([0], [], [], 0)[0]:
                    for value in os.read(0, 128):
                        key_callback(value)
                while True:
                    try:
                        key = keys.get_nowait()
                    except queue.Empty:
                        break
                    if key == "c" and height_enabled:
                        action = "calibrate"
                        next_epoch = 0
                    elif key == "r":
                        action = "rearm"
                        next_epoch = consumer.control_status[1] + 1
                    elif key == "s":
                        action, next_epoch = "start", 0
                    elif key == "q":
                        action, next_epoch = "shutdown", 0
                        exit_trigger = "control_stop"
                        shutdown_requested = True
                    else:
                        action, next_epoch = "return", 0
                    command_id = consumer.send_action(gateway, action, next_epoch=next_epoch)
                    action_ids[action] = command_id

                if viewer is not None:
                    import mujoco
                    snapshot = consumer.latest
                    with viewer.lock():
                        if snapshot is not None:
                            _set_render_qpos(model, render_data,
                                             {side: list(snapshot.result.commands[side].position_rad)
                                              for side in ("left", "right")}, mujoco)
                            mujoco.mj_forward(model, render_data)
                        viewer.user_scn.ngeom = 0
                        if mapped_overlay is not None:
                            mapped_overlay.update_render_targets(
                                model, render_data, mujoco, time.monotonic_ns(),
                                fk_current=snapshot is not None,
                            )
                        if overlay is not None:
                            overlay.append(viewer.user_scn, mujoco, time.monotonic_ns())
                    viewer.sync()
                stop.wait(.005)

            if not consumer.complete and not shutdown_requested:
                consumer.send_action(gateway, "shutdown")
                shutdown_requested = True
            if not consumer.complete:
                if not gateway.wait_for_complete(_NATIVE_CLOSE_TIMEOUT_S):
                    raise TimeoutError("native gateway did not complete after shutdown")
            failure = _finish_native_outputs(consumer, gateway, guard, output)
            for report in consumer.poll_reports():
                print(json.dumps(report, ensure_ascii=False), flush=True)
            counters = consumer.counters
            state = consumer.control_status[0]
            exit_code = 1 if failure else 0
            report = {
                "kind": "dual_live_complete", "state": state,
                "reason": failure or "return complete", "scheduler_backend": "cpp",
                "native_ticks": counters["native_ticks"], "late_cycles": counters["late_cycles"],
                "control_ticks": counters["cycles"],
                "exit_trigger": exit_trigger, "home_return_completed": consumer.home_return_completed,
                "real_time_qualified": False,
            }
            if capture:
                capture.audit("lifecycle", report, time.monotonic_ns())
            if recorder is not None:
                try:
                    recorder.close(complete=exit_code == 0)
                except Exception as exc:
                    failure = failure or str(exc)
                    exit_code = 1
            if recorder is not None:
                report["recording"] = recorder.statistics
            report["reason"] = failure or report["reason"]
            print(json.dumps(report, ensure_ascii=False), flush=True)
            return exit_code
    except KeyboardInterrupt:
        exit_trigger = "SIGINT"
        if gateway is not None:
            try:
                if consumer is not None:
                    consumer.send_action(gateway, "shutdown")
                else:
                    gateway.send_action("shutdown")
                gateway.wait_for_complete(_NATIVE_CLOSE_TIMEOUT_S)
            except Exception:
                pass
        if recorder is not None:
            try:
                recorder.close(complete=False)
            except Exception:
                pass
        raise
    finally:
        if manifest_path is not None:
            manifest_path.unlink(missing_ok=True)
        if consumer is not None:
            consumer.close(drain=False)
        if gateway is not None:
            gateway.close(graceful=False)
        if recorder is not None and not getattr(recorder, "_closed", False):
            try:
                recorder.close(complete=False)
            except Exception:
                pass


__all__ = ["run_live_native"]
