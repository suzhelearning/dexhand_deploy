"""Canonical Marvin arm executor.

SDK 只在本模块的 rad↔degree 边界出现。所有上游输入均为 protocol v1
消息，executor 不创建 command 或 SessionState authority。
"""
from __future__ import annotations
import argparse
import json
import logging
import os
from typing import Any, Mapping
import time

import numpy as np

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...hardware_safety import HardwareSafetyController, HardwareSafetySettings
from ...marvin_hardware import MarvinFeedback, MarvinHardwareError, MarvinHardwareSession
from ...marvin_state import command_states_compatible
from ...protocol import topics
from ...protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    ArmJointCommand,
    ArmJointState,
    ComponentStatus,
    ProtocolEnvelope,
    ProtocolError,
    SafetyStopAck,
    SafetyStopRequest,
    SessionState,
    strict_loads,
)
from ...sources.common.real_admission import RealCapabilityInput, parse_real_capability
from ...zenoh_util import open_session, require_single_router, declare_component_liveliness
from .readiness import MarvinReadiness

_LOG = logging.getLogger(__name__)


def _put(publisher: Any, payload: Mapping[str, Any]) -> None:
    if publisher is None:
        return
    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    try:
        publisher.put(data, encoding="application/json")
    except TypeError:
        publisher.put(data)


def _payload(value: Any) -> Mapping[str, Any]:
    value = getattr(value, "payload", value)
    return value if isinstance(value, Mapping) else strict_loads(bytes(value))


def _create_official_session() -> MarvinHardwareSession:
    from .sdk_session import create_official_marvin_session
    return create_official_marvin_session()


class MarvinExecutor:
    """双臂 canonical final-command consumer与 Marvin SDK 安全边界。"""

    def __init__(
        self,
        *,
        session: Any = None,
        hardware_session: MarvinHardwareSession | None = None,
        robot_config: ArmRobotConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str,
        real_capability: RealCapabilityInput | Any | None = None,
        run_id: str | None = None,
        safety_supervisor_instance_id: str | None = None,
        params: Mapping[str, Any] | None = None,
        clock: Any = time.monotonic_ns,
    ) -> None:
        if not publisher_instance_id or not router_zid or not coordinator_instance_id:
            raise ValueError("executor, router, and coordinator identities are required")
        self.session = session
        self.robot = robot_config if isinstance(robot_config, ArmRobotConfig) else ArmRobotConfig.from_mapping(robot_config) if isinstance(robot_config, Mapping) else ArmRobotConfig.load(robot_config)
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.coordinator_instance_id = coordinator_instance_id
        self.real_capability = real_capability
        self.run_id = run_id
        self.safety_supervisor_instance_id = safety_supervisor_instance_id
        self.clock = clock
        options = {
            "command_timeout_s": 0.15,
            "feedback_timeout_s": 0.15,
            "state_timeout_s": 1.0,
            "home_tolerance_rad": np.deg2rad(1.0),
            "maximum_tracking_error_deg": 8.0,
            "maximum_output_step_deg": 0.5,
            "return_max_speed_deg_s": 10.0,
            "return_minimum_duration_s": 2.0,
            "rate_hz": 30.0,
            "velocity_ratio": 10,
            "acceleration_ratio": 10,
            "feedback_hard_limit_padding_deg": 5.0,
            "robot_ip": "",
            "connection_wait_s": 1.0,
            "hardware_factory": _create_official_session,
        }
        if params:
            options.update(params)
        self.params = options
        self._rate_hz = float(options["rate_hz"])
        if self._rate_hz <= 0.0:
            raise ValueError("rate must be positive")
        self._session_factory = options["hardware_factory"]
        self._hardware = hardware_session
        self._readiness = MarvinReadiness(
            robot_config=self.robot,
            router_zid=router_zid,
            freshness_timeout_s=float(options.get("host_status_timeout_s", 1.0)),
            command_timeout_s=float(options["command_timeout_s"]),
            home_tolerance_rad=float(options["home_tolerance_rad"]),
            expected_authorities=options.get("expected_authorities"),
        )
        self._hardware_safety = HardwareSafetyController(
            left_home_deg=np.degrees(self.robot.left_home_rad),
            right_home_deg=np.degrees(self.robot.right_home_rad),
            lower_limits_deg=np.degrees(self.robot.lower_limits_rad),
            upper_limits_deg=np.degrees(self.robot.upper_limits_rad),
            settings=HardwareSafetySettings(
                command_timeout_s=float(options["command_timeout_s"]),
                state_timeout_s=float(options["state_timeout_s"]),
                feedback_timeout_s=float(options["feedback_timeout_s"]),
                maximum_output_step_deg=float(options["maximum_output_step_deg"]),
                maximum_tracking_error_deg=float(options["maximum_tracking_error_deg"]),
                return_minimum_duration_s=float(options["return_minimum_duration_s"]),
                return_max_speed_deg_s=float(options["return_max_speed_deg_s"]),
                home_tolerance_deg=float(np.degrees(options["home_tolerance_rad"])),
            ),
        )
        self._phase = "waiting_for_connection"
        self._session_state: SessionState | None = None
        self._session_state_received_ns: int | None = None
        self._commands: dict[str, ArmJointCommand] = {}
        self._command_received_ns: dict[str, int] = {}
        self._feedback: MarvinFeedback | None = None
        self._feedback_received_ns: int | None = None
        self._last_output_deg: np.ndarray | None = None
        self._last_error: str | None = None
        self._last_action = "none"
        self._command_count = 0
        self._safety_locked = False
        self._safety_ack: SafetyStopAck | None = None
        self._state_sequence = 0
        self._status_sequence = 0
        self._publishers: dict[str, Any] = {}
        self._subscriptions: list[Any] = []
        self._liveliness_token = declare_component_liveliness(
            session, role="executor/arm", logical_id="marvin", instance_id=publisher_instance_id
        ) if session is not None else None
        self._setup_transport()
        self._status = self._make_status(False, False, self._phase)

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def safety_locked(self) -> bool:
        return self._safety_locked

    @property
    def safety_ack(self) -> SafetyStopAck | None:
        return self._safety_ack

    @property
    def hardware_session(self) -> MarvinHardwareSession | None:
        return self._hardware

    @property
    def readiness(self) -> MarvinReadiness:
        return self._readiness

    @property
    def arm_state(self) -> ArmJointState | None:
        if self._feedback is None:
            return None
        values = np.concatenate([self._feedback.left_joints_deg, self._feedback.right_joints_deg])
        return ArmJointState(
            1, self._state_sequence, int(self.clock()), "marvin",
            list(ALL_ARM_JOINT_NAMES), np.radians(values).tolist(), None,
            self.publisher_instance_id, self.router_zid,
        )

    @property
    def status(self) -> ComponentStatus:
        return self._status
    def _clock_seconds(self, now_ns: int | None = None) -> float:
        """Convert the injected monotonic-ns clock to controller seconds."""
        value = int(self.clock()) if now_ns is None else int(now_ns)
        return value / 1e9


    def _setup_transport(self) -> None:
        if self.session is None:
            return
        # Subscriber declaration precedes ready status publication.
        self._subscriptions.extend([
            self.session.declare_subscriber(topics.arm_command("left"), self.on_arm_command),
            self.session.declare_subscriber(topics.arm_command("right"), self.on_arm_command),
            self.session.declare_subscriber(topics.SESSION_STATE, self.on_session_state),
            self.session.declare_subscriber(topics.SOURCE_STATUS, lambda value: self.on_component_status(value)),
            self.session.declare_subscriber(topics.PRODUCER_STATUS, lambda value: self.on_component_status(value)),
            self.session.declare_subscriber(topics.SAFETY_STOP, self.on_safety_stop),
        ])
        self._publishers = {
            "state": self.session.declare_publisher(topics.ARM_STATE),
            "status": self.session.declare_publisher(topics.EXECUTOR_STATUS),
            "safety_ack": self.session.declare_publisher(topics.safety_ack(self.publisher_instance_id)),
        }

    def on_component_status(self, value: ComponentStatus | Mapping[str, Any] | Any) -> None:
        try:
            status = value if isinstance(value, ComponentStatus) else ComponentStatus.from_dict(_payload(value))
            self._readiness.observe_component(status, received_ns=int(self.clock()))
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid component status: {exc}"

    def on_session_state(self, value: SessionState | Mapping[str, Any] | Any) -> None:
        try:
            state = value if isinstance(value, SessionState) else SessionState.from_dict(_payload(value))
            if state.router_zid != self.router_zid or state.publisher_instance_id != self.coordinator_instance_id:
                raise ProtocolError("session state coordinator identity mismatch")
            received_ns = int(self.clock())
            if state.timestamp_ns > received_ns or received_ns - state.timestamp_ns > int(self.params["state_timeout_s"] * 1e9):
                raise ProtocolError("session state is stale")
            if self._session_state is not None and state.sequence <= self._session_state.sequence:
                raise ProtocolError("session state sequence rollback")
            self._session_state = state
            self._session_state_received_ns = received_ns
            self._readiness.observe_session_state(state, received_ns=received_ns)
            # fault is a latched coordinator decision.  A later returning or
            # idle snapshot may not downgrade it and accidentally re-enable
            # normal motion during a reconnect race.
            if state.state == "fault":
                self._phase = "fault_return"
            elif self._phase != "fault_return":
                if state.state == "returning":
                    self._phase = "returning"
                elif state.state == "idle" and self._phase not in {"soft_stopped", "fault_return"}:
                    self._phase = "armed_idle"
                elif state.state == "teleop" and self._phase == "armed_idle":
                    self._phase = "teleop"
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid session state: {exc}"

    def on_arm_command(self, value: ArmJointCommand | Mapping[str, Any] | Any) -> bool:
        try:
            command = value if isinstance(value, ArmJointCommand) else ArmJointCommand.from_dict(_payload(value))
            if command.router_zid != self.router_zid or command.producer != "coordinator" or command.publisher_instance_id != self.coordinator_instance_id:
                raise ProtocolError("arm command coordinator identity mismatch")
            received = int(self.clock())
            if not self._readiness.observe_command(command, received_ns=received):
                raise ProtocolError(self._readiness.last_error or "arm command rejected")
            self._commands[command.side] = command
            self._command_received_ns[command.side] = received
            return True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid arm command: {exc}"
            return False

    def on_safety_stop(self, value: SafetyStopRequest | Mapping[str, Any] | Any) -> bool:
        if self._safety_locked:
            return False
        if not self.safety_supervisor_instance_id or not self.run_id:
            self._last_error = "safety stop rejected: authorization is not configured"
            return False
        try:
            request = value if isinstance(value, SafetyStopRequest) else SafetyStopRequest.from_dict(_payload(value))
            request.validate_authority(self.safety_supervisor_instance_id, self.run_id)
            if request.envelope.router_zid != self.router_zid:
                raise ProtocolError("safety stop router_zid mismatch")
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid safety stop: {exc}"
            return False
        self._safety_locked = True
        self._phase = "soft_stopped"
        self._last_error = request.reason
        if self._hardware is not None:
            try:
                self._hardware.soft_stop_once()
            except Exception as exc:
                self._last_error = f"{request.reason}; soft_stop_failed: {exc}"
        self._safety_ack = SafetyStopAck(
            ProtocolEnvelope(1, self.publisher_instance_id, self.router_zid, request.envelope.sequence, int(self.clock())),
            self.publisher_instance_id, self.run_id, True, request.reason,
        )
        self._publish_status()
        _put(self._publishers.get("safety_ack"), self._safety_ack.to_dict())
        return True

    def _admission_ok(self) -> bool:
        if self.real_capability is None:
            self._last_error = "typed real capability input missing"
            return False
        try:
            value = parse_real_capability(self.real_capability)
        except Exception as exc:
            self._last_error = str(exc)
            return False
        if not value.admitted:
            self._last_error = "real capability preflight denied"
            return False
        return True

    def connect(self) -> bool:
        if self._safety_locked:
            self._last_error = "SafetyStop is latched; restart executor before reconnect"
            self._phase = "soft_stopped"
            self._publish_status()
            return False
        now = int(self.clock())
        recovery_state = self._session_state.state if self._session_state is not None else None
        fault_reconnect = recovery_state == "fault"
        bounded_reconnect = self._readiness.fault_return_ready(now_ns=now)
        deadline = time.monotonic() + float(self.params.get("connection_wait_s", 1.0))
        if not bounded_reconnect:
            while True:
                current_state = self._session_state.state if self._session_state is not None else None
                # A fault can arrive while the normal admission gate is
                # waiting.  It is safer to switch to the coordinator-owned
                # bounded return path than to wait for source readiness.
                if current_state == "fault":
                    if self._readiness.fault_return_ready(now_ns=now):
                        fault_reconnect = True
                        bounded_reconnect = True
                        break
                    self._last_error = "fault reconnect requires fresh bounded returning command"
                    if time.monotonic() >= deadline:
                        self._phase = "waiting_for_connection"
                        self._publish_status()
                        return False
                    time.sleep(0.02)
                    now = int(self.clock())
                    continue
                if self._admission_ok() and self._readiness.connection_ready(now_ns=now):
                    break
                if time.monotonic() >= deadline:
                    self._phase = "waiting_for_connection"
                    self._publish_status()
                    return False
                time.sleep(0.02)
                now = int(self.clock())
        if self._hardware is None:
            self._hardware = self._session_factory()
        self._phase = "connecting"
        try:
            feedback = self._hardware.connect_and_prepare(
                str(self.params.get("robot_ip", "")),
                velocity_ratio=int(self.params["velocity_ratio"]),
                acceleration_ratio=int(self.params["acceleration_ratio"]),
                lower_limits_deg=np.degrees(self.robot.lower_limits_rad),
                upper_limits_deg=np.degrees(self.robot.upper_limits_rad),
                hard_limit_padding_deg=float(self.params["feedback_hard_limit_padding_deg"]),
            )
            self._feedback = feedback
            self._feedback_received_ns = int(self.clock())
            latest_state = self._session_state.state if self._session_state is not None else None
            # Re-check immediately before any home motion: a fault that won
            # the race during connect must dominate the older state captured
            # by the admission check.
            if latest_state == "fault":
                fault_reconnect = True
            if not fault_reconnect and latest_state != "returning":
                self._hardware.move_to_home(
                    np.degrees(self.robot.left_home_rad), np.degrees(self.robot.right_home_rad),
                    rate_hz=self._rate_hz,
                    minimum_duration_s=float(self.params["return_minimum_duration_s"]),
                    max_speed_deg_s=float(self.params["return_max_speed_deg_s"]),
                    maximum_tracking_error_deg=float(self.params["maximum_tracking_error_deg"]),
                    home_tolerance_deg=float(np.degrees(self.params["home_tolerance_rad"])),
                    lower_limits_deg=np.degrees(self.robot.lower_limits_rad),
                    upper_limits_deg=np.degrees(self.robot.upper_limits_rad),
                    hard_limit_padding_deg=float(self.params["feedback_hard_limit_padding_deg"]),
                )
            # Consult the latest state again so a fault callback cannot be
            # overwritten by the stale pre-connect state.
            latest_state = self._session_state.state if self._session_state is not None else None
            if latest_state == "fault":
                self._phase = "fault_return"
            elif latest_state == "returning":
                self._phase = "returning"
            else:
                self._phase = "armed_idle"
            return True
        except BaseException as exc:
            self._last_error = f"startup_error: {exc}"
            self._phase = "failed"
            self._release_hardware(soft_stop=True)
            return False

    def _release_hardware(self, *, soft_stop: bool = False) -> None:
        hardware = self._hardware
        self._hardware = None
        if hardware is None:
            return
        if soft_stop:
            try:
                hardware.soft_stop_once()
            except Exception:
                pass
        try:
            hardware.shutdown()
        except Exception:
            pass

    def _command_pair_fresh(self, now_ns: int) -> bool:
        return all(
            side in self._commands and 0 <= now_ns - self._command_received_ns.get(side, -1) <= int(self.params["command_timeout_s"] * 1e9)
            for side in ("left", "right")
        )

    def _bounded_fault_return(self, now_ns: int) -> bool:
        # A fault may only consume coordinator's bounded returning command. Do
        # not synthesize/send a direct Home jump in this branch.
        if not self._command_pair_fresh(now_ns) or any(command.mode != "returning" for command in self._commands.values()):
            self._last_error = "bounded fault-return command stale or missing"
            return False
        self._send_commands(self._commands["left"], self._commands["right"])
        return True

    def _send_commands(self, left: ArmJointCommand, right: ArmJointCommand) -> None:
        if self._hardware is None or self._safety_locked:
            return
        output = np.concatenate([
            np.asarray(left.position_rad, dtype=np.float64),
            np.asarray(right.position_rad, dtype=np.float64),
        ])
        lower = np.tile(np.asarray(self.robot.lower_limits_rad, dtype=np.float64), 2)
        upper = np.tile(np.asarray(self.robot.upper_limits_rad, dtype=np.float64), 2)
        if not np.isfinite(output).all() or np.any(output < lower) or np.any(output > upper):
            self._trip_soft_stop("command exceeds robot hard limits")
            return
        output_deg = np.degrees(output)
        try:
            now = self._clock_seconds()
            self._hardware_safety.observe_command("left", output_deg[:7], received_at=now, frame_id="left_base_marvin_degrees")
            self._hardware_safety.observe_command("right", output_deg[7:], received_at=now, frame_id="right_base_marvin_degrees")
        except (TypeError, ValueError) as exc:
            self._trip_soft_stop(f"hardware safety rejected command: {exc}")
            return
        decision = self._hardware_safety.decide(now=self._clock_seconds())
        if decision.action == "soft_stop":
            self._trip_soft_stop(decision.reason)
            return
        if decision.left_joints_deg is not None and decision.right_joints_deg is not None:
            output_deg = np.concatenate([decision.left_joints_deg, decision.right_joints_deg])
        self._hardware.send_joint_targets(output_deg[:7], output_deg[7:])
        self._last_output_deg = output_deg
        self._command_count += 1

    def _check_feedback(self, feedback: MarvinFeedback, now_ns: int, *, received_ns: int | None = None) -> str | None:
        measured = np.concatenate([feedback.left_joints_deg, feedback.right_joints_deg])
        if self._last_output_deg is None:
            self._last_output_deg = measured.copy()
            self._hardware_safety._last_output = measured.copy()
        feedback_received_at = self._clock_seconds(now_ns)
        self._hardware_safety.observe_feedback(
            left_joints_deg=feedback.left_joints_deg,
            right_joints_deg=feedback.right_joints_deg,
            arm_states=feedback.arm_states,
            command_states=feedback.command_states,
            error_codes=feedback.error_codes,
            servo_error_reports=feedback.servo_error_reports,
            frame_serials=feedback.frame_serials,
            received_at=feedback_received_at,
        )
        if self._session_state is None:
            current_state = "returning"
            state_received_at = feedback_received_at
        else:
            current_state = "returning" if self._session_state.state == "fault" else self._session_state.state
            state_received_at = (
                feedback_received_at
                if self._session_state_received_ns is None
                else self._session_state_received_ns / 1e9
            )
        # Authority freshness is tied to the actual SessionState receive event.
        # Feedback polling must never renew an old coordinator state.
        self._hardware_safety.observe_teleop_state(
            current_state, received_at=state_received_at
        )
        if feedback.error_codes != (0, 0):
            return f"arm_error:{feedback.error_codes}"
        if feedback.servo_error_reports != ("None", "None"):
            return f"servo_error:{feedback.servo_error_reports}"
        if feedback.arm_states != (1, 1) or not command_states_compatible(feedback.command_states, 1):
            return f"invalid_arm_state:{feedback.arm_states}/{feedback.command_states}"
        lower = np.tile(np.degrees(self.robot.lower_limits_rad), 2)
        upper = np.tile(np.degrees(self.robot.upper_limits_rad), 2)
        padding = float(self.params["feedback_hard_limit_padding_deg"])
        if np.any(measured < lower - padding) or np.any(measured > upper + padding):
            return "feedback_hard_limit"
        if np.max(np.abs(self._last_output_deg - measured), initial=0.0) > float(self.params["maximum_tracking_error_deg"]):
            return "tracking_error"
        return None

    def _controlled_return(self, now_ns: int) -> None:
        if self._hardware is None or self._safety_locked:
            return
        if self._command_pair_fresh(int(now_ns)) and all(
            command.mode == "returning" for command in self._commands.values()
        ):
            self._send_commands(self._commands["left"], self._commands["right"])
            return
        # Normal command/coordinator timeout has a local bounded-home
        # failsafe.  A latched coordinator fault never uses this fallback.
        if self._session_state is not None and self._session_state.state == "fault":
            self._last_error = "fault-return command stale or missing"
            return
        if self._feedback is None:
            self._last_error = "controlled return feedback unavailable"
            return
        current = np.concatenate([self._feedback.left_joints_deg, self._feedback.right_joints_deg])
        home = np.degrees(np.asarray(self.robot.home_all, dtype=np.float64))
        step = float(self.params["return_max_speed_deg_s"]) / self._rate_hz
        target = current + np.clip(home - current, -step, step)
        left = ArmJointCommand(
            1, self._state_sequence, int(now_ns), "coordinator", "left",
            "returning", None, None, list(ARM_JOINT_NAMES["left"]),
            np.radians(target[:7]).tolist(), self.coordinator_instance_id, self.router_zid,
        )
        right = ArmJointCommand(
            1, self._state_sequence, int(now_ns), "coordinator", "right",
            "returning", None, None, list(ARM_JOINT_NAMES["right"]),
            np.radians(target[7:]).tolist(), self.coordinator_instance_id, self.router_zid,
        )
        self._send_commands(left, right)

    def _trip_soft_stop(self, reason: str) -> None:
        self._last_error = reason
        self._phase = "soft_stopped"
        self._safety_locked = True
        if self._hardware is not None:
            try:
                self._hardware.soft_stop_once()
            except Exception:
                pass

    def tick(self, *, now_ns: int | None = None) -> None:
        now_ns = int(self.clock()) if now_ns is None else int(now_ns)
        self._state_sequence += 1
        if self._safety_locked:
            self._publish_status()
            return
        if self._hardware is None:
            return
        try:
            feedback = self._hardware.read_feedback()
            unsafe = self._check_feedback(feedback, now_ns)
            self._feedback = feedback
            if unsafe is None:
                unsafe = self._hardware_safety.feedback_unsafe_reason(now=self._clock_seconds(now_ns))
            if unsafe:
                self._trip_soft_stop(unsafe)
                self._publish_status()
                return
            if self._phase == "fault_return":
                self._bounded_fault_return(now_ns)
            elif self._session_state is None or self._session_state.state != "teleop":
                if self._session_state is not None and self._session_state.state == "returning":
                    self._phase = "returning"
                self._controlled_return(now_ns)
            elif not self._command_pair_fresh(now_ns):
                self._phase = "returning"
                self._controlled_return(now_ns)
            else:
                self._phase = "teleop"
                self._send_commands(self._commands["left"], self._commands["right"])
            self._publish_state()
        except BaseException as exc:
            self._trip_soft_stop(f"runtime_error: {exc}")

    def _publish_state(self) -> None:
        state = self.arm_state
        if state is not None:
            _put(self._publishers.get("state"), state.to_dict())

    def _make_status(self, ready: bool, healthy: bool, phase: str) -> ComponentStatus:
        self._status_sequence += 1
        return ComponentStatus(
            1, self._status_sequence, int(self.clock()), "executor_arm", "marvin",
            phase, ready, healthy, ["real"], self._last_error,
            {"safety_locked": self._safety_locked, "commands_sent": self._command_count, "rad_boundary": "sdk_only"},
            self.publisher_instance_id, self.router_zid,
        )

    def _publish_status(self) -> None:
        status = self._make_status(
            ready=self._phase in {"armed_idle", "teleop", "returning", "fault_return"} and not self._safety_locked,
            healthy=not self._safety_locked and self._phase != "failed",
            phase=self._phase,
        )
        self._status = status
        _put(self._publishers.get("status"), status.to_dict())

    def run(self) -> None:
        period = 1.0 / self._rate_hz
        next_tick = time.monotonic()
        while True:
            next_tick += period
            self.tick()
            self._publish_status()
            time.sleep(max(0.0, next_tick - time.monotonic()))
    def close(self) -> None:
        self._release_hardware()
        if self._liveliness_token is not None:
            try:
                self._liveliness_token.undeclare()
            except Exception:
                pass
            self._liveliness_token = None
        for resource in (*self._subscriptions, *self._publishers.values()):
            try:
                if resource is not None:
                    resource.undeclare()
            except (AttributeError, RuntimeError):
                pass
        self._subscriptions.clear()
        self._publishers.clear()


def main(argv: list[str] | None = None) -> int:
    """Canonical product entry; preflight provider is process-issued and mandatory."""
    parser = argparse.ArgumentParser(description="canonical Marvin executor")
    parser.add_argument("--confirm-real", action="store_true")
    parser.add_argument("--robot-ip", default=os.environ.get("MARVIN_ROBOT_IP", ""))
    parser.add_argument("--config", default="")
    args = parser.parse_args(argv)
    if not args.confirm_real:
        print("Marvin executor requires --confirm-real", flush=True)
        return 2
    provider_spec = os.environ.get("TIANJI_REAL_CAPABILITY_PROVIDER", "")
    if ":" not in provider_spec:
        raise RuntimeError("TIANJI_REAL_CAPABILITY_PROVIDER=module:callable is required")
    module_name, attribute = provider_spec.split(":", 1)
    import importlib
    provider = getattr(importlib.import_module(module_name), attribute)
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "")
    coordinator = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID", "")
    if not instance or not coordinator:
        raise RuntimeError("TIANJI_COMPONENT_INSTANCE_ID and TIANJI_COORDINATOR_INSTANCE_ID are required")
    params = {"robot_ip": args.robot_ip} if args.robot_ip else {}
    authorities_raw = os.environ.get("TIANJI_AUTHORITIES", "")
    if authorities_raw:
        try:
            authorities = json.loads(authorities_raw)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"TIANJI_AUTHORITIES is not valid JSON: {exc}") from exc
        if not isinstance(authorities, Mapping):
            raise RuntimeError("TIANJI_AUTHORITIES must be a JSON object")
        params["expected_authorities"] = {
            role: authorities[role]
            for role in ("source", "producer_arm")
            if isinstance(authorities.get(role), Mapping)
        }
    if args.config:
        import yaml
        configured = yaml.safe_load(open(args.config, encoding="utf-8")) or {}
        if not isinstance(configured, Mapping):
            raise RuntimeError("Marvin executor config must be a mapping")
        configured.update(params)
        params = configured
    session = open_session()
    node = None
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        node = MarvinExecutor(
            session=session, publisher_instance_id=instance, router_zid=router,
            coordinator_instance_id=coordinator, real_capability=provider,
            run_id=os.environ.get("TIANJI_RUN_ID"),
            safety_supervisor_instance_id=os.environ.get("TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID"),
            params=params,
        )
        if not node.connect():
            return 1
        node.run()
    finally:
        if node is not None:
            node.close()
        session.close()
    return 0


__all__ = ["MarvinExecutor", "main"]
