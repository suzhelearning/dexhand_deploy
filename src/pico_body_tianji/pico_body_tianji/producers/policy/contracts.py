"""可替换 arm policy producer 的纯 Python contract 与适配器。

Policy producer 与 IK producer 共用 canonical ``ArmJointProposal`` wire contract，
但不依赖或包装 IK solver。本模块没有 Zenoh 依赖，方便对 observation、
action safety boundary 和 hold runner 做确定性测试。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import os
from pathlib import Path
import time
from typing import Any, Mapping, Protocol, Sequence

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...protocol.messages import (
    ARM_JOINT_NAMES,
    ArmJointProposal,
    ArmJointState,
    ArmTargetCommand,
    ProtocolError,
    SessionState,
)


POLICY_ACTION_MODES = (
    "absolute_position_rad",
    "delta_position_rad",
    "velocity_rad_s",
)


class ActionValidationError(ValueError):
    """Policy action 无法安全转换为 proposal。"""


@dataclass(frozen=True)
class PolicyObservation:
    """Policy runner 的最小输入。

    ``joint_state`` 已经通过 executor freshness 检查；若原 wire state 缺少
    velocity，``ObservationBuilder`` 会在这里放入相邻 frame 的有限差分。
    """

    joint_state: ArmJointState
    arm_targets: Mapping[str, ArmTargetCommand] | None = None
    session_state: SessionState | None = None

    @property
    def position_rad(self) -> list[float]:
        return list(self.joint_state.position_rad)

    @property
    def velocity_rad_s(self) -> list[float] | None:
        return None if self.joint_state.velocity_rad_s is None else list(self.joint_state.velocity_rad_s)


@dataclass(frozen=True)
class PolicyAction:
    """Runner 输出；shape、finite、limits 和 step 由 ``ActionAdapter`` 校验。"""

    mode: str
    values: Sequence[float]

    def __post_init__(self) -> None:
        if self.mode not in POLICY_ACTION_MODES:
            raise ActionValidationError(
                f"mode must be one of {', '.join(POLICY_ACTION_MODES)}"
            )
        # Freeze the container boundary without rejecting malformed values here:
        # producer must turn those into an unhealthy status, not a placeholder.
        try:
            object.__setattr__(self, "values", list(self.values))
        except (TypeError, ValueError) as exc:
            raise ActionValidationError("values must be a finite numeric sequence") from exc


class PolicyRunner(Protocol):
    """可替换 policy 的最小接口。"""

    loaded: bool
    healthy: bool

    def step(self, observation: PolicyObservation) -> PolicyAction:
        ...


class ObservationBuilder:
    """构建带 freshness/velocity guarantee 的 policy observation。

    ``build`` 在 observation 尚不可安全使用时返回 ``None``，并在
    ``last_reason`` 中保留可诊断原因。timestamp 使用 wire 的接收主机时钟；
    为 deterministic tests 可通过 ``now_ns`` 或 ``clock`` 注入。
    """

    def __init__(
        self,
        stale_timeout_s: float = 0.2,
        *,
        velocity_interval_s: float | None = None,
        timeout_s: float | None = None,
        clock: Any = time.monotonic_ns,
    ) -> None:
        if timeout_s is not None:
            stale_timeout_s = timeout_s
        if not math.isfinite(float(stale_timeout_s)) or float(stale_timeout_s) <= 0.0:
            raise ValueError("stale_timeout_s must be finite and positive")
        if velocity_interval_s is None:
            velocity_interval_s = stale_timeout_s
        if not math.isfinite(float(velocity_interval_s)) or float(velocity_interval_s) <= 0.0:
            raise ValueError("velocity_interval_s must be finite and positive")
        self.stale_timeout_ns = int(float(stale_timeout_s) * 1e9)
        self.velocity_interval_ns = int(float(velocity_interval_s) * 1e9)
        self.clock = clock
        self._previous: ArmJointState | None = None
        self._last_observation: PolicyObservation | None = None
        self._last_reason = "not observed"

    @property
    def ready(self) -> bool:
        return self._last_observation is not None

    @property
    def last_reason(self) -> str:
        return self._last_reason

    @property
    def last_observation(self) -> PolicyObservation | None:
        return self._last_observation

    def reset(self) -> None:
        self._previous = None
        self._last_observation = None
        self._last_reason = "reset"

    def _coerce_state(self, value: ArmJointState | Mapping[str, Any]) -> ArmJointState:
        if isinstance(value, ArmJointState):
            return value
        try:
            return ArmJointState.from_dict(value)
        except (TypeError, ValueError, ProtocolError) as exc:
            raise ValueError(f"invalid arm executor state: {exc}") from exc

    def build(
        self,
        joint_state: ArmJointState | Mapping[str, Any],
        arm_targets: Mapping[str, ArmTargetCommand] | None = None,
        session_state: SessionState | None = None,
        *,
        now_ns: int | None = None,
    ) -> PolicyObservation | None:
        state = self._coerce_state(joint_state)
        now = int(self.clock() if now_ns is None else now_ns)
        age = now - int(state.timestamp_ns)
        # A future timestamp is not a fresh state: accepting it would make an
        # untrusted producer control the local freshness clock.
        if age < 0 or age > self.stale_timeout_ns:
            self._last_observation = None
            self._last_reason = f"state stale (age_ns={age})"
            self._previous = state
            return None

        velocity = state.velocity_rad_s
        if velocity is None:
            previous = self._previous
            if previous is None:
                self._last_observation = None
                self._last_reason = "velocity unavailable; need adjacent state"
                self._previous = state
                return None
            delta_ns = int(state.timestamp_ns) - int(previous.timestamp_ns)
            if delta_ns <= 0 or delta_ns > self.velocity_interval_ns:
                self._last_observation = None
                self._last_reason = f"velocity interval invalid ({delta_ns} ns)"
                self._previous = state
                return None
            velocity = [
                (float(current) - float(old)) / (delta_ns / 1e9)
                for current, old in zip(state.position_rad, previous.position_rad)
            ]
            if len(velocity) != 14 or not all(math.isfinite(item) for item in velocity):
                self._last_observation = None
                self._last_reason = "estimated velocity is non-finite"
                self._previous = state
                return None
            state = replace(state, velocity_rad_s=velocity)

        # ArmJointState has already validated shape/finiteness, but check the
        # generated state too so custom test doubles cannot bypass this gate.
        if len(state.position_rad) != 14 or not all(math.isfinite(x) for x in state.position_rad):
            self._last_observation = None
            self._last_reason = "joint position is malformed"
            self._previous = state
            return None
        if len(velocity) != 14 or not all(math.isfinite(x) for x in velocity):
            self._last_observation = None
            self._last_reason = "joint velocity is malformed"
            self._previous = state
            return None

        if arm_targets is not None:
            arm_targets = dict(arm_targets)
        observation = PolicyObservation(state, arm_targets, session_state)
        self._previous = state
        self._last_observation = observation
        self._last_reason = "ready"
        return observation

    build_observation = build


class ActionAdapter:
    """把 14 维 policy action 转成左右 canonical ``ArmJointProposal``。

    Limits 与 maximum step 在 producer 边界校验。任何拒绝都抛出
    ``ActionValidationError``，调用方必须置 producer unhealthy 并停止 proposal；
    绝不生成 ``accepted=false`` 或其它占位 proposal。
    """

    def __init__(
        self,
        robot_config: ArmRobotConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        *,
        publisher_instance_id: str | None = None,
        router_zid: str | None = None,
        maximum_step_rad: float | None = None,
        maximum_command_step_rad: float | None = None,
        max_step_rad: float | None = None,
        producer: str = "policy_hold",
        control_period_s: float = 1.0 / 90.0,
        dt_s: float | None = None,
    ) -> None:
        if isinstance(robot_config, ArmRobotConfig):
            self.robot = robot_config
        elif isinstance(robot_config, Mapping):
            self.robot = ArmRobotConfig.from_mapping(robot_config)
        else:
            self.robot = ArmRobotConfig.load(robot_config)
        if maximum_step_rad is None:
            maximum_step_rad = maximum_command_step_rad
        if maximum_step_rad is None:
            maximum_step_rad = max_step_rad
        if maximum_step_rad is None:
            maximum_step_rad = self._load_default_step()
        if not math.isfinite(float(maximum_step_rad)) or float(maximum_step_rad) <= 0.0:
            raise ValueError("maximum_step_rad must be finite and positive")
        period = control_period_s if dt_s is None else dt_s
        if not math.isfinite(float(period)) or float(period) <= 0.0:
            raise ValueError("control_period_s must be finite and positive")
        self.maximum_step_rad = float(maximum_step_rad)
        self.control_period_s = float(period)
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.producer = str(producer)

    @staticmethod
    def _load_default_step() -> float:
        path = Path(__file__).resolve().parents[4] / "src" / "pico_body_tianji" / "config" / "coordinator" / "arm.yaml"
        try:
            import yaml
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            return float(data["maximum_command_step_rad"])
        except Exception:
            # Keep the adapter usable in an installed bundle where the config
            # path is supplied through robot config. This is still finite and
            # conservative; callers can (and production does) pass the config value.
            return 0.0132645022

    @staticmethod
    def _action_values(action: PolicyAction | Mapping[str, Any]) -> tuple[str, list[float]]:
        if isinstance(action, Mapping):
            try:
                action = PolicyAction(str(action["mode"]), action["values"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ActionValidationError(f"malformed action: {exc}") from exc
        if not isinstance(action, PolicyAction):
            raise ActionValidationError("action must be PolicyAction")
        if action.mode not in POLICY_ACTION_MODES:
            raise ActionValidationError(f"unsupported action mode: {action.mode}")
        try:
            values = [float(value) for value in action.values]
        except (TypeError, ValueError) as exc:
            raise ActionValidationError("action values must be numeric") from exc
        if len(values) != 14:
            raise ActionValidationError("action values must have shape [14]")
        if not all(math.isfinite(value) for value in values):
            raise ActionValidationError("action values must be finite")
        return action.mode, values

    @staticmethod
    def _current_values(current: Any) -> list[float]:
        if isinstance(current, PolicyObservation):
            current = current.joint_state
        if isinstance(current, ArmJointState):
            values = current.position_rad
        elif isinstance(current, Mapping):
            values = current.get("position_rad")
        else:
            values = current
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise ActionValidationError("current position must be numeric") from exc
        if len(result) != 14 or not all(math.isfinite(value) for value in result):
            raise ActionValidationError("current position must have finite shape [14]")
        return result

    def convert_positions(
        self,
        action: PolicyAction | Mapping[str, Any],
        current_position_rad: Any,
        *,
        control_period_s: float | None = None,
    ) -> list[float]:
        mode, values = self._action_values(action)
        current = self._current_values(current_position_rad)
        period = self.control_period_s if control_period_s is None else float(control_period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise ActionValidationError("control period must be finite and positive")
        if mode == "absolute_position_rad":
            result = values
        elif mode == "delta_position_rad":
            result = [old + delta for old, delta in zip(current, values)]
        else:
            result = [old + velocity * period for old, velocity in zip(current, values)]

        for index, (new, old, lower, upper) in enumerate(
            zip(result, current, self.robot.lower_limits_rad * 2, self.robot.upper_limits_rad * 2)
        ):
            if not math.isfinite(new):
                raise ActionValidationError(f"action result {index} is non-finite")
            if not lower <= new <= upper:
                raise ActionValidationError(f"action result {index} exceeds joint limits")
            if abs(new - old) > self.maximum_step_rad + 1e-12:
                raise ActionValidationError(f"action result {index} exceeds maximum step")
        return result

    # Concise alias useful to policy implementations and tests.
    adapt_positions = convert_positions

    def adapt(
        self,
        action: PolicyAction | Mapping[str, Any],
        current_position_rad: Any,
        *,
        sequence: int = 1,
        timestamp_ns: int | None = None,
        target_sequences: Mapping[str, int | None] | None = None,
        arm_targets: Mapping[str, ArmTargetCommand] | None = None,
        producer: str | None = None,
        publisher_instance_id: str | None = None,
        router_zid: str | None = None,
        control_period_s: float | None = None,
    ) -> dict[str, ArmJointProposal]:
        positions = self.convert_positions(action, current_position_rad, control_period_s=control_period_s)
        instance = publisher_instance_id or self.publisher_instance_id
        router = router_zid or self.router_zid
        if not instance or not router:
            raise ValueError("publisher_instance_id and router_zid are required")
        if isinstance(sequence, bool) or int(sequence) < 0:
            raise ActionValidationError("sequence must be non-negative integer")
        sequence = int(sequence)
        timestamp = int(time.monotonic_ns() if timestamp_ns is None else timestamp_ns)
        if timestamp < 0:
            raise ActionValidationError("timestamp_ns must be non-negative")
        if target_sequences is None:
            inferred: dict[str, int | None] = {}
            for side in ("left", "right"):
                target = arm_targets.get(side) if arm_targets else None
                if target is None:
                    inferred[side] = None
                elif isinstance(target, ArmTargetCommand):
                    inferred[side] = target.envelope.sequence
                elif isinstance(target, Mapping):
                    envelope = target.get("envelope")
                    if isinstance(envelope, Mapping):
                        inferred[side] = envelope.get("sequence")
                    else:
                        inferred[side] = target.get("sequence")
                else:
                    raise ActionValidationError(f"{side} arm target is malformed")
            target_sequences = inferred
        selected_producer = self.producer if producer is None else str(producer)
        proposals: dict[str, ArmJointProposal] = {}
        for side, offset in (("left", 0), ("right", 7)):
            proposals[side] = ArmJointProposal(
                schema_version=1,
                sequence=sequence,
                timestamp_ns=timestamp,
                producer=selected_producer,
                side=side,
                target_sequence=target_sequences.get(side),
                names=list(ARM_JOINT_NAMES[side]),
                position_rad=positions[offset : offset + 7],
                diagnostics={"action_mode": self._action_values(action)[0]},
                publisher_instance_id=str(instance),
                router_zid=str(router),
            )
        return proposals

    to_proposals = adapt


class HoldPolicyRunner:
    """安全 reference policy：原样保持最新 executor position。"""

    def __init__(self, *, loaded: bool = True) -> None:
        self.loaded = bool(loaded)
        self._last_error: str | None = None

    @property
    def healthy(self) -> bool:
        return self.loaded and self._last_error is None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def step(self, observation: PolicyObservation) -> PolicyAction:
        if not self.loaded:
            self._last_error = "hold policy is not loaded"
            raise RuntimeError(self._last_error)
        if not isinstance(observation, PolicyObservation):
            self._last_error = "observation must be PolicyObservation"
            raise ValueError(self._last_error)
        values = list(observation.joint_state.position_rad)
        if len(values) != 14 or not all(math.isfinite(value) for value in values):
            self._last_error = "observation position is malformed"
            raise ValueError(self._last_error)
        self._last_error = None
        return PolicyAction("absolute_position_rad", values)


__all__ = [
    "POLICY_ACTION_MODES",
    "ActionAdapter",
    "ActionValidationError",
    "HoldPolicyRunner",
    "ObservationBuilder",
    "PolicyAction",
    "PolicyObservation",
    "PolicyRunner",
]
