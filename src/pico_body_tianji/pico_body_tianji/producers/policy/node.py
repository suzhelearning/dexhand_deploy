"""可运行的 hold policy arm producer。

该节点只订阅 executor state/session state，输出 typed ``ArmJointProposal`` 和
``producer_arm`` ``ComponentStatus``；绝不发布 coordinator final command，也不
依赖 IK solver。
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any, Mapping

from ...protocol import topics
from ...protocol.messages import (
    ArmJointProposal,
    ArmJointState,
    ArmTargetCommand,
    ComponentStatus,
    ProtocolError,
    SessionState,
    strict_loads,
)
from .contracts import (
    ActionAdapter,
    ActionValidationError,
    HoldPolicyRunner,
    ObservationBuilder,
    PolicyAction,
    PolicyObservation,
    PolicyRunner,
)


class PolicyProducerNode:
    """在 coordinator authority 下运行一个 arm policy producer。

    ``publisher_instance_id`` 和 ``router_zid`` 必须由 launcher 预分配/验证；
    这里不生成默认 UUID，不从 topic 或 liveliness 推断身份。
    """

    def __init__(
        self,
        session: Any = None,
        *,
        publisher_instance_id: str,
        router_zid: str,
        producer_id: str = "policy_hold",
        policy_name: str = "hold",
        runner: PolicyRunner | None = None,
        observation_builder: ObservationBuilder | None = None,
        action_adapter: ActionAdapter | None = None,
        stale_timeout_s: float = 0.2,
        control_rate_hz: float = 90.0,
        maximum_step_rad: float | None = None,
        coordinator_instance_id: str | None = None,
        capabilities: tuple[str, ...] = ("simulation",),
        clock: Any = time.monotonic_ns,
    ) -> None:
        if not publisher_instance_id or not router_zid:
            raise ValueError("publisher_instance_id and router_zid are required")
        if policy_name != "hold":
            raise ValueError("only the hold policy is registered")
        if not producer_id:
            raise ValueError("producer_id is required")
        if control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if not capabilities or not ({"simulation", "real"} & set(capabilities)):
            raise ValueError("capabilities must include simulation or real")
        self.session = session
        self.publisher_instance_id = str(publisher_instance_id)
        self.router_zid = str(router_zid)
        self.producer_id = str(producer_id)
        self.policy_name = policy_name
        self.coordinator_instance_id = coordinator_instance_id
        self.capabilities = tuple(capabilities)
        self.clock = clock
        self.control_rate_hz = float(control_rate_hz)
        self.runner = runner or HoldPolicyRunner()
        self.observation_builder = observation_builder or ObservationBuilder(
            stale_timeout_s=stale_timeout_s,
            clock=clock,
        )
        self.action_adapter = action_adapter or ActionAdapter(
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
            producer=self.producer_id,
            maximum_step_rad=maximum_step_rad,
            control_period_s=1.0 / self.control_rate_hz,
        )
        self._state: ArmJointState | None = None
        self._state_baseline: tuple[str, int] | None = None
        self._session_state: SessionState | None = None
        self._session_baseline: tuple[str, int] | None = None
        self._targets: dict[str, ArmTargetCommand] = {}
        self._target_baseline: dict[str, tuple[str, int]] = {}
        self._subscriptions: list[Any] = []
        self._publishers: dict[str, Any] = {}
        self._liveliness_token: Any = None
        self._sequence = 0
        self._started = False
        self._closed = False
        self._last_error: str | None = None
        self._session_invalid = False
        self._session_error: str | None = None
        self._status: ComponentStatus | None = None
        self._last_observation: PolicyObservation | None = None

        self._setup_transport()

    @property
    def status(self) -> ComponentStatus:
        if self._status is None:
            return self._make_status(phase="startup", ready=False)
        return self._status

    @property
    def latest_observation(self) -> PolicyObservation | None:
        return self._last_observation

    @property
    def proposals_published(self) -> int:
        return self._sequence

    def _setup_transport(self) -> None:
        if self.session is None:
            return
        if hasattr(self.session, "declare_publisher"):
            self._publishers["status"] = self.session.declare_publisher(topics.PRODUCER_STATUS)
            self._publishers["left"] = self.session.declare_publisher(topics.arm_proposal("left"))
            self._publishers["right"] = self.session.declare_publisher(topics.arm_proposal("right"))

    @staticmethod
    def _put(publisher: Any, payload: Mapping[str, Any]) -> None:
        if publisher is None:
            return
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        try:
            publisher.put(encoded, encoding="application/json")
        except TypeError:
            publisher.put(encoded)

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    def _make_status(
        self,
        *,
        phase: str,
        ready: bool,
        healthy: bool | None = None,
        error: str | None = None,
    ) -> ComponentStatus:
        status_error = self._session_error if self._session_invalid else error
        if healthy is None:
            healthy = (
                bool(getattr(self.runner, "healthy", True))
                and self._last_error is None
                and not self._session_invalid
            )
        status = ComponentStatus(
            schema_version=1,
            sequence=self._next_sequence(),
            timestamp_ns=int(self.clock()),
            component_role="producer_arm",
            component_id=self.producer_id,
            phase=phase,
            ready=bool(ready),
            healthy=bool(healthy),
            capabilities=list(self.capabilities),
            error=status_error,
            diagnostics={
                "observation_reason": self.observation_builder.last_reason,
                "coordinator_instance_id": self.coordinator_instance_id,
            },
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
        )
        self._status = status
        self._put(self._publishers.get("status"), status.to_dict())
        return status

    def _set_error(self, error: str) -> None:
        self._last_error = str(error)

    @staticmethod
    def _payload(sample: Any) -> Mapping[str, Any]:
        payload = getattr(sample, "payload", sample)
        if isinstance(payload, Mapping):
            return payload
        raw = bytes(payload)
        if not raw:
            raise ProtocolError("empty policy producer payload")
        return strict_loads(raw)

    def on_arm_state(self, value: ArmJointState | Mapping[str, Any] | Any) -> None:
        try:
            if isinstance(value, ArmJointState):
                parsed = value
            else:
                parsed = ArmJointState.from_dict(self._payload(value))
            if parsed.router_zid != self.router_zid:
                raise ProtocolError("arm state router_zid mismatch")
            baseline = self._state_baseline
            if baseline is not None:
                if parsed.publisher_instance_id != baseline[0]:
                    if self._session_state is not None and self._session_state.state == "teleop":
                        raise ProtocolError("arm state executor instance changed during teleop")
                elif parsed.sequence < baseline[1]:
                    raise ProtocolError("arm state sequence rollback")
                elif parsed.sequence == baseline[1]:
                    return
            self._state_baseline = (parsed.publisher_instance_id, parsed.sequence)
            self._state = parsed
            self._last_error = None
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._state = None
            self._set_error(f"malformed arm state: {exc}")

    def on_session_state(self, value: SessionState | Mapping[str, Any] | Any) -> None:
        try:
            if isinstance(value, SessionState):
                parsed = value
            else:
                parsed = SessionState.from_dict(self._payload(value))
            if parsed.router_zid != self.router_zid:
                raise ProtocolError("session state router_zid mismatch")
            if (
                self.coordinator_instance_id is not None
                and parsed.publisher_instance_id != self.coordinator_instance_id
            ):
                raise ProtocolError("session state coordinator identity mismatch")
            baseline = self._session_baseline
            if baseline is not None:
                if parsed.publisher_instance_id != baseline[0]:
                    raise ProtocolError("coordinator instance changed")
                if parsed.sequence < baseline[1]:
                    raise ProtocolError("session state sequence rollback")
                if parsed.sequence == baseline[1]:
                    return
            self._session_baseline = (parsed.publisher_instance_id, parsed.sequence)
            self._session_state = parsed
            self._session_invalid = False
            self._session_error = None
            self._last_error = None
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._session_state = None
            self._session_invalid = True
            self._session_error = f"malformed session state: {exc}"
            self._set_error(self._session_error)
    def on_arm_target(self, side: str, value: ArmTargetCommand | Mapping[str, Any] | Any) -> None:
        try:
            if isinstance(value, ArmTargetCommand):
                parsed = value
            else:
                parsed = ArmTargetCommand.from_dict(self._payload(value))
            if parsed.envelope.router_zid != self.router_zid or parsed.side != side:
                raise ProtocolError("arm target identity/side mismatch")
            baseline = self._target_baseline.get(side)
            if baseline is not None:
                if parsed.envelope.publisher_instance_id != baseline[0]:
                    raise ProtocolError(f"{side} arm target instance changed")
                if parsed.envelope.sequence < baseline[1]:
                    raise ProtocolError(f"{side} arm target sequence rollback")
                if parsed.envelope.sequence == baseline[1]:
                    return
            self._target_baseline[side] = (
                parsed.envelope.publisher_instance_id,
                parsed.envelope.sequence,
            )
            self._targets[side] = parsed
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._set_error(f"malformed {side} arm target: {exc}")

    # Explicit update aliases make the pure/test and transport APIs equivalent.
    update_arm_state = on_arm_state
    update_session_state = on_session_state
    update_arm_target = on_arm_target

    def _declare_subscriber(self, key: str, callback: Any) -> None:
        if self.session is None or not hasattr(self.session, "declare_subscriber"):
            return
        self._subscriptions.append(self.session.declare_subscriber(key, callback))

    def start(self) -> None:
        if self._started:
            raise RuntimeError("PolicyProducerNode already started")
        self._declare_subscriber(topics.ARM_STATE, self._on_arm_state_payload)
        self._declare_subscriber(topics.SESSION_STATE, self._on_session_state_payload)
        self._declare_subscriber(topics.arm_target("left"), lambda sample: self._on_arm_target_payload("left", sample))
        self._declare_subscriber(topics.arm_target("right"), lambda sample: self._on_arm_target_payload("right", sample))
        if self.session is not None and hasattr(self.session, "liveliness"):
            self._liveliness_token = self.session.liveliness().declare_token(
                f"tj/live/producer/arm/{self.producer_id}/{self.publisher_instance_id}"
            )
        self._started = True
        self._publish_status(phase="startup", ready=False)

    def _on_arm_state_payload(self, sample: Any) -> None:
        self.on_arm_state(sample)

    def _on_session_state_payload(self, sample: Any) -> None:
        self.on_session_state(sample)

    def _on_arm_target_payload(self, side: str, sample: Any) -> None:
        self.on_arm_target(side, sample)

    def _publish_status(self, *, phase: str, ready: bool, healthy: bool | None = None) -> ComponentStatus:
        return self._make_status(phase=phase, ready=ready, healthy=healthy, error=self._last_error)

    def tick(self, *, now_ns: int | None = None) -> dict[str, ArmJointProposal]:
        if self._closed:
            return {}
        if self._session_invalid:
            if self._last_error is None:
                self._set_error("session authority is invalid; waiting for a new snapshot")
            self._publish_status(phase="waiting_session", ready=False, healthy=False)
            return {}
        now = int(self.clock() if now_ns is None else now_ns)
        runner_healthy = bool(getattr(self.runner, "healthy", True))
        runner_loaded = bool(getattr(self.runner, "loaded", runner_healthy))
        if not runner_loaded or not runner_healthy:
            self._set_error(getattr(self.runner, "last_error", None) or "policy runner is not healthy")
            self._publish_status(phase="fault", ready=False, healthy=False)
            return {}
        if self._state is None:
            self._publish_status(phase="waiting_state", ready=False, healthy=self._last_error is None)
            return {}

        try:
            observation = self.observation_builder.build(
                self._state,
                self._targets,
                self._session_state,
                now_ns=now,
            )
        except (ProtocolError, TypeError, ValueError) as exc:
            observation = None
            self._set_error(f"observation rejected: {exc}")
        self._last_observation = observation
        if observation is None:
            # State staleness is a readiness failure, not a runner health failure.
            phase = "stale" if "stale" in self.observation_builder.last_reason or "velocity" in self.observation_builder.last_reason else "not_ready"
            self._publish_status(phase=phase, ready=False, healthy=self._last_error is None)
            return {}

        session_state = self._session_state
        if self._session_invalid:
            self._publish_status(phase="waiting_session", ready=False, healthy=False)
            return {}
        if session_state is None:
            self._publish_status(phase="waiting_session", ready=True, healthy=self._last_error is None)
            return {}
        if session_state.state != "teleop":
            self._last_error = None
            self._publish_status(phase=session_state.state, ready=True, healthy=True)
            return {}

        try:
            action = self.runner.step(observation)
            # Reserve the next wire sequence without publishing a placeholder.
            # A valid teleop status is emitted first, then proposal N+1; if the
            # action is rejected only the unhealthy status is emitted.
            proposals = self.action_adapter.adapt(
                action,
                observation,
                sequence=self._sequence + 2,
                timestamp_ns=now,
                arm_targets=self._targets,
            )
        except (ActionValidationError, ProtocolError, TypeError, ValueError, RuntimeError) as exc:
            self._set_error(f"policy action rejected: {exc}")
            self._publish_status(phase="fault", ready=False, healthy=False)
            # Crucially, no malformed or accepted=false placeholder is emitted.
            return {}

        self._last_error = None
        self._publish_status(phase="teleop", ready=True, healthy=True)
        for side, proposal in proposals.items():
            self._put(self._publishers.get(side), proposal.to_dict())
        return proposals

    step = tick

    def run(self) -> int:
        self.start()
        period = 1.0 / self.control_rate_hz
        try:
            while not self._closed:
                started = time.monotonic()
                self.tick()
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        finally:
            self.close()
        return 0

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for resource in self._subscriptions:
            try:
                resource.undeclare()
            except Exception:
                pass
        self._subscriptions.clear()
        if self._liveliness_token is not None:
            try:
                self._liveliness_token.undeclare()
            except Exception:
                pass
            self._liveliness_token = None
        for publisher in self._publishers.values():
            try:
                publisher.undeclare()
            except Exception:
                try:
                    publisher.close()
                except Exception:
                    pass
        self._publishers.clear()


def _load_policy_config() -> dict[str, Any]:
    bundle_root = os.environ.get("PICO_BODY_TIANJI_BUNDLE_ROOT")
    if bundle_root:
        path = (
            Path(bundle_root)
            / "runtime"
            / "pico_body_tianji"
            / "share"
            / "pico_body_tianji"
            / "config"
            / "producers"
            / "policy_hold.yaml"
        )
    else:
        path = (
            Path(__file__).resolve().parents[5]
            / "src"
            / "pico_body_tianji"
            / "config"
            / "producers"
            / "policy_hold.yaml"
        )
    try:
        import yaml
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise RuntimeError(f"unable to load policy config {path}: {exc}") from exc
    if not isinstance(value, Mapping) or value.get("policy") != "hold":
        raise RuntimeError("policy_hold.yaml must declare policy: hold")
    return dict(value)


def main() -> int:
    from ...zenoh_util import open_session, require_single_router
    endpoint = os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447")
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    router_zid = os.environ.get("TIANJI_ROUTER_ZID")
    coordinator_instance = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    if not instance or not router_zid or not coordinator_instance:
        raise RuntimeError(
            "TIANJI_COMPONENT_INSTANCE_ID, TIANJI_COORDINATOR_INSTANCE_ID and "
            "TIANJI_ROUTER_ZID are required"
        )
    config = _load_policy_config()
    session = open_session(endpoint)
    router_zid = require_single_router(session, router_zid)
    capabilities = tuple(str(value) for value in config.get("capabilities", ("simulation",)))
    node = PolicyProducerNode(
        session,
        publisher_instance_id=instance,
        router_zid=router_zid,
        producer_id=os.environ.get("TIANJI_PRODUCER_ID", "policy_hold"),
        coordinator_instance_id=coordinator_instance,
        control_rate_hz=float(config.get("rate_hz", 90.0)),
        stale_timeout_s=float(config.get("stale_timeout_s", 0.2)),
        maximum_step_rad=float(config.get("maximum_step_rad", 0.0132645022)),
        capabilities=capabilities,
    )
    try:
        return node.run()
    finally:
        node.close()
        session.close()


__all__ = ["PolicyProducerNode", "main"]
