"""Controller-button observation binding for the XR/Manus session route."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict
import threading
import time
from typing import Any

from ..protocol import topics
from ..protocol.messages import strict_loads
from ..zenoh_util import ZenohJsonSub
from .operator_input import OperatorEdgeFilter, OperatorObservation
from .xr_input import XR_ARM_INPUTS, XrButtonBinding, XrFrame


_OBSERVATION_KEYS = frozenset({
    "source", "side", "sequence", "epoch", "receive_time_ns", "valid", "available",
    "action", "pressed", "confidence",
})
_WIRE_KEYS = frozenset({"schema_version", "kind", "router_zid", "publisher_instance_id", "observation"})


def _identity(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip() or "/" in value:
        raise ValueError(f"{field} must be a non-empty path-safe string")
    return value


def encode_xr_operator_observation(
    observation: OperatorObservation, *, router_zid: str, publisher_instance_id: str
) -> dict[str, Any]:
    if not isinstance(observation, OperatorObservation):
        raise TypeError("observation must be OperatorObservation")
    return {
        "schema_version": 1,
        "kind": "xr_operator_observation",
        "router_zid": _identity(router_zid, "router_zid"),
        "publisher_instance_id": _identity(publisher_instance_id, "publisher_instance_id"),
        "observation": asdict(observation),
    }


def decode_xr_operator_observation(
    value: Any, *, expected_router_zid: str | None = None,
    expected_publisher_instance_id: str | None = None,
) -> OperatorObservation:
    if not isinstance(value, Mapping) or set(value) != _WIRE_KEYS:
        raise ValueError("invalid XR operator observation envelope")
    if value["schema_version"] != 1 or value["kind"] != "xr_operator_observation":
        raise ValueError("unsupported XR operator observation envelope")
    router_zid = _identity(value["router_zid"], "router_zid")
    publisher = _identity(value["publisher_instance_id"], "publisher_instance_id")
    if expected_router_zid is not None and router_zid != expected_router_zid:
        raise ValueError("XR operator router identity is not authorized")
    if expected_publisher_instance_id is not None and publisher != expected_publisher_instance_id:
        raise ValueError("XR operator publisher identity is not authorized")
    payload = value["observation"]
    if not isinstance(payload, Mapping) or set(payload) != _OBSERVATION_KEYS:
        raise ValueError("invalid XR operator observation payload")
    return OperatorObservation(**dict(payload))


class XrControllerOperatorConfig:
    """Validated action bindings shared by the source and target processes."""

    def __init__(self, *, start: XrButtonBinding, home: XrButtonBinding,
                 clutch: XrButtonBinding, freshness_ns: int = 200_000_000,
                 stable_ns: int = 800_000_000):
        for value in (start, home, clutch):
            if not isinstance(value, XrButtonBinding):
                raise TypeError("XR operator bindings must be XrButtonBinding values")
        if type(freshness_ns) is not int or freshness_ns <= 0:
            raise ValueError("freshness_ns must be a positive integer")
        if type(stable_ns) is not int or stable_ns < 0:
            raise ValueError("stable_ns must be a non-negative integer")
        self.start = start
        self.home = home
        self.clutch = clutch
        self.freshness_ns = freshness_ns
        self.stable_ns = stable_ns

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "XrControllerOperatorConfig":
        if not isinstance(value, Mapping):
            raise ValueError("xr operator config must be a mapping")
        allowed = {"start", "home", "clutch", "freshness_ns", "stable_ns"}
        extra = set(value) - allowed
        if extra or not {"start", "home", "clutch"} <= set(value):
            raise ValueError("xr operator config requires start, home and clutch bindings")

        def binding(item: Any, field: str) -> XrButtonBinding:
            if not isinstance(item, Mapping) or set(item) != {"side", "control", "threshold"}:
                raise ValueError(f"{field} must contain side, control and threshold")
            return XrButtonBinding(item["side"], item["control"], item["threshold"])

        return cls(
            start=binding(value["start"], "start"),
            home=binding(value["home"], "home"),
            clutch=binding(value["clutch"], "clutch"),
            freshness_ns=value.get("freshness_ns", 200_000_000),
            stable_ns=value.get("stable_ns", 800_000_000),
        )


class XrControllerOperatorPublisher:
    """Convert SDK analog controls into four explicit action observations."""

    def __init__(
        self,
        *,
        source_instance_id: str,
        epoch: int,
        start: XrButtonBinding,
        home: XrButtonBinding,
        clutch: XrButtonBinding,
    ) -> None:
        self.source_instance_id = _identity(source_instance_id, "source_instance_id")
        if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
            raise ValueError("epoch must be a non-negative integer")
        self.epoch = epoch
        for value in (start, home, clutch):
            if not isinstance(value, XrButtonBinding):
                raise TypeError("XR operator bindings must be XrButtonBinding values")
        self._bindings = {
            "start_request": start,
            "home_request": home,
            "clutch_press": clutch,
            "clutch_release": clutch,
        }

    @classmethod
    def from_config(cls, *, source_instance_id: str, epoch: int,
                    config: XrControllerOperatorConfig) -> "XrControllerOperatorPublisher":
        if not isinstance(config, XrControllerOperatorConfig):
            raise TypeError("config must be XrControllerOperatorConfig")
        return cls(source_instance_id=source_instance_id, epoch=epoch,
                   start=config.start, home=config.home, clutch=config.clutch)

    def observations(self, frame: XrFrame) -> tuple[OperatorObservation, ...]:
        if not isinstance(frame, XrFrame):
            raise TypeError("frame must be XrFrame")
        result: list[OperatorObservation] = []
        for action, binding in self._bindings.items():
            controller = frame.controller(binding.side)
            available = bool(controller.available and controller.valid)
            pressed = bool(binding.pressed(frame)) if available else False
            if action == "clutch_release":
                pressed = not pressed if available else False
            result.append(OperatorObservation(
                source=self.source_instance_id,
                side=binding.side,
                sequence=frame.sequence,
                epoch=self.epoch,
                receive_time_ns=frame.received_timestamp_ns,
                valid=available,
                available=available,
                action=action,
                pressed=pressed,
                confidence=1.0 if available else 0.0,
            ))
        return tuple(result)

    def payloads(self, frame: XrFrame, *, router_zid: str) -> tuple[dict[str, Any], ...]:
        return tuple(
            encode_xr_operator_observation(
                observation,
                router_zid=router_zid,
                publisher_instance_id=self.source_instance_id,
            )
            for observation in self.observations(frame)
        )


class XrControllerOperatorSessionBinding:
    """Target-side, fail-closed controller event binding.

    The source publishes observations only.  This class owns debounce, session
    requests, and clutch callbacks, so an XR process cannot authorize motion
    by itself.
    """

    def __init__(
        self,
        session: Any,
        *,
        router_zid: str,
        publisher_instance_id: str,
        observation_instance_id: str,
        connection_generation: int,
        config: XrControllerOperatorConfig,
        request_start: Any,
        request_return: Any,
        set_clutch: Any,
        armed: Any,
        clock: Any = time.monotonic_ns,
    ) -> None:
        for identity in (router_zid, publisher_instance_id, observation_instance_id):
            if not isinstance(identity, str) or not identity.strip() or "/" in identity:
                raise ValueError("explicit XR operator identities are required")
        if type(connection_generation) is not int or connection_generation < 0:
            raise ValueError("connection_generation must be a non-negative integer")
        if not isinstance(config, XrControllerOperatorConfig):
            raise TypeError("config must be XrControllerOperatorConfig")
        if not all(callable(value) for value in (request_start, request_return, set_clutch, armed, clock)):
            raise ValueError("XR operator callbacks and clock are required")
        self._router = router_zid
        self._publisher = publisher_instance_id
        self._observation = observation_instance_id
        self._generation = connection_generation
        self._config = config
        self._clock = clock
        self._request_start = request_start
        self._request_return = request_return
        self._set_clutch = set_clutch
        self._armed = armed
        self._lock = threading.RLock()
        self._closed = False
        self._filters = self._make_filters(connection_generation)
        self._subscription = session.declare_subscriber(
            topics.XR_OPERATOR_OBSERVATION, self._receive
        )

    def _make_filters(self, generation: int) -> dict[str, OperatorEdgeFilter]:
        return {
            "start_request": OperatorEdgeFilter(
                source=self._observation, side=self._config.start.side,
                action="start_request", epoch=generation,
                freshness_ns=self._config.freshness_ns, stable_ns=self._config.stable_ns),
            "home_request": OperatorEdgeFilter(
                source=self._observation, side=self._config.home.side,
                action="home_request", epoch=generation,
                freshness_ns=self._config.freshness_ns, stable_ns=self._config.stable_ns),
            "clutch_press": OperatorEdgeFilter(
                source=self._observation, side=self._config.clutch.side,
                action="clutch_press", epoch=generation,
                freshness_ns=self._config.freshness_ns, stable_ns=self._config.stable_ns),
            "clutch_release": OperatorEdgeFilter(
                source=self._observation, side=self._config.clutch.side,
                action="clutch_release", epoch=generation,
                freshness_ns=self._config.freshness_ns, stable_ns=self._config.stable_ns),
        }

    def _invalidate(self) -> None:
        for item in self._filters.values():
            item.invalidate()

    def _receive(self, sample: Any) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                value = getattr(sample, "payload", sample)
                if hasattr(value, "to_bytes"):
                    value = value.to_bytes()
                if isinstance(value, (str, bytes, bytearray, memoryview)):
                    value = strict_loads(value)
                observation = decode_xr_operator_observation(
                    value,
                    expected_router_zid=self._router,
                    expected_publisher_instance_id=self._observation,
                )
            except (TypeError, ValueError):
                self._invalidate()
                return
            if observation.source != self._observation:
                return
            if observation.epoch < self._generation:
                return
            if observation.epoch > self._generation:
                if observation.epoch != self._generation + 1:
                    self._invalidate()
                    return
                # A reconnect is a new input epoch.  Recreate the edge filters
                # so a held controller cannot carry a pending edge across the
                # disconnect, while retaining the explicit source identity.
                self._generation = observation.epoch
                self._filters = self._make_filters(self._generation)
            if observation.action not in self._filters:
                return
            valid = observation.valid and observation.available and bool(self._armed())
            if not valid:
                observation = OperatorObservation(
                    source=observation.source, side=observation.side,
                    sequence=observation.sequence, epoch=observation.epoch,
                    receive_time_ns=observation.receive_time_ns, valid=False,
                    available=observation.available, action=observation.action,
                    pressed=False, confidence=observation.confidence,
                )
            event = self._filters[observation.action].update(
                observation, now_ns=int(self._clock())
            )
            if event is None:
                return
            if event.action == "start_request":
                self._request_start()
            elif event.action == "home_request":
                self._request_return("xr_controller_home")
            elif event.action == "clutch_press":
                self._set_clutch(event.side, True)
            elif event.action == "clutch_release":
                self._set_clutch(event.side, False)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            self._subscription.undeclare()
        except AttributeError:
            self._subscription.close()


def bind_from_environment(
    environment: Mapping[str, str], *, session: Any, node: Any,
    config: Mapping[str, Any],
) -> XrControllerOperatorSessionBinding | None:
    """Create the XR controller binding only for the managed XR/Manus route.

    The default is deliberately ``keyboard``.  This is important for the
    existing PICO2 gesture route: importing this module or starting a PICO2
    session must not create an XR subscription.  The resolved session envelope
    is checked as an admission boundary so a controller observation cannot be
    reused by another profile.
    """
    mode = environment.get("TIANJI_XR_OPERATOR_INPUT", "keyboard")
    if mode == "keyboard":
        return None
    if (
        mode != "controller"
        or environment.get("TIANJI_REQUIRED_CAPABILITY") != "simulation"
        or environment.get("TIANJI_REQUIRED_OBSERVATION_PROFILE") != "manus"
    ):
        raise ValueError("XR/Manus controller binding requires explicit managed simulation")
    try:
        resolved = strict_loads(environment.get("TIANJI_RESOLVED_DUAL_SESSION", "{}"))
    except (TypeError, ValueError) as exc:
        raise ValueError("XR controller binding requires a valid resolved session") from exc
    if (
        not isinstance(resolved, Mapping)
        or resolved.get("profile") != "vr_manus_xr_sim"
        or not isinstance(resolved.get("config"), Mapping)
        or resolved["config"].get("input_mode") != "vr_manus"
        or resolved["config"].get("operator_input") != "controller"
        or resolved["config"].get("arm_input") not in XR_ARM_INPUTS
    ):
        raise ValueError("XR controller binding requires matching resolved XR/Manus session")
    if not isinstance(config, Mapping):
        raise ValueError("XR controller binding requires the target configuration")
    operator_config = XrControllerOperatorConfig.from_mapping(config.get("operator_config"))
    router_zid = environment.get("TIANJI_ROUTER_ZID")
    target_instance_id = environment.get("TIANJI_COMPONENT_INSTANCE_ID")
    observation_instance_id = environment.get("TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID")
    return XrControllerOperatorSessionBinding(
        session,
        router_zid=router_zid,
        publisher_instance_id=target_instance_id,
        observation_instance_id=observation_instance_id,
        connection_generation=1,
        config=operator_config,
        request_start=lambda: node.request_start(reason="xr_controller_start"),
        request_return=lambda reason: node.request_return(reason),
        set_clutch=lambda side, pressed: node.set_clutch(side, pressed),
        armed=lambda: node.phase in {"armed", "start_pending", "teleop"},
    )


__all__ = [
    "XrControllerOperatorConfig",
    "XrControllerOperatorSessionBinding",
    "XrControllerOperatorPublisher",
    "bind_from_environment",
    "decode_xr_operator_observation",
    "encode_xr_operator_observation",
]
