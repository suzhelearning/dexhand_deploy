"""PICO controller-only source publishing canonical target contracts."""
from __future__ import annotations

import logging
import os
import time
from typing import Any

from tianji_world_output.config_loader import TianjiConfig

from ...protocol import topics
from ...sources.common.freshness import FreshnessGate
from ...sources.common.session_client import SessionClient
from ...sources.common.target_conditioner import TargetConditioningSettings
from ...sources.common.target_mapper import ArmTargetBatch, EndEffectorTargetMapper
from ...sources.common.target_publisher import SequenceAllocator, TargetPublisher
from ...zenoh_util import (
    load_node_config,
    load_tianji_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    require_single_router,
)
from .source import ControllerSample, XRoboControllerOnlySource

_LOG = logging.getLogger("pico_controller")

DEFAULT_PARAMETERS = {
    "rate": 90.0,
    "stale_timeout": 0.5,
    "require_reliable_timestamp": True,
    "allow_unstamped_input": False,
    "min_cutoff": 1.0,
    "beta": 0.7,
    "translation_gain": [0.75, 0.75, 0.75],
    "rotation_gain": 0.85,
    "workspace_relative_radii_m": [0.32, 0.28, 0.28],
    "workspace_soft_zone_ratio": 0.80,
    "maximum_linear_speed_m_s": 0.18,
    "maximum_angular_speed_rad_s": 0.80,
    "maximum_linear_acceleration_m_s2": 1.20,
    "maximum_angular_acceleration_rad_s2": 4.0,
    "left_default_elbow_direction": [0.45638698, -0.74604902, -0.48489358],
    "right_default_elbow_direction": [0.45638698, 0.74604902, -0.48489358],
}


class PicoControllerSource:
    """A rising A edge requests start; only coordinator teleop authorizes motion."""

    def __init__(
        self,
        session: Any,
        params: dict[str, Any] | None = None,
        *,
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str | None = None,
        source: XRoboControllerOnlySource | None = None,
        session_client: SessionClient | None = None,
        target_publisher: TargetPublisher | None = None,
    ) -> None:
        params = {**DEFAULT_PARAMETERS, **(params or {})}
        rate = float(params["rate"])
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        self._rate = rate
        self._require_reliable_timestamp = bool(params["require_reliable_timestamp"])
        config = load_tianji_config()
        self._mapper = EndEffectorTargetMapper(
            config,
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            conditioning_settings=TargetConditioningSettings(
                rate_hz=rate,
                translation_gain=params["translation_gain"],
                rotation_gain=float(params["rotation_gain"]),
                workspace_relative_radii_m=params["workspace_relative_radii_m"],
                workspace_soft_zone_ratio=float(params["workspace_soft_zone_ratio"]),
                maximum_linear_speed_m_s=float(params["maximum_linear_speed_m_s"]),
                maximum_angular_speed_rad_s=float(params["maximum_angular_speed_rad_s"]),
                maximum_linear_acceleration_m_s2=float(params["maximum_linear_acceleration_m_s2"]),
                maximum_angular_acceleration_rad_s2=float(params["maximum_angular_acceleration_rad_s2"]),
            ),
            default_zsp_directions={
                side: params[f"{side}_default_elbow_direction"] for side in ("left", "right")
            },
        )
        self._source = source or XRoboControllerOnlySource()
        self._source.open()
        self._freshness = FreshnessGate(
            timeout_seconds=float(params["stale_timeout"]),
            allow_unstamped=bool(params["allow_unstamped_input"]),
        )
        allocator = SequenceAllocator()
        self._session_client = session_client or SessionClient(
            session,
            source="pico_controller",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            expected_coordinator_instance_id=coordinator_instance_id,
            allocator=allocator,
        )
        self._publisher = target_publisher or TargetPublisher(
            session,
            source="pico_controller",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            allocator=allocator,
        )
        self._phase = "armed"
        self._last_a = False
        self._edge_previous = False
        self._last_sample: ControllerSample | None = None
        self._last_source_state = "unavailable"
        self._last_source_timestamp_ns: int | None = None
        self._last_error: str | None = None
        self._last_targets: ArmTargetBatch | None = None
        self._return_deadline = 0.0
        self._return_timed_out = False
        self._closed = False
    @property
    def phase(self) -> str:
        return self._phase

    @property
    def session_client(self) -> SessionClient:
        return self._session_client

    @property
    def target_publisher(self) -> TargetPublisher:
        return self._publisher

    def _request_return(self, reason: str) -> None:
        if self._phase in ("returning", "fault"):
            return
        try:
            self._session_client.request_return(reason, timeout_s=1.0)
        except (RuntimeError, ValueError) as exc:
            self._last_error = str(exc)
        self._phase = "returning"
        self._return_deadline = time.monotonic() + 5.0
        self._return_timed_out = False
        self._last_targets = None

    def _tick(self, now: float | None = None) -> None:
        if self._closed:
            return
        now = time.monotonic() if now is None else float(now)
        self._session_client.poll()
        try:
            sample = self._source.read()
            self._last_error = None
        except Exception as exc:
            sample = None
            self._last_error = str(exc)
        signal_live = False
        if sample is not None:
            self._last_sample = sample
            self._last_a = bool(sample.right_a_pressed)
            self._last_source_timestamp_ns = int(sample.source_timestamp_ns)
            freshness = self._freshness.observe(
                source_timestamp_ns=sample.source_timestamp_ns,
                frame_signature=sample.frame.signature(),
                now=now,
            )
            self._last_source_state = freshness.state
            signal_live = freshness.allow_publish
            if self._require_reliable_timestamp and not freshness.reliable_clock:
                signal_live = False
            self._publisher.publish_raw_pico_controller(
                left_pose=sample.frame.left_pose,
                right_pose=sample.frame.right_pose,
                right_a_pressed=sample.right_a_pressed,
                source_timestamp_ns=sample.source_timestamp_ns,
            )
        else:
            self._last_source_state = "unavailable"
            self._last_a = False
        pressed = bool(sample.right_a_pressed) if sample is not None else False

        if self._phase == "armed":
            rising = pressed and not self._edge_previous
            self._edge_previous = pressed
            if rising and signal_live and self._session_client.startup_ready:
                self._session_client.request_start("right_controller_a")
                self._phase = "start_pending"
            self._publish_status()
            return

        self._edge_previous = pressed
        if self._phase == "start_pending":
            if not signal_live:
                self._request_return("pico_controller_stale_before_start")
            elif self._session_client.start_authorized:
                if self._last_sample is None:
                    self._request_return("pico_controller_sample_missing")
                else:
                    initialized = self._mapper.initialize(self._last_sample.frame)
                    if initialized != {"pico_left_wrist", "pico_right_wrist"}:
                        self._last_error = "controller reference initialization incomplete"
                        self._request_return(self._last_error)
                    else:
                        self._phase = "teleop"
            elif self._session_client.pending_intent_sequence is None:
                self._phase = "armed"
            self._publish_status()
            return

        if self._phase == "teleop":
            if not signal_live:
                self._request_return("pico_controller_stale")
            elif sample is not None:
                try:
                    targets = self._mapper.map_relative_controller_frame(sample.frame)
                    self._last_targets = targets
                    self._publish_targets(targets, sample.source_timestamp_ns)
                except Exception as exc:
                    self._last_error = str(exc)
                    self._request_return("pico_controller_mapping_error")
            self._publish_status()
            return

        if self._phase == "returning":
            if self._session_client.return_completion_fresh:
                self._phase = "armed"
                self._return_deadline = 0.0
                self._return_timed_out = False
            elif now >= self._return_deadline:
                self._last_error = "coordinator return completion timeout"
                self._return_timed_out = True
                self._phase = "fault"
            self._publish_status()

    def _publish_targets(self, targets: ArmTargetBatch, source_timestamp_ns: int | None) -> None:
        self._publisher.publish_arm_target(
            side="left",
            position_m=targets.left_pose[:3],
            orientation_xyzw=targets.left_pose[3:],
            elbow_reference_direction=targets.left_default_elbow_direction,
            source_timestamp_ns=source_timestamp_ns,
        )
        self._publisher.publish_arm_target(
            side="right",
            position_m=targets.right_pose[:3],
            orientation_xyzw=targets.right_pose[3:],
            elbow_reference_direction=targets.right_default_elbow_direction,
            source_timestamp_ns=source_timestamp_ns,
        )

    def _publish_status(self) -> None:
        self._publisher.publish_source_status(
            component_id="pico_controller",
            phase=self._phase,
            ready=self._session_client.startup_ready and self._last_error is None,
            healthy=self._last_error is None and self._phase != "fault",
            capabilities=["simulation", "real"],
            error=self._last_error,
            diagnostics={
                "source_state": self._last_source_state,
                "source_timestamp_ns": self._last_source_timestamp_ns,
                "right_a_pressed": self._last_a,
                "return_timed_out": self._return_timed_out,
                "return_intent_baseline": self._session_client.return_intent_baseline,
            },
        )

    def run(self) -> int:
        self.start()
        interval = 1.0 / self._rate
        next_tick = time.monotonic()
        while not self._closed:
            now = time.monotonic()
            if now >= next_tick:
                self._tick(now)
                next_tick += interval
            time.sleep(max(0.001, next_tick - time.monotonic()))
        return 0

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._source.close()
        finally:
            self._publisher.close()
            self._session_client.close()




def main(argv: list[str] | None = None) -> int:
    args = parse_cli_args(argv)
    overrides = dict(parse_param_override(item) for item in args.param)
    params = load_node_config(args.config, "pico_controller", DEFAULT_PARAMETERS, overrides)
    instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    router_zid = os.environ.get("TIANJI_ROUTER_ZID")
    coordinator_id = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    endpoint = os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447")
    if not instance_id or not router_zid or not coordinator_id:
        raise RuntimeError(
            "TIANJI_COMPONENT_INSTANCE_ID, TIANJI_ROUTER_ZID and "
            "TIANJI_COORDINATOR_INSTANCE_ID are required"
        )
    session = open_session(endpoint)
    require_single_router(session, router_zid)
    node = PicoControllerSource(
        session,
        params,
        publisher_instance_id=instance_id,
        router_zid=router_zid,
        coordinator_instance_id=coordinator_id,
    )
    try:
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        node.close()


__all__ = ["DEFAULT_PARAMETERS", "PicoControllerSource", "main"]
