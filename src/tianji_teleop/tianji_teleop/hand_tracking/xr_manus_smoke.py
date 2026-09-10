"""Deterministic XR + Manus observation/target integration smoke.

The smoke is deliberately below the robot-command boundary.  It generates
typed XR frames and Manus callbacks, runs the same canonical observation
publishers and target bridge used by ``vr_manus_xr_sim``, and reports the
resulting arm/hand targets.  It does not load a native XR SDK, start rawviz,
open Zenoh, solve IK, or publish a robot command.
"""
from __future__ import annotations

from collections import Counter
from typing import Any

import numpy as np

from ..config_loader import component_path
from .operator_input import OperatorEdgeFilter
from .reference_manus_process import ManusCallback
from .runtime import ObservationRuntime
from .target_node import _load_config as load_target_config
from .target_node import create_bridge_from_config, select_xr_arm_input
from .xr_input import (
    XR_ARM_INPUTS,
    XrBindingConfig,
    XrControllerState,
    XrFrame,
    XrTrackerState,
)
from .xr_manus_runtime import manus_callback_observations
from .xr_manus_observation import _load_config as load_observation_config
from .xr_operator import (
    XrControllerOperatorConfig,
    XrControllerOperatorPublisher,
    decode_xr_operator_observation,
)


_ROUTER = "offline-router"
_OBSERVATION_PUBLISHER = "xr-manus-observation"
_RECEIVER = "xr-receiver"
_PERIOD_NS = 10_000_000


def _pose(x: float, y: float, z: float) -> list[float]:
    return [float(x), float(y), float(z), 0.0, 0.0, 0.0, 1.0]


def _binding(arm_input: str, source_config: dict[str, Any]) -> XrBindingConfig:
    xr = source_config["xr"]
    return XrBindingConfig(
        arm_input=arm_input,
        tracker_serials=xr["tracker_serials"],
        controller_sides=xr["controller_sides"],
        elbow_tracker_serials=xr["elbow_tracker_serials"],
    )


def _frame(index: int, *, arm_input: str, binding: XrBindingConfig) -> XrFrame:
    """Create a non-degenerate frame with both wrists and elbows moving."""
    displacement = 0.004 * np.sin(index * 0.11)
    controller_displacement = 0.006 * np.sin(index * 0.11)
    grip = 0.0 if index == 0 else 1.0
    controllers = {
        "left": XrControllerState(
            side="left", pose=_pose(controller_displacement, 0.35, 0.25),
            valid=True, available=True, trigger=0.0, grip=grip,
            axis=[0.15 * np.sin(index * 0.07), 0.0],
        ),
        "right": XrControllerState(
            side="right", pose=_pose(controller_displacement, -0.35, 0.25),
            valid=True, available=True, trigger=0.0, grip=grip,
            axis=[0.15 * np.sin(index * 0.07), 0.0],
        ),
    }
    trackers = (
        XrTrackerState(
            serial_number="190058", side="left",
            pose=_pose(displacement, 0.20, 0.30), valid=True,
        ),
        XrTrackerState(
            serial_number="190600", side="right",
            pose=_pose(displacement, -0.20, 0.30), valid=True,
        ),
        XrTrackerState(
            serial_number="190046", side=None,
            pose=_pose(-0.10 + displacement, 0.30, 0.42), valid=True,
        ),
        XrTrackerState(
            serial_number="190023", side=None,
            pose=_pose(-0.10 + displacement, -0.30, 0.42), valid=True,
        ),
    )
    frame = XrFrame(
        sequence=index,
        source_timestamp_ns=1_000_000_000 + index * _PERIOD_NS,
        received_timestamp_ns=1_000_000_000 + index * _PERIOD_NS,
        hmd_pose=_pose(0.0, 0.0, 1.6),
        controllers=controllers,
        trackers=trackers,
        receiver_instance_id=_RECEIVER,
        connection_generation=1,
    )
    # Keep the generated object tied to the selected binding.  This catches a
    # future smoke change which accidentally stops emitting the selected arm
    # source while still allowing either controller or tracker mode.
    if binding.pose_for_arm(frame, "left") is None:
        raise RuntimeError(f"synthetic {arm_input} binding produced no left pose")
    return frame


def _manus_callback(index: int, timestamp_ns: int) -> ManusCallback:
    points = np.zeros((42, 3), dtype=np.float64)
    motion = 0.004 * np.sin(index * 0.13)
    for block in range(2):
        for joint in range(21):
            points[block * 21 + joint] = [
                0.012 * (joint % 5) + motion,
                0.010 * (joint // 5) + 0.002 * block,
                0.008 * (joint % 3),
            ]
        points[block * 21] = 0.0
    return ManusCallback(
        receiver_instance_id="manus-receiver",
        sequence=index + 1,
        received_timestamp_ns=timestamp_ns,
        points=tuple(points.reshape(-1).tolist()),
        source_sequences={"right": index + 1, "left": index + 1},
        source_timestamps_ns={"right": timestamp_ns, "left": timestamp_ns},
    )


def _operator_filter(config: XrControllerOperatorConfig) -> OperatorEdgeFilter:
    return OperatorEdgeFilter(
        source="xr-operator",
        side=config.start.side,
        action="start_request",
        epoch=1,
        freshness_ns=config.freshness_ns,
        stable_ns=config.stable_ns,
    )


def run_offline_smoke(*, arm_input: str = "xr_tracker", frame_count: int = 100) -> dict[str, Any]:
    """Run the receive-only XR/Manus target smoke and return a JSON report."""
    if arm_input not in XR_ARM_INPUTS:
        raise ValueError("arm_input must be xr_tracker or xr_controller")
    if isinstance(frame_count, bool) or not isinstance(frame_count, int):
        raise ValueError("frame_count must be an integer")

    source_config_path = component_path("sources/xr_manus_observation.yaml")
    source_config = load_observation_config(source_config_path)
    operator_config = source_config["xr"]["operator_config"]
    minimum_frames = int(np.ceil(operator_config.stable_ns / _PERIOD_NS)) + 3
    if frame_count < minimum_frames:
        raise ValueError(f"frame_count must be at least {minimum_frames}")

    target_config = load_target_config(
        component_path("sources/hand_tracking_target_xr_manus.yaml")
    )
    select_xr_arm_input(target_config, arm_input)
    binding = _binding(arm_input, source_config)
    clock_value = [1_000_000_000]
    bridge = create_bridge_from_config(
        target_config,
        router_zid=_ROUTER,
        observation_publisher_instance_id=_OBSERVATION_PUBLISHER,
        clock=lambda: clock_value[0],
    )
    published_topics: Counter[str] = Counter()
    runtime = ObservationRuntime(
        publish=lambda topic, payload: published_topics.update((topic,)),
        publisher_instance_id=_OBSERVATION_PUBLISHER,
        router_zid=_ROUTER,
        clock=lambda: clock_value[0],
    )
    operator_publisher = XrControllerOperatorPublisher.from_config(
        source_instance_id="xr-operator", epoch=1, config=operator_config,
    )
    start_filter = _operator_filter(operator_config)
    start_events = 0
    start_sequence = None
    started = False
    prestart_targets = 0
    arm_targets = []
    hand_targets = []
    controller_binding_verified = False

    try:
        for index in range(frame_count):
            timestamp_ns = 1_000_000_000 + index * _PERIOD_NS
            clock_value[0] = timestamp_ns
            frame = _frame(index, arm_input=arm_input, binding=binding)
            arm_wires = runtime.ingest_xr(
                frame,
                binding,
                tracked_frame=("controller" if arm_input == "xr_controller" else "wrist_tracker"),
            )
            callback = _manus_callback(index, timestamp_ns)
            runtime.publish_manus_callback(callback)
            hand_wires = {
                side: runtime.publish_hand_observation(observation)
                for side, observation in manus_callback_observations(callback).items()
            }
            for side in ("left", "right"):
                bridge.ingest_arm_observation(arm_wires[side])
                bridge.ingest_hand_observation(hand_wires[side], now_ns=timestamp_ns)

            decoded_start = None
            for payload in operator_publisher.payloads(frame, router_zid=_ROUTER):
                observation = decode_xr_operator_observation(
                    payload,
                    expected_router_zid=_ROUTER,
                    expected_publisher_instance_id="xr-operator",
                )
                if observation.action == "start_request":
                    decoded_start = start_filter.update(observation, now_ns=timestamp_ns)
            if decoded_start is not None:
                start_events += 1
                start_sequence = decoded_start.sequence
                if started:
                    raise RuntimeError("duplicate synthetic start event")
                bridge.start(now_ns=timestamp_ns)
                started = True

            targets = bridge.tick(now_ns=timestamp_ns)
            if not started:
                if targets.arm or targets.hand:
                    raise RuntimeError("target emitted before explicit start")
                prestart_targets += len(targets.arm) + len(targets.hand)
                continue
            if set(item.side for item in targets.arm) != {"left", "right"}:
                raise RuntimeError("XR smoke did not emit both arm targets")
            if set(item.side for item in targets.hand) != {"left", "right"}:
                raise RuntimeError("XR smoke did not emit both hand targets")
            arm_targets.extend(targets.arm)
            hand_targets.extend(targets.hand)
            if arm_input == "xr_controller":
                controller_binding_verified = all(
                    item.tracked_frame == "controller" for item in arm_wires.values()
                )

        if start_events != 1 or not started:
            raise RuntimeError(f"expected one explicit start event, got {start_events}")
        if not arm_targets or not hand_targets:
            raise RuntimeError("synthetic start left no target frames")
        arm_positions_by_side = {
            side: np.asarray([item.pose[:3] for item in arm_targets if item.side == side], dtype=np.float64)
            for side in ("left", "right")
        }
        hand_points_by_side = {
            side: np.asarray([item.keypoints_m[5] for item in hand_targets if item.side == side], dtype=np.float64)
            for side in ("left", "right")
        }
        return {
            "kind": "xr_manus_offline_input_smoke",
            "passed": True,
            "simulation_only": True,
            "robot_commands_enabled": False,
            "hardware_acceptance_complete": False,
            "arm_input": arm_input,
            "frames": frame_count,
            "start_events": start_events,
            "start_frame_sequence": start_sequence,
            "prestart_targets": prestart_targets,
            "arm_targets": len(arm_targets),
            "hand_targets": len(hand_targets),
            "target_frames": len(arm_targets) // 2,
            "arm_motion_m": float(max(
                np.max(np.ptp(values, axis=0)) for values in arm_positions_by_side.values()
            )),
            "hand_motion_m": float(max(
                np.max(np.ptp(values, axis=0)) for values in hand_points_by_side.values()
            )),
            "controller_arm_binding_verified": controller_binding_verified,
            "published": dict(published_topics),
            "mapping_backend": sorted({item.mapping_backend for item in arm_targets}),
            "hand_adapter_versions": sorted({item.adapter_version for item in hand_targets}),
        }
    finally:
        bridge.reset()


__all__ = ["run_offline_smoke"]
