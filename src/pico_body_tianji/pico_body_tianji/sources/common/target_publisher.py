"""Typed target/raw publishers shared by every source role."""
from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any, Callable

import numpy as np

from ...protocol import topics
from ...protocol.messages import (
    ArmTargetCommand,
    ComponentStatus,
    Frame0HandSkeleton,
    HandTargetCommand,
    ProtocolEnvelope,
    RawH5ReplaySample,
    RawMocapLiveSample,
    RawPicoControllerSample,
)
from ...zenoh_util import ZenohPub


class TargetPublisher:
    """Compose typed arm/hand/raw publication with one local wire clock.

    ``time.monotonic_ns`` is used for every internal ``timestamp_ns``. Device,
    H5, and acquisition times are passed only as ``source_timestamp_ns``.
    """

    def __init__(
        self,
        session: Any,
        *,
        source: str,
        publisher_instance_id: str,
        router_zid: str,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not source or not publisher_instance_id or not router_zid:
            raise ValueError("source, publisher_instance_id and router_zid are required")
        self.session = session
        self.source = source
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self._clock = clock
        self._sequence = 0
        self._publishers: dict[str, ZenohPub] = {}

    @property
    def sequence(self) -> int:
        return self._sequence

    def _publisher(self, key: str) -> ZenohPub:
        publisher = self._publishers.get(key)
        if publisher is None:
            publisher = ZenohPub(self.session, key)
            self._publishers[key] = publisher
        return publisher

    def _envelope(self) -> ProtocolEnvelope:
        self._sequence += 1
        return ProtocolEnvelope(
            schema_version=1,
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
            sequence=self._sequence,
            timestamp_ns=int(self._clock()),
        )

    @staticmethod
    def relative_hand_keypoints(keypoints_world_m: Any, wrist_world_m: Any | None = None) -> list[list[float]]:
        """Return finite wrist-relative MediaPipe points with exact zero root."""
        points = np.asarray(keypoints_world_m, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("hand keypoints must be a finite (21,3) array")
        if wrist_world_m is None:
            wrist = points[0].copy()
        else:
            wrist = np.asarray(wrist_world_m, dtype=np.float64)
            if wrist.shape != (3,) or not np.isfinite(wrist).all():
                raise ValueError("wrist_world_m must be a finite 3-vector")
        relative = points - wrist
        relative[0] = 0.0
        return relative.tolist()

    def publish_arm_target(
        self,
        *,
        side: str,
        position_m: Any,
        orientation_xyzw: Any,
        elbow_reference_direction: Any,
        source_timestamp_ns: int | None = None,
        source: str | None = None,
        frame_id: str | None = None,
    ) -> ArmTargetCommand:
        envelope = self._envelope()
        command = ArmTargetCommand(
            envelope=envelope,
            source_timestamp_ns=source_timestamp_ns,
            source=self.source if source is None else source,
            side=side,
            frame_id=frame_id or ("Base_L" if side == "left" else "Base_R"),
            position_m=np.asarray(position_m, dtype=np.float64).tolist(),
            orientation_xyzw=np.asarray(orientation_xyzw, dtype=np.float64).tolist(),
            elbow_reference_direction=np.asarray(elbow_reference_direction, dtype=np.float64).tolist(),
        )
        self._publisher(topics.arm_target(side)).put_json(command.to_dict())
        return command

    def publish_hand_target(
        self,
        *,
        side: str,
        keypoints_m: Any,
        source_timestamp_ns: int | None = None,
        source: str | None = None,
    ) -> HandTargetCommand:
        envelope = self._envelope()
        command = HandTargetCommand(
            schema_version=1,
            sequence=envelope.sequence,
            timestamp_ns=envelope.timestamp_ns,
            source_timestamp_ns=source_timestamp_ns,
            source=self.source if source is None else source,
            side=side,
            frame_id="wrist_relative_mediapipe",
            keypoints_m=np.asarray(keypoints_m, dtype=np.float64).tolist(),
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
        )
        self._publisher(topics.hand_target(side)).put_json(command.to_dict())
        return command

    def publish_source_status(
        self,
        *,
        component_id: str | None = None,
        phase: str,
        ready: bool,
        healthy: bool = True,
        capabilities: list[str] | tuple[str, ...] = ("simulation",),
        error: str | None = None,
        diagnostics: Mapping[str, Any] | None = None,
    ) -> ComponentStatus:
        envelope = self._envelope()
        status = ComponentStatus(
            schema_version=1,
            sequence=envelope.sequence,
            timestamp_ns=envelope.timestamp_ns,
            component_role="source",
            component_id=component_id or self.source,
            phase=phase,
            ready=bool(ready),
            healthy=bool(healthy),
            capabilities=list(capabilities),
            error=error,
            diagnostics=dict(diagnostics or {}),
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
        )
        self._publisher(topics.SOURCE_STATUS).put_json(status.to_dict())
        return status

    def publish_hand_joint_command(
        self, *, side: str, names: Any, position_rad: Any, producer: str | None = None
    ) -> Any:
        from ...protocol.messages import HandJointCommand

        envelope = self._envelope()
        command = HandJointCommand(
            schema_version=1,
            sequence=envelope.sequence,
            timestamp_ns=envelope.timestamp_ns,
            producer=self.source if producer is None else producer,
            side=side,
            names=list(names),
            position_rad=np.asarray(position_rad, dtype=np.float64).tolist(),
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
        )
        self._publisher(topics.hand_command(side)).put_json(command.to_dict())
        return command
    def publish_frame0_skeleton_data(
        self,
        *,
        side: str,
        keypoints_world_m: Any,
        manus_wrist_pose: Any,
        robot_wrist_home_pose: Any,
        target_wrist_pose: Any,
        tcp_to_wrist_pose: Any,
        edges: Any,
    ) -> Frame0HandSkeleton:
        envelope = self._envelope()
        skeleton = Frame0HandSkeleton(
            schema_version=1,
            timestamp_ns=envelope.timestamp_ns,
            side=side,
            frame_id="motive_world",
            keypoints_world_m=np.asarray(keypoints_world_m, dtype=np.float64).tolist(),
            edges=np.asarray(edges, dtype=np.int64).tolist(),
            manus_wrist_pose=np.asarray(manus_wrist_pose, dtype=np.float64).tolist(),
            robot_wrist_home_pose=np.asarray(robot_wrist_home_pose, dtype=np.float64).tolist(),
            target_wrist_pose=np.asarray(target_wrist_pose, dtype=np.float64).tolist(),
            tcp_to_wrist_pose=np.asarray(tcp_to_wrist_pose, dtype=np.float64).tolist(),
            sequence=envelope.sequence,
            publisher_instance_id=self.publisher_instance_id,
            router_zid=self.router_zid,
        )
        self._publisher(topics.FRAME0_HAND_SKELETON).put_json(skeleton.to_dict())
        return skeleton


    def publish_raw_pico_controller(
        self,
        *,
        left_pose: Any,
        right_pose: Any,
        right_a_pressed: bool,
        source_timestamp_ns: int | None,
    ) -> RawPicoControllerSample:
        envelope = self._envelope()
        sample = RawPicoControllerSample(
            envelope=envelope,
            source_timestamp_ns=source_timestamp_ns,
            left_pose=np.asarray(left_pose, dtype=np.float64).tolist(),
            right_pose=np.asarray(right_pose, dtype=np.float64).tolist(),
            right_a_pressed=bool(right_a_pressed),
        )
        self._publisher(topics.RAW_PICO_CONTROLLER).put_json(sample.to_dict())
        return sample

    def publish_raw_mocap_live(self, payload: Mapping[str, Any]) -> RawMocapLiveSample:
        """Re-envelope one acquisition ``mocap/aligned/hands`` payload."""
        envelope = self._envelope()
        sample = RawMocapLiveSample(
            envelope=envelope,
            source_timestamp_ns=_optional_int(payload.get("time_ns")),
            stream_instance_id=str(payload["stream_instance_id"]),
            stream_sequence=int(payload["stream_sequence"]),
            frame_index=int(payload["frame_index"]),
            hands=_copy_hands(payload["hands"], h5=False),
        )
        self._publisher(topics.RAW_MOCAP_LIVE).put_json(sample.to_dict())
        return sample

    def publish_raw_h5_replay(
        self,
        *,
        source_timestamp_ns: int | None,
        hands: Mapping[str, Any],
    ) -> RawH5ReplaySample:
        envelope = self._envelope()
        sample = RawH5ReplaySample(
            envelope=envelope,
            source_timestamp_ns=source_timestamp_ns,
            hands=_copy_hands(hands, h5=True),
        )
        self._publisher(topics.RAW_H5_REPLAY).put_json(sample.to_dict())
        return sample

    def publish_frame0_skeleton(self, skeleton: Frame0HandSkeleton) -> Frame0HandSkeleton:
        if not isinstance(skeleton, Frame0HandSkeleton):
            raise TypeError("skeleton must be Frame0HandSkeleton")
        self._publisher(topics.FRAME0_HAND_SKELETON).put_json(skeleton.to_dict())
        return skeleton

    def close(self) -> None:
        for publisher in self._publishers.values():
            publisher.close()
        self._publishers.clear()


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _copy_hands(hands: Mapping[str, Any], *, h5: bool) -> dict[str, dict[str, Any]]:
    if set(hands) != {"left", "right"}:
        raise ValueError("hands must contain exactly left and right")
    result: dict[str, dict[str, Any]] = {}
    for side in ("left", "right"):
        item = dict(hands[side])
        if bool(item.get("valid", False)):
            item["wrist_pose"] = list(item["wrist_pose"])
            item["keypoints_world_m"] = np.asarray(item["keypoints_world_m"], dtype=np.float64).tolist()
        else:
            item["wrist_pose"] = None
            item["keypoints_world_m"] = None
        if h5:
            item["wuji2_joints_rad"] = (
                None if item.get("wuji2_joints_rad") is None
                else np.asarray(item["wuji2_joints_rad"], dtype=np.float64).tolist()
            )
        result[side] = item
    return result


__all__ = ["TargetPublisher"]
