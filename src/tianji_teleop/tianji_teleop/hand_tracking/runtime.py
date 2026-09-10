"""Receive-only runtime for PICO and Manus hand-tracking observations.

The runtime owns only source decoding, canonical observation publication, and
optional recording.  It deliberately has no imports from coordinators,
producers, IK, or executors, so starting an observation profile cannot create a
robot command path as a side effect.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import logging
import socket
import subprocess
import threading
import time
from typing import Any

import numpy as np

from ..protocol import topics
from ..protocol.messages import (
    ArmInputObservation as ArmInputObservationWire,
    HandSkeletonObservation,
    ProtocolEnvelope,
    strict_loads,
)
from .legacy_pico import legacy_palm_observation, parse_legacy_pico_packet
from .manus import manus_to_mediapipe, parse_manus_payload
from .models import (
    ArmInputObservation,
    HandObservation,
    LegacyPicoPalmFrame,
    ManusRawFrame,
    PICO_HEAD_CURRENT_FRAME,
    PICO_HEAD_MAPPING_VERSION,
    PICO_MAPPING_VERSION,
    PicoRawFrame,
    PicoRawHand,
)
from .reference_manus_process import ManusCallback
from .xr_input import XrBindingConfig, XrFrame
from .xr_manus_runtime import encode_manus_callback, xr_frame_arm_observations
from .pico import HEADER, MAGIC, MESSAGE_TYPE, PAYLOAD_BYTES, pico_to_mediapipe, parse_pico_packet, tracking_pose_to_current_head


LOG = logging.getLogger(__name__)
MAX_PACKET_BYTES = HEADER.size + PAYLOAD_BYTES


def _valid_pose(pose: np.ndarray) -> bool:
    return bool(np.isfinite(pose).all() and np.linalg.norm(pose[3:]) >= 1.0e-12)


def _canonical_hand_observation(frame: PicoRawFrame, side: str, hand: PicoRawHand) -> HandObservation:
    positions = np.asarray([joint.pose[:3] for joint in hand.joints], dtype=np.float64)
    validity = np.asarray([joint.valid for joint in hand.joints], dtype=np.bool_)
    points, point_valid = pico_to_mediapipe(positions, validity)
    if not hand.valid:
        point_valid[:] = False
        points[:] = 0.0
    elif bool(point_valid[0]):
        points[point_valid] -= points[0]
    else:
        points[:] = 0.0
    wrist_pose = hand.wrist_pose.copy() if hand.wrist_valid and _valid_pose(hand.wrist_pose) else None
    valid = bool(hand.valid and wrist_pose is not None and point_valid.all())
    return HandObservation(
        source="pico",
        side=side,
        source_instance_id=frame.receiver_instance_id,
        source_sequence=None,
        source_timestamp_ns=frame.source_timestamp_ns,
        received_timestamp_ns=frame.received_timestamp_ns,
        receiver_instance_id=frame.receiver_instance_id,
        receiver_frame_sequence=frame.receiver_frame_sequence,
        coordinate_frame="pico_tracking_initial_wrist_relative",
        mapping_version=PICO_MAPPING_VERSION,
        keypoints_m=points,
        joint_valid=point_valid,
        valid=valid,
        wrist_pose=wrist_pose,
        frame_association_id=frame.association_id,
    )


def _pico_arm_observation(frame: PicoRawFrame, side: str, hand: PicoRawHand) -> ArmInputObservation:
    pose: np.ndarray | None = None
    if frame.head_valid and hand.wrist_valid and _valid_pose(frame.head_pose) and _valid_pose(hand.wrist_pose):
        pose = tracking_pose_to_current_head(frame.head_pose, hand.wrist_pose)
    return ArmInputObservation(
        source="pico",
        side=side,
        tracked_frame="wrist",
        reference_frame=PICO_HEAD_CURRENT_FRAME,
        pose=pose,
        valid=pose is not None,
        source_timestamp_ns=frame.source_timestamp_ns,
        received_timestamp_ns=frame.received_timestamp_ns,
        receiver_instance_id=frame.receiver_instance_id,
        receiver_frame_sequence=frame.receiver_frame_sequence,
        mapping_version=PICO_HEAD_MAPPING_VERSION,
        frame_association_id=frame.association_id,
        source_sequence=None,
        source_instance_id=frame.receiver_instance_id,
    )


def pico_frame_observations(frame: PicoRawFrame) -> dict[str, tuple[HandObservation, ArmInputObservation]]:
    """Convert one raw PICO frame into one pair per hand side."""
    if not isinstance(frame, PicoRawFrame):
        raise TypeError("frame must be PicoRawFrame")
    return {
        side: (
            _canonical_hand_observation(frame, side, frame.hands[side]),
            _pico_arm_observation(frame, side, frame.hands[side]),
        )
        for side in ("left", "right")
    }


class PicoPacketStream:
    """Incrementally frame PICO TCP bytes without assuming recv boundaries."""

    def __init__(self, *, receiver_instance_id: str, connection_generation: int) -> None:
        if not receiver_instance_id:
            raise ValueError("receiver_instance_id is required")
        if isinstance(connection_generation, bool) or connection_generation < 0:
            raise ValueError("connection_generation must be non-negative")
        self.receiver_instance_id = receiver_instance_id
        self.connection_generation = int(connection_generation)
        self._buffer = bytearray()
        self._next_frame_sequence = 0

    def reset(self, *, connection_generation: int | None = None) -> None:
        self._buffer.clear()
        self._next_frame_sequence = 0
        if connection_generation is not None:
            if isinstance(connection_generation, bool) or connection_generation < 0:
                raise ValueError("connection_generation must be non-negative")
            self.connection_generation = int(connection_generation)

    def feed(self, data: bytes, *, received_timestamp_ns: int | None = None) -> list[PicoRawFrame]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("PICO TCP data must be bytes-like")
        self._buffer.extend(bytes(data))
        frames: list[PicoRawFrame] = []
        while True:
            if len(self._buffer) < 1:
                break
            if self._buffer[0] != MAGIC:
                try:
                    index = self._buffer.index(MAGIC)
                except ValueError:
                    self._buffer.clear()
                    break
                del self._buffer[:index]
            if len(self._buffer) < HEADER.size:
                break
            _magic, message_type, _timestamp_ms, payload_length = HEADER.unpack_from(self._buffer, 0)
            if message_type != MESSAGE_TYPE or payload_length != PAYLOAD_BYTES:
                raise ValueError(
                    f"unsupported PICO TCP frame header: type={message_type}, payload_length={payload_length}"
                )
            packet_length = HEADER.size + payload_length
            if packet_length > MAX_PACKET_BYTES:
                raise ValueError("PICO packet exceeds configured maximum")
            if len(self._buffer) < packet_length:
                break
            packet = bytes(self._buffer[:packet_length])
            del self._buffer[:packet_length]
            timestamp = time.monotonic_ns() if received_timestamp_ns is None else received_timestamp_ns
            frame = parse_pico_packet(
                packet,
                receiver_instance_id=self.receiver_instance_id,
                connection_generation=self.connection_generation,
                receiver_frame_sequence=self._next_frame_sequence,
                received_timestamp_ns=timestamp,
            )
            self._next_frame_sequence += 1
            frames.append(frame)
        return frames


class ObservationRuntime:
    """Publish and optionally record receive-only canonical observations."""

    def __init__(
        self,
        *,
        publish: Callable[[str, dict[str, Any]], None],
        publisher_instance_id: str,
        router_zid: str,
        session_writer: Any = None,
        clock: Callable[[], int] = time.monotonic_ns,
        run_id: str | None = None,
    ) -> None:
        if not callable(publish):
            raise TypeError("publish must be callable")
        if not publisher_instance_id or not router_zid:
            raise ValueError("publisher_instance_id and router_zid are required")
        if run_id is not None and (not isinstance(run_id, str) or not run_id.strip()):
            raise ValueError("run_id must be a non-empty string when supplied")
        self._publish = publish
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.session_writer = session_writer
        self._clock = clock
        self.run_id = run_id.strip() if run_id is not None else None
        self._sequence = 0
        self._manus_frame_sequence = 0

    def _next_envelope(self, timestamp_ns: int) -> ProtocolEnvelope:
        sequence = self._sequence
        self._sequence += 1
        return ProtocolEnvelope(1, self.publisher_instance_id, self.router_zid, sequence, timestamp_ns)

    def _raw_payload(self, value: Any) -> dict[str, Any]:
        """Attach transport identity to raw frames without changing the model."""
        payload = value.to_dict()
        payload["router_zid"] = self.router_zid
        return payload

    def _publish_hand(self, observation: HandObservation) -> HandSkeletonObservation:
        envelope = self._next_envelope(observation.received_timestamp_ns)
        wire = HandSkeletonObservation(
            schema_version=envelope.schema_version,
            sequence=envelope.sequence,
            timestamp_ns=envelope.timestamp_ns,
            source_timestamp_ns=observation.source_timestamp_ns,
            received_timestamp_ns=observation.received_timestamp_ns,
            source=observation.source,
            side=observation.side,
            source_instance_id=observation.source_instance_id,
            source_sequence=observation.source_sequence,
            receiver_instance_id=observation.receiver_instance_id,
            receiver_frame_sequence=observation.receiver_frame_sequence,
            coordinate_frame=observation.coordinate_frame,
            mapping_version=observation.mapping_version,
            keypoints_m=observation.keypoints_m.tolist(),
            joint_valid=observation.joint_valid.tolist(),
            valid=bool(observation.valid),
            wrist_pose=None if observation.wrist_pose is None else observation.wrist_pose.tolist(),
            frame_association_id=observation.frame_association_id,
            publisher_instance_id=envelope.publisher_instance_id,
            router_zid=envelope.router_zid,
        )
        self._publish(topics.hand_observation(observation.side), wire.to_dict())
        if self.session_writer is not None:
            self.session_writer.append_hand_observation(observation)
        return wire

    def publish_hand_observation(self, observation: HandObservation) -> HandSkeletonObservation:
        """Publish one already validated canonical hand observation."""
        if not isinstance(observation, HandObservation):
            raise TypeError("observation must be HandObservation")
        return self._publish_hand(observation)

    def _publish_arm(self, observation: ArmInputObservation) -> ArmInputObservationWire:
        envelope = self._next_envelope(observation.received_timestamp_ns)
        wire = ArmInputObservationWire(
            schema_version=envelope.schema_version,
            sequence=envelope.sequence,
            timestamp_ns=envelope.timestamp_ns,
            source_timestamp_ns=observation.source_timestamp_ns,
            received_timestamp_ns=observation.received_timestamp_ns,
            source=observation.source,
            side=observation.side,
            tracked_frame=observation.tracked_frame,
            reference_frame=observation.reference_frame,
            source_instance_id=observation.source_instance_id,
            source_sequence=observation.source_sequence,
            receiver_instance_id=observation.receiver_instance_id,
            receiver_frame_sequence=observation.receiver_frame_sequence,
            mapping_version=observation.mapping_version,
            pose=None if observation.pose is None else observation.pose.tolist(),
            valid=bool(observation.valid),
            frame_association_id=observation.frame_association_id,
            publisher_instance_id=envelope.publisher_instance_id,
            router_zid=envelope.router_zid,
            elbow_pose=None if observation.elbow_pose is None else observation.elbow_pose.tolist(),
        )
        self._publish(topics.arm_input_observation(observation.side), wire.to_dict())
        if self.session_writer is not None:
            self.session_writer.append_arm_input_observation(observation)
        return wire

    def publish_arm_observation(self, observation: ArmInputObservation) -> ArmInputObservationWire:
        """Publish one already validated canonical arm observation."""
        if not isinstance(observation, ArmInputObservation):
            raise TypeError("observation must be ArmInputObservation")
        return self._publish_arm(observation)

    def ingest_xr(
        self,
        frame: XrFrame,
        binding: XrBindingConfig,
        *,
        tracked_frame: str | None = None,
        reference_frame: str = "xr_tracking",
    ) -> dict[str, ArmInputObservationWire]:
        """Publish one raw XR snapshot and its two canonical arm inputs."""
        if not isinstance(frame, XrFrame) or not isinstance(binding, XrBindingConfig):
            raise TypeError("frame and binding are required")
        selected_frame = tracked_frame or (
            "controller" if binding.arm_input == "xr_controller" else "wrist_tracker"
        )
        if self.session_writer is not None:
            self.session_writer.append_raw_xr(frame)
        self._publish(
            topics.RAW_XR_INPUT,
            {**frame.to_dict(), "router_zid": self.router_zid,
             "receiver_instance_id": frame.receiver_instance_id},
        )
        observations = xr_frame_arm_observations(
            frame,
            binding=binding,
            tracked_frame=selected_frame,
            reference_frame=reference_frame,
            receiver_instance_id=frame.receiver_instance_id,
            source_instance_id=frame.receiver_instance_id,
        )
        return {side: self._publish_arm(observation) for side, observation in observations.items()}

    def _publish_manus_audit(self, payload: dict[str, Any], *, received_timestamp_ns: int) -> None:
        """Publish one managed rawviz audit row without changing data flow."""
        if self.run_id is None:
            return
        message = {
            "schema_version": 1,
            "router_zid": self.router_zid,
            "run_id": self.run_id,
            "received_timestamp_ns": received_timestamp_ns,
            **payload,
        }
        self._publish(topics.MANUS_INPUT_AUDIT, message)
        if self.session_writer is not None and hasattr(self.session_writer, "append_dual_audit"):
            stored = dict(message)
            stored.pop("router_zid")
            self.session_writer.append_dual_audit(
                message["kind"], stored, received_timestamp_ns=received_timestamp_ns
            )

    def publish_manus_rawviz_line(
        self, text: str, *, line_sequence: int, received_timestamp_ns: int
    ) -> None:
        """Publish one rawviz stdout line before the reference parser consumes it."""
        if self.run_id is None:
            raise ValueError("run_id is required for rawviz audit")
        if type(line_sequence) is not int or not 0 < line_sequence < 2**63:
            raise ValueError("rawviz line sequence must be a positive int64")
        if type(received_timestamp_ns) is not int or not 0 < received_timestamp_ns < 2**63:
            raise ValueError("rawviz receive timestamp must be a positive int64")
        if not isinstance(text, str) or "\n" in text:
            raise ValueError("rawviz line must be a single text line")
        try:
            encoded_length = len(text.encode("utf-8"))
        except UnicodeError as exc:
            raise ValueError("rawviz line must be valid UTF-8") from exc
        if encoded_length > 65536:
            raise ValueError("rawviz line exceeds 65536 bytes")
        self._publish_manus_audit(
            {
                "kind": "manus_rawviz_line",
                "line_sequence": line_sequence,
                "text": text,
                "terminator": "LF",
                "input_stage": "rawviz_stdout_before_parser",
            },
            received_timestamp_ns=received_timestamp_ns,
        )

    def publish_manus_callback(self, callback: ManusCallback) -> None:
        """Publish and optionally record one complete rawviz callback."""
        if not isinstance(callback, ManusCallback):
            raise TypeError("callback must be ManusCallback")
        payload = encode_manus_callback(callback, self.router_zid)
        self._publish(topics.RAW_MANUS_CALLBACK, payload)
        if self.session_writer is not None:
            self.session_writer.append_manus_callback(
                callback.points,
                callback_sequence=callback.sequence,
                received_timestamp_ns=callback.received_timestamp_ns,
                receiver_instance_id=callback.receiver_instance_id,
                source_sequences=callback.source_sequences,
                source_timestamps_ns=callback.source_timestamps_ns,
            )
        self._publish_manus_audit(
            {
                "kind": "manus_callback_metadata",
                "callback_sequence": callback.sequence,
                "receiver_instance_id": callback.receiver_instance_id,
                "source_sequences": dict(callback.source_sequences),
                "source_timestamps_ns": dict(callback.source_timestamps_ns),
            },
            received_timestamp_ns=callback.received_timestamp_ns,
        )

    def ingest_pico(self, frame: PicoRawFrame) -> dict[str, tuple[HandSkeletonObservation, ArmInputObservationWire]]:
        if not isinstance(frame, PicoRawFrame):
            raise TypeError("frame must be PicoRawFrame")
        if self.session_writer is not None:
            self.session_writer.append_raw_pico(frame)
        self._publish(topics.RAW_PICO_HAND_TRACKING, self._raw_payload(frame))
        result: dict[str, tuple[HandSkeletonObservation, ArmInputObservationWire]] = {}
        for side, (hand, arm) in pico_frame_observations(frame).items():
            result[side] = self._publish_hand(hand), self._publish_arm(arm)
        return result

    def ingest_pico_packet(self, packet: bytes, *, receiver_instance_id: str, connection_generation: int, receiver_frame_sequence: int, received_timestamp_ns: int | None = None) -> PicoRawFrame:
        timestamp = self._clock() if received_timestamp_ns is None else received_timestamp_ns
        frame = parse_pico_packet(
            packet,
            receiver_instance_id=receiver_instance_id,
            connection_generation=connection_generation,
            receiver_frame_sequence=receiver_frame_sequence,
            received_timestamp_ns=timestamp,
        )
        self.ingest_pico(frame)
        return frame

    @staticmethod
    def _legacy_arm_observations(
        frame: LegacyPicoPalmFrame,
        *,
        use_corrected_skeleton: bool,
    ) -> dict[str, ArmInputObservation]:
        observations: dict[str, ArmInputObservation] = {}
        for side in ("left", "right"):
            try:
                observations[side] = legacy_palm_observation(
                    frame,
                    side,
                    use_corrected_skeleton=use_corrected_skeleton,
                )
            except ValueError:
                # Keep a corresponding invalid row on the observation stream
                # when corrected skeleton data is unavailable.  The raw frame
                # is still recorded, which makes source dropouts diagnosable.
                observations[side] = ArmInputObservation(
                    source="legacy_pico_palm",
                    side=side,
                    tracked_frame="palm",
                    reference_frame="legacy_pico_tracking",
                    pose=None,
                    valid=False,
                    source_timestamp_ns=frame.source_timestamp_ns,
                    received_timestamp_ns=frame.received_timestamp_ns,
                    receiver_instance_id=frame.receiver_instance_id,
                    receiver_frame_sequence=frame.receiver_frame_sequence,
                    mapping_version=(
                        "legacy_pico_corrected_palm_v1"
                        if use_corrected_skeleton
                        else "legacy_pico_pose_v1"
                    ),
                    frame_association_id=frame.association_id,
                    source_sequence=frame.sequence,
                    source_instance_id=frame.receiver_instance_id,
                )
        return observations

    def ingest_legacy_palm(
        self,
        frame: LegacyPicoPalmFrame,
        *,
        use_corrected_skeleton: bool = True,
    ) -> dict[str, ArmInputObservationWire]:
        """Publish one historical PICO palm frame as arm-input observations."""
        if not isinstance(frame, LegacyPicoPalmFrame):
            raise TypeError("frame must be LegacyPicoPalmFrame")
        if self.session_writer is not None:
            self.session_writer.append_raw_legacy_palm(frame)
        self._publish(topics.RAW_LEGACY_PICO_PALM, self._raw_payload(frame))
        return {
            side: self._publish_arm(observation)
            for side, observation in self._legacy_arm_observations(
                frame,
                use_corrected_skeleton=use_corrected_skeleton,
            ).items()
        }

    def ingest_legacy_packet(
        self,
        packet: bytes,
        *,
        receiver_instance_id: str,
        receiver_frame_sequence: int,
        received_timestamp_ns: int | None = None,
        use_corrected_skeleton: bool = True,
    ) -> LegacyPicoPalmFrame:
        timestamp = self._clock() if received_timestamp_ns is None else received_timestamp_ns
        frame = parse_legacy_pico_packet(
            packet,
            receiver_instance_id=receiver_instance_id,
            receiver_frame_sequence=receiver_frame_sequence,
            received_timestamp_ns=timestamp,
        )
        self.ingest_legacy_palm(frame, use_corrected_skeleton=use_corrected_skeleton)
        return frame

    def ingest_manus_frame(self, frame: Any) -> HandSkeletonObservation:
        if not isinstance(frame, ManusRawFrame):
            raise TypeError("frame must be ManusRawFrame")
        if self.session_writer is not None:
            self.session_writer.append_raw_manus(frame)
        self._publish(topics.RAW_MANUS_HAND_TRACKING, self._raw_payload(frame))
        try:
            observation = manus_to_mediapipe(frame)
        except ValueError:
            observation = HandObservation(
                source="manus",
                side=frame.side,
                source_instance_id=frame.glove_id,
                source_sequence=frame.source_sequence,
                source_timestamp_ns=frame.source_monotonic_ns,
                received_timestamp_ns=frame.received_timestamp_ns,
                receiver_instance_id=frame.receiver_instance_id,
                receiver_frame_sequence=frame.receiver_frame_sequence,
                coordinate_frame="manus_local_vuh_y_flipped_wrist_relative",
                mapping_version="manus25_to_mediapipe21_v1",
                keypoints_m=np.zeros((21, 3), dtype=np.float64),
                joint_valid=np.zeros(21, dtype=np.bool_),
                valid=False,
                wrist_pose=None,
                frame_association_id=frame.association_id,
            )
        return self._publish_hand(observation)

    def ingest_manus_payload(self, payload: Mapping[str, Any] | bytes | bytearray, *, receiver_instance_id: str, received_timestamp_ns: int | None = None) -> HandSkeletonObservation:
        value: Mapping[str, Any]
        if isinstance(payload, (bytes, bytearray)):
            value = strict_loads(bytes(payload))
        elif isinstance(payload, Mapping):
            value = payload
        else:
            raise TypeError("Manus payload must be a mapping or JSON bytes")
        timestamp = self._clock() if received_timestamp_ns is None else received_timestamp_ns
        frame = parse_manus_payload(
            value,
            receiver_instance_id=receiver_instance_id,
            receiver_frame_sequence=self._manus_frame_sequence,
            received_timestamp_ns=timestamp,
        )
        self._manus_frame_sequence += 1
        return self.ingest_manus_frame(frame)


class PicoTcpReceiver:
    """Reconnectable PICO TCP/ADB receiver that feeds ``ObservationRuntime``."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        receiver_instance_id: str,
        reconnect_seconds: float = 1.0,
        adb_path: str = "adb",
        adb_serial: str | None = None,
        adb_device_port: int | None = None,
        auto_adb_forward: bool = True,
        socket_factory: Callable[..., Any] = socket.create_connection,
        on_error: Callable[[Exception], None] | None = None,
    ) -> None:
        if not host or not receiver_instance_id:
            raise ValueError("host and receiver_instance_id are required")
        if isinstance(port, bool) or not 1 <= int(port) <= 65535:
            raise ValueError("port must be in 1..65535")
        if reconnect_seconds <= 0.0 or not np.isfinite(reconnect_seconds):
            raise ValueError("reconnect_seconds must be finite and positive")
        self.host = host
        self.port = int(port)
        self.receiver_instance_id = receiver_instance_id
        self.reconnect_seconds = float(reconnect_seconds)
        self.adb_path = adb_path
        self.adb_serial = adb_serial
        self.adb_device_port = self.port if adb_device_port is None else int(adb_device_port)
        self.auto_adb_forward = bool(auto_adb_forward)
        self.socket_factory = socket_factory
        self.on_error = on_error
        self.stop_event = threading.Event()
        self._connection_generation = 0
        self._socket: Any = None

    def stop(self) -> None:
        self.stop_event.set()
        current_socket = self._socket
        if current_socket is not None:
            try:
                current_socket.close()
            except Exception:
                pass

    def ensure_adb_forward(self) -> None:
        if not self.auto_adb_forward:
            return
        command = [self.adb_path]
        if self.adb_serial:
            command.extend(("-s", self.adb_serial))
        command.extend(("forward", f"tcp:{self.port}", f"tcp:{self.adb_device_port}"))
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5.0, check=False)
        except FileNotFoundError as exc:
            raise RuntimeError(f"ADB executable not found: {self.adb_path!r}") from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("adb forward timed out") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"adb forward failed (exit {result.returncode})" + (f": {detail}" if detail else ""))

    def run(self, on_frame: Callable[[PicoRawFrame], None]) -> None:
        if not callable(on_frame):
            raise TypeError("on_frame must be callable")
        while not self.stop_event.is_set():
            try:
                self.ensure_adb_forward()
                LOG.info("connecting to PICO at %s:%s", self.host, self.port)
                with self.socket_factory((self.host, self.port), timeout=3.0) as sock:
                    self._socket = sock
                    self._connection_generation += 1
                    stream = PicoPacketStream(
                        receiver_instance_id=self.receiver_instance_id,
                        connection_generation=self._connection_generation,
                    )
                    try:
                        sock.settimeout(1.0)
                    except AttributeError:
                        pass
                    while not self.stop_event.is_set():
                        try:
                            data = sock.recv(64 * 1024)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError("PICO closed the TCP connection")
                        for frame in stream.feed(data):
                            on_frame(frame)
            except (OSError, ConnectionError, RuntimeError, ValueError) as exc:
                if self.on_error is not None and not self.stop_event.is_set():
                    self.on_error(exc)
                if not self.stop_event.is_set():
                    self.stop_event.wait(self.reconnect_seconds)
            finally:
                self._socket = None


class LegacyPicoUdpReceiver:
    """Receive the historical TJVR UDP arm/palm stream without a command path."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        receiver_instance_id: str,
        socket_factory: Callable[..., Any] = socket.socket,
        recv_timeout_seconds: float = 1.0,
        clock: Callable[[], int] = time.monotonic_ns,
        on_error: Callable[[Exception], None] | None = None,
    ) -> None:
        if not host or not receiver_instance_id:
            raise ValueError("host and receiver_instance_id are required")
        if isinstance(port, bool) or not 1 <= int(port) <= 65535:
            raise ValueError("port must be in 1..65535")
        if recv_timeout_seconds <= 0.0 or not np.isfinite(recv_timeout_seconds):
            raise ValueError("recv_timeout_seconds must be finite and positive")
        if not callable(socket_factory) or not callable(clock):
            raise TypeError("socket_factory and clock must be callable")
        self.host = host
        self.port = int(port)
        self.receiver_instance_id = receiver_instance_id
        self.socket_factory = socket_factory
        self.recv_timeout_seconds = float(recv_timeout_seconds)
        self.clock = clock
        self.on_error = on_error
        self.stop_event = threading.Event()
        self._receiver_frame_sequence = 0
        self._socket: Any = None

    def stop(self) -> None:
        self.stop_event.set()
        current_socket = self._socket
        if current_socket is not None:
            try:
                current_socket.close()
            except Exception:
                pass

    def run(self, on_frame: Callable[[LegacyPicoPalmFrame], None]) -> None:
        if not callable(on_frame):
            raise TypeError("on_frame must be callable")
        while not self.stop_event.is_set():
            current_socket: Any = None
            try:
                current_socket = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket = current_socket
                try:
                    current_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                except (AttributeError, OSError):
                    pass
                current_socket.bind((self.host, self.port))
                current_socket.settimeout(self.recv_timeout_seconds)
                while not self.stop_event.is_set():
                    try:
                        packet, _address = current_socket.recvfrom(4096)
                    except socket.timeout:
                        continue
                    if not packet:
                        continue
                    try:
                        frame = parse_legacy_pico_packet(
                            packet,
                            receiver_instance_id=self.receiver_instance_id,
                            receiver_frame_sequence=self._receiver_frame_sequence,
                            received_timestamp_ns=self.clock(),
                        )
                    except ValueError as exc:
                        if self.on_error is not None:
                            self.on_error(exc)
                        continue
                    self._receiver_frame_sequence += 1
                    on_frame(frame)
            except (OSError, ValueError) as exc:
                if self.on_error is not None and not self.stop_event.is_set():
                    self.on_error(exc)
                if not self.stop_event.is_set():
                    self.stop_event.wait(1.0)
            finally:
                self._socket = None
                if current_socket is not None:
                    try:
                        current_socket.close()
                    except Exception:
                        pass


class ZenohObservationPublisher:
    """Small lazy Zenoh adapter used only by the observation entry point."""

    def __init__(self, session: Any) -> None:
        self.session = session
        self._publishers: dict[str, Any] = {}

    def __call__(self, key: str, payload: dict[str, Any]) -> None:
        publisher = self._publishers.get(key)
        if publisher is None:
            publisher = self.session.declare_publisher(key)
            self._publishers[key] = publisher
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        try:
            publisher.put(encoded, encoding="application/json")
        except TypeError:
            publisher.put(encoded)

    def close(self) -> None:
        for publisher in self._publishers.values():
            try:
                publisher.undeclare()
            except Exception:
                try:
                    publisher.close()
                except Exception:
                    pass
        self._publishers.clear()


__all__ = [
    "MAX_PACKET_BYTES",
    "LegacyPicoUdpReceiver",
    "ObservationRuntime",
    "PicoPacketStream",
    "PicoTcpReceiver",
    "ZenohObservationPublisher",
    "pico_frame_observations",
]
