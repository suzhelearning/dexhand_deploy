from __future__ import annotations

import struct
import socket
import unittest

import numpy as np

from tianji_teleop.hand_tracking.pico import HEADER, parse_pico_packet
from tianji_teleop.hand_tracking.legacy_pico import parse_legacy_pico_packet
from tianji_teleop.hand_tracking.manus import MEDIAPIPE_SEMANTIC_ORDER, parse_manus_payload
from tianji_teleop.hand_tracking.runtime import (
    LegacyPicoUdpReceiver,
    ObservationRuntime,
    PicoTcpReceiver,
    PicoPacketStream,
    pico_frame_observations,
)
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import ArmInputObservation as ArmInputWire
from tianji_teleop.protocol.messages import HandSkeletonObservation


def _pose(index: int) -> list[float]:
    return [float(index), float(index) + 0.1, float(index) + 0.2, 0.0, 0.0, 0.0, 1.0]


def _packet() -> bytes:
    payload = bytearray(struct.pack("<BBBB", 1, 0x07, 26, 0))
    payload.extend(struct.pack("<7f", *_pose(100)))
    for side_offset in (0, 1000):
        payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
        payload.extend(struct.pack("<7f", *_pose(200 + side_offset)))
        for joint in range(26):
            payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
            payload.extend(struct.pack("<7f", *_pose(side_offset + joint)))
            payload.extend(struct.pack("<f", 0.01 * (joint + 1)))
    return struct.pack("<BBqI", 0xAB, 0x40, 1234, len(payload)) + payload


def _write_double(packet: bytearray, offset: int, value: float) -> None:
    struct.pack_into("<d", packet, offset, value)


def _legacy_crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _legacy_packet(*, flags: int = 0xCF) -> bytes:
    packet = bytearray(656)
    packet[:4] = b"TJVR"
    struct.pack_into("<HHQQqqI", packet, 4, 4, 656, 7, 11, 1_000_000, 1_000_100, flags)
    for offset, index in ((44, 10), (100, 20)):
        for component, value in enumerate((float(index), float(index + 1), float(index + 2), 0.0, 0.0, 0.0, 1.0)):
            _write_double(packet, offset + component * 8, value)
    for index in range(8):
        struct.pack_into("<ddd", packet, 204 + index * 24, float(index), float(index + 1), float(index + 2))
        struct.pack_into("<dddd", packet, 396 + index * 32, 0.0, 0.0, 0.0, 1.0)
    struct.pack_into("<I", packet, 652, _legacy_crc32(packet[:652]))
    return bytes(packet)


def _invalid_manus_frame():
    positions = [[float(index), 0.0, 0.0] for index in range(21)]
    semantics = [
        {
            "array_index": index,
            "node_id": 100 + index,
            "parent_id": 99 + index,
            "chain_type": 13 if chain == "hand" else 5,
            "side": 0,
            "finger_joint_type": 0 if chain == "hand" else 1,
        }
        for index, (chain, _joint) in enumerate(MEDIAPIPE_SEMANTIC_ORDER[:-1])
    ]
    return parse_manus_payload(
        {
            "glove_id": "manus-1",
            "side": "right",
            "seq": 4,
            "source_monotonic_ns": 500,
            "sdk_publish_time": 6,
            "nodes": positions,
            "node_quaternions_wxyz": [[1.0, 0.0, 0.0, 0.0] for _ in positions],
            "node_semantics": semantics,
        },
        receiver_instance_id="manus-receiver",
        receiver_frame_sequence=12,
        received_timestamp_ns=1100,
    )


class HandTrackingRuntimeTest(unittest.TestCase):
    def test_tcp_stream_reassembles_split_packet_and_assigns_receiver_metadata(self) -> None:
        stream = PicoPacketStream(
            receiver_instance_id="receiver",
            connection_generation=5,
        )
        packet = _packet()
        frames = stream.feed(packet[:17], received_timestamp_ns=10)
        self.assertEqual(frames, [])
        frames = stream.feed(packet[17:], received_timestamp_ns=11)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].receiver_frame_sequence, 0)
        self.assertEqual(frames[0].received_timestamp_ns, 11)

    def test_pico_tcp_receiver_closes_socket_and_advances_generation_on_successful_connection(self) -> None:
        class FakeSocket:
            def __init__(self) -> None:
                self.closed = False
                self.sent = False

            def settimeout(self, _timeout) -> None:
                return None

            def recv(self, _size):
                if not self.sent:
                    self.sent = True
                    return _packet()
                raise AssertionError("receiver should stop after the first callback")

            def close(self) -> None:
                self.closed = True

            def __enter__(self):
                return self

            def __exit__(self, *_args) -> None:
                self.close()

        fake_socket = FakeSocket()
        errors = []
        frames = []
        receiver = PicoTcpReceiver(
            host="127.0.0.1",
            port=10002,
            receiver_instance_id="pico-receiver",
            auto_adb_forward=False,
            socket_factory=lambda *_args, **_kwargs: fake_socket,
            on_error=errors.append,
        )

        def on_frame(frame) -> None:
            frames.append(frame)
            receiver.stop()

        receiver.run(on_frame)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].connection_generation, 1)
        self.assertTrue(fake_socket.closed)
        self.assertEqual(errors, [])

    def test_pico_frame_produces_wrist_relative_hand_and_head_relative_arm_inputs(self) -> None:
        frame = parse_pico_packet(
            _packet(),
            receiver_instance_id="receiver",
            connection_generation=1,
            receiver_frame_sequence=3,
            received_timestamp_ns=100,
        )

        observations = pico_frame_observations(frame)

        self.assertEqual(set(observations), {"left", "right"})
        left_hand, left_arm = observations["left"]
        np.testing.assert_allclose(left_hand.keypoints_m[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(left_hand.keypoints_m[1], [1.0, 1.0, 1.0], atol=1.0e-5)
        self.assertEqual(left_hand.coordinate_frame, "pico_tracking_initial_wrist_relative")
        self.assertEqual(left_arm.reference_frame, "pico_head_current")
        np.testing.assert_allclose(left_arm.pose[:3], [100.0, 100.0, 100.0], atol=1.0e-5)

    def test_pico_hand_block_invalidates_canonical_points_even_if_joint_flags_are_set(self) -> None:
        packet = bytearray(_packet())
        # Payload flags retain a valid left hand bit in the header fixture, but
        # the left hand block itself is marked invalid.
        left_hand_offset = HEADER.size + 4 + 28
        packet[left_hand_offset] = 0
        frame = parse_pico_packet(
            bytes(packet),
            receiver_instance_id="receiver",
            connection_generation=1,
            receiver_frame_sequence=3,
            received_timestamp_ns=100,
        )

        left_hand, _left_arm = pico_frame_observations(frame)["left"]

        self.assertFalse(left_hand.valid)
        self.assertFalse(bool(left_hand.joint_valid.any()))
        np.testing.assert_array_equal(left_hand.keypoints_m, np.zeros((21, 3)))

    def test_observation_runtime_only_publishes_receive_streams(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="publisher",
            router_zid="router",
        )
        frame = parse_pico_packet(
            _packet(),
            receiver_instance_id="receiver",
            connection_generation=1,
            receiver_frame_sequence=3,
            received_timestamp_ns=100,
        )

        runtime.ingest_pico(frame)

        keys = [key for key, _payload in published]
        self.assertIn(topics.RAW_PICO_HAND_TRACKING, keys)
        self.assertEqual(keys.count(topics.hand_observation("left")), 1)
        self.assertEqual(keys.count(topics.hand_observation("right")), 1)
        self.assertEqual(keys.count(topics.arm_input_observation("left")), 1)
        self.assertEqual(keys.count(topics.arm_input_observation("right")), 1)
        self.assertFalse(any(key.startswith("tianji/target/") or key.startswith("tianji/command/") for key in keys))

        hand_payload = next(payload for key, payload in published if key == topics.hand_observation("left"))
        arm_payload = next(payload for key, payload in published if key == topics.arm_input_observation("left"))
        self.assertIsInstance(HandSkeletonObservation.from_dict(hand_payload), HandSkeletonObservation)
        self.assertIsInstance(ArmInputWire.from_dict(arm_payload), ArmInputWire)

    def test_legacy_pico_runtime_publishes_arm_observations_without_hand_or_command_streams(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="publisher",
            router_zid="router",
        )
        frame = parse_legacy_pico_packet(
            _legacy_packet(),
            receiver_instance_id="legacy-receiver",
            receiver_frame_sequence=8,
            received_timestamp_ns=100,
        )

        result = runtime.ingest_legacy_palm(frame)

        self.assertEqual(set(result), {"left", "right"})
        keys = [key for key, _payload in published]
        self.assertIn(topics.RAW_LEGACY_PICO_PALM, keys)
        self.assertEqual(keys.count(topics.arm_input_observation("left")), 1)
        self.assertEqual(keys.count(topics.arm_input_observation("right")), 1)
        self.assertFalse(any(key.startswith("tianji/observation/hand/") for key in keys))
        self.assertFalse(any(key.startswith("tianji/target/") or key.startswith("tianji/command/") for key in keys))
        arm_payload = next(payload for key, payload in published if key == topics.arm_input_observation("right"))
        arm = ArmInputWire.from_dict(arm_payload)
        self.assertTrue(arm.valid)
        self.assertEqual(arm.source, "legacy_pico_palm")
        self.assertEqual(arm.source_sequence, 7)
        self.assertEqual(arm.reference_frame, "legacy_pico_tracking")

    def test_legacy_pico_runtime_keeps_invalid_corrected_arm_observation_observable(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="publisher",
            router_zid="router",
        )
        frame = parse_legacy_pico_packet(
            _legacy_packet(flags=0x0F),
            receiver_instance_id="legacy-receiver",
            receiver_frame_sequence=8,
            received_timestamp_ns=100,
        )

        runtime.ingest_legacy_palm(frame)

        arm_payload = next(payload for key, payload in published if key == topics.arm_input_observation("left"))
        arm = ArmInputWire.from_dict(arm_payload)
        self.assertFalse(arm.valid)
        self.assertIsNone(arm.pose)

    def test_manus_mapping_failure_publishes_invalid_hand_observation_after_raw_frame(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="publisher",
            router_zid="router",
        )

        runtime.ingest_manus_frame(_invalid_manus_frame())

        self.assertIn(topics.RAW_MANUS_HAND_TRACKING, [key for key, _payload in published])
        payload = next(payload for key, payload in published if key == topics.hand_observation("right"))
        observation = HandSkeletonObservation.from_dict(payload)
        self.assertFalse(observation.valid)
        self.assertEqual(observation.joint_valid, [False] * 21)

    def test_legacy_udp_receiver_validates_and_decodes_datagrams(self) -> None:
        class FakeSocket:
            def __init__(self) -> None:
                self.packets = [(_legacy_packet(), ("127.0.0.1", 42000))]
                self.closed = False

            def setsockopt(self, *_args) -> None:
                return None

            def bind(self, _address) -> None:
                return None

            def settimeout(self, _timeout) -> None:
                return None

            def recvfrom(self, _size):
                if self.packets:
                    return self.packets.pop(0)
                raise socket.timeout

            def close(self) -> None:
                self.closed = True

        fake_socket = FakeSocket()
        frames = []
        receiver = LegacyPicoUdpReceiver(
            host="127.0.0.1",
            port=15000,
            receiver_instance_id="legacy-receiver",
            socket_factory=lambda *_args: fake_socket,
        )
        def on_frame(frame) -> None:
            frames.append(frame)
            receiver.stop()

        receiver.run(on_frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 7)
        self.assertEqual(frames[0].receiver_frame_sequence, 0)
        self.assertTrue(fake_socket.closed)
        self.assertTrue(receiver.stop_event.is_set())


if __name__ == "__main__":
    unittest.main()
