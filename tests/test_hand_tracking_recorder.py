from __future__ import annotations

import sys
import tempfile
import types
import unittest
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# The repository's recorder imports the optional Zenoh binding through
# zenoh_util.  These tests exercise the deterministic recorder boundary with
# an injected fake transport, so they can run without a router.
if "zenoh" not in sys.modules:
    sys.modules["zenoh"] = types.ModuleType("zenoh")

from tianji_teleop.hand_tracking.models import (  # noqa: E402
    ArmInputObservation as ArmInputObservationModel,
    HandObservation,
    LegacyPicoPalmFrame,
    ManusRawFrame,
    PICO_JOINT_NAMES,
    PicoRawFrame,
    PicoRawHand,
    PicoRawJoint,
)
from tianji_teleop.protocol import topics  # noqa: E402
from tianji_teleop.protocol.messages import (  # noqa: E402
    ArmInputObservation,
    ArmTargetCommand,
    HandSkeletonObservation,
    ProtocolEnvelope,
)
from tianji_teleop.recording.recorder import SessionRecorderNode  # noqa: E402
from tianji_teleop.recording.session_recorder import _load_recording_config  # noqa: E402
from tianji_teleop.recording.session_h5 import (  # noqa: E402
    EXTENDED_SCHEMA_VERSION,
    SessionH5Reader,
)


class _Session:
    def __init__(self) -> None:
        self.subscriptions: list[tuple[str, object]] = []

    def declare_subscriber(self, key: str, callback: object) -> SimpleNamespace:
        self.subscriptions.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)


class RecorderShutdownTest(unittest.TestCase):
    def test_close_waits_for_inflight_append_and_ignores_late_callback(self):
        with tempfile.TemporaryDirectory() as directory:
            node = SessionRecorderNode(_Session(), Path(directory) / 'test.h5',
                source_type='hand_tracking_sim', robot_model='marvin', router_zid='router',
                input_profile='pico', recording_config={'flush_interval_s': 1.0,
                    'schema_name': 'tianji-teleop-session', 'schema_version': EXTENDED_SCHEMA_VERSION})
            entered, release, closed = threading.Event(), threading.Event(), threading.Event()
            original = node.writer.append_raw_pico
            errors = []

            def append(*args, **kwargs):
                entered.set()
                release.wait(3)
                return original(*args, **kwargs)

            node.writer.append_raw_pico = append
            raw = _pico_frame().to_dict()
            raw['router_zid'] = 'router'

            def receive():
                try:
                    node.receive(topics.RAW_PICO_HAND_TRACKING, raw)
                except Exception as error:
                    errors.append(error)

            worker = threading.Thread(target=receive)
            closer = threading.Thread(target=lambda: (node.close(), closed.set()))
            worker.start()
            self.assertTrue(entered.wait(2))
            closer.start()
            prematurely_closed = closed.wait(0.1)
            release.set()
            worker.join(3)
            closer.join(3)
            self.assertFalse(prematurely_closed)
            self.assertFalse(errors)
            self.assertTrue(closed.is_set())
            node._on_sample(topics.RAW_PICO_HAND_TRACKING, raw)


def _pico_frame() -> PicoRawFrame:
    joints = tuple(
        PicoRawJoint(
            index=index,
            name=PICO_JOINT_NAMES[index],
            valid=True,
            pose=np.asarray([index * 0.001, 0.01, 0.02, 0.0, 0.0, 0.0, 1.0]),
            radius_m=0.005,
        )
        for index in range(26)
    )
    hand = PicoRawHand(
        valid=True,
        wrist_valid=True,
        wrist_pose=np.asarray([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
        joints=joints,
    )
    return PicoRawFrame(
        source_timestamp_ms=10,
        protocol_version=1,
        flags=0x07,
        joint_count=26,
        head_valid=True,
        head_pose=np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        hands={"left": hand, "right": hand},
        raw_packet=b"pico-wire",
        received_timestamp_ns=1_000,
        receiver_instance_id="pico-receiver",
        connection_generation=2,
        receiver_frame_sequence=3,
    )


def _manus_frame() -> ManusRawFrame:
    semantics = tuple(
        {
            "array_index": index,
            "node_id": 100 + index,
            "parent_id": 99 + index,
            "chain_type": 13,
            "side": 0,
            "finger_joint_type": 0,
        }
        for index in range(21)
    )
    return ManusRawFrame(
        glove_id="manus-glove",
        side="right",
        source_sequence=8,
        source_monotonic_ns=20_000,
        sdk_publish_time=21,
        node_positions=np.arange(63, dtype=np.float64).reshape(21, 3),
        node_quaternions_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (21, 1)),
        node_semantics=semantics,
        received_timestamp_ns=2_000,
        receiver_instance_id="manus-receiver",
        receiver_frame_sequence=4,
    )


def _legacy_frame() -> LegacyPicoPalmFrame:
    return LegacyPicoPalmFrame(
        protocol_version=1,
        packet_size=160,
        flags=0x0F,
        sequence=12,
        tracking_epoch=2,
        source_timestamp_ns=30_000,
        bridge_send_monotonic_ns=31_000,
        left_pose=np.asarray([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
        right_pose=np.asarray([0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0]),
        upper_limb_skeleton_valid=False,
        upper_limb_rotations_valid=False,
        upper_limb_points=np.zeros((8, 3), dtype=np.float64),
        upper_limb_rotations_xyzw=np.tile([0.0, 0.0, 0.0, 1.0], (8, 1)),
        raw_packet=b"l" * 160,
        received_timestamp_ns=2_100,
        receiver_instance_id="legacy-receiver",
        receiver_frame_sequence=5,
    )


def _hand_wire(model: HandObservation, *, sequence: int = 0) -> HandSkeletonObservation:
    return HandSkeletonObservation(
        schema_version=1,
        sequence=sequence,
        timestamp_ns=model.received_timestamp_ns,
        source_timestamp_ns=model.source_timestamp_ns,
        received_timestamp_ns=model.received_timestamp_ns,
        source=model.source,
        side=model.side,
        source_instance_id=model.source_instance_id,
        source_sequence=model.source_sequence,
        receiver_instance_id=model.receiver_instance_id,
        receiver_frame_sequence=model.receiver_frame_sequence,
        coordinate_frame=model.coordinate_frame,
        mapping_version=model.mapping_version,
        keypoints_m=model.keypoints_m.tolist(),
        joint_valid=model.joint_valid.tolist(),
        valid=model.valid,
        wrist_pose=None if model.wrist_pose is None else model.wrist_pose.tolist(),
        frame_association_id=model.frame_association_id,
        publisher_instance_id="observation-publisher",
        router_zid="router",
    )


def _arm_wire(model: ArmInputObservationModel, *, sequence: int = 0) -> ArmInputObservation:
    return ArmInputObservation(
        schema_version=1,
        sequence=sequence,
        timestamp_ns=model.received_timestamp_ns,
        source_timestamp_ns=model.source_timestamp_ns,
        received_timestamp_ns=model.received_timestamp_ns,
        source=model.source,
        side=model.side,
        tracked_frame=model.tracked_frame,
        reference_frame=model.reference_frame,
        source_instance_id=model.source_instance_id,
        source_sequence=model.source_sequence,
        receiver_instance_id=model.receiver_instance_id,
        receiver_frame_sequence=model.receiver_frame_sequence,
        mapping_version=model.mapping_version,
        pose=None if model.pose is None else model.pose.tolist(),
        valid=model.valid,
        frame_association_id=model.frame_association_id,
        publisher_instance_id="observation-publisher",
        router_zid="router",
    )


class HandTrackingRecorderTest(unittest.TestCase):
    def test_session_recorder_loader_accepts_schema_11_profile(self) -> None:
        config = Path(__file__).parents[1] / "src" / "tianji_teleop" / "config" / "recording" / "session_hand_tracking.yaml"
        loaded = _load_recording_config(str(config))
        self.assertEqual(loaded["schema_version"], EXTENDED_SCHEMA_VERSION)

    def test_pico_session_records_raw_observations_and_targets_in_one_schema_11_file(self) -> None:
        frame = _pico_frame()
        hand = HandObservation(
            source="pico",
            side="left",
            source_instance_id=frame.receiver_instance_id,
            source_sequence=None,
            source_timestamp_ns=frame.source_timestamp_ns,
            received_timestamp_ns=frame.received_timestamp_ns,
            receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            coordinate_frame="pico_tracking_initial_wrist_relative",
            mapping_version="pico26_to_mediapipe21_v1",
            keypoints_m=np.zeros((21, 3), dtype=np.float64),
            joint_valid=np.ones(21, dtype=np.bool_),
            valid=True,
            wrist_pose=frame.hands["left"].wrist_pose,
            frame_association_id=frame.association_id,
        )
        arm_model = ArmInputObservationModel(
            source="pico",
            side="left",
            tracked_frame="wrist",
            reference_frame="pico_head_current",
            pose=np.asarray([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            source_timestamp_ns=frame.source_timestamp_ns,
            received_timestamp_ns=frame.received_timestamp_ns,
            receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            mapping_version="pico_head_current_v1",
            frame_association_id=frame.association_id,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pico.h5"
            node = SessionRecorderNode(
                _Session(),
                path,
                source_type="hand_tracking_sim",
                robot_model="marvin",
                router_zid="router",
                input_profile="pico",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": EXTENDED_SCHEMA_VERSION,
                },
            )
            raw = frame.to_dict()
            raw["router_zid"] = "router"
            node.receive(topics.RAW_PICO_HAND_TRACKING, raw)
            node.receive(topics.hand_observation("left"), _hand_wire(hand))
            node.receive(topics.arm_input_observation("left"), _arm_wire(arm_model))
            target = ArmTargetCommand(
                ProtocolEnvelope(1, "target", "router", 9, 1_100),
                frame.source_timestamp_ns,
                "hand_tracking_target",
                "left",
                "Base_L",
                [0.2, 0.3, 0.4],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
            )
            node.receive(topics.arm_target("left"), target)
            node.close()

            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.attrs["source_type"], "hand_tracking_sim")
                self.assertEqual(reader.read_raw_pico()[0]["association_id"], frame.association_id)
                self.assertEqual(reader.read_hand_observation("left")[0]["frame_association_id"], frame.association_id)
                self.assertEqual(reader.read_arm_input_observation("left")[0]["reference_frame"], "pico_head_current")
                self.assertEqual(reader.read_arm_target("left")[0]["source"], "hand_tracking_target")
                self.assertEqual(reader.read_hand_tracking_metadata()["input_profile"], "pico")

    def test_manus_profile_records_manus_and_historical_palm_raw_streams(self) -> None:
        manus = _manus_frame()
        legacy = _legacy_frame()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manus.h5"
            node = SessionRecorderNode(
                _Session(),
                path,
                source_type="hand_tracking_sim_manus",
                robot_model="marvin",
                router_zid="router",
                input_profile="manus",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": EXTENDED_SCHEMA_VERSION,
                },
            )
            for key, value in (
                (topics.RAW_MANUS_HAND_TRACKING, manus),
                (topics.RAW_LEGACY_PICO_PALM, legacy),
            ):
                payload = value.to_dict()
                payload["router_zid"] = "router"
                node.receive(key, payload)
            node.close()
            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.attrs["source_type"], "hand_tracking_sim_manus")
                self.assertEqual(reader.read_raw_manus()[0]["glove_id"], manus.glove_id)
                self.assertEqual(reader.read_raw_legacy_palm()[0]["packet_size"], 160)
                self.assertEqual(reader.read_hand_tracking_metadata()["input_profile"], "manus")


if __name__ == "__main__":
    unittest.main()
