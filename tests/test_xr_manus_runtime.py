import unittest

import numpy as np

from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback
from tianji_teleop.hand_tracking.runtime import ObservationRuntime
from tianji_teleop.hand_tracking.xr_input import XrBindingConfig, XrRoboToolkitSource
from tianji_teleop.hand_tracking.xr_manus_runtime import (
    manus_callback_observations,
    xr_frame_arm_observations,
)
from tianji_teleop.protocol import topics


class XrManusRuntimeTest(unittest.TestCase):
    def test_manus_callback_becomes_two_canonical_wrist_relative_observations(self):
        right = np.arange(63, dtype=np.float32).reshape(21, 3)
        left = (100.0 + np.arange(63, dtype=np.float32)).reshape(21, 3)
        callback = ManusCallback(
            receiver_instance_id="manus-receiver",
            sequence=7,
            received_timestamp_ns=900,
            points=tuple(np.concatenate((right, left)).tolist()),
            source_sequences={"right": 11, "left": 12},
            source_timestamps_ns={"right": 101, "left": 102},
        )
        result = manus_callback_observations(callback, sides=("right", "left"))
        self.assertEqual(set(result), {"left", "right"})
        for side, expected, source_sequence, timestamp in (
            ("right", right, 11, 101), ("left", left, 12, 102)
        ):
            observation = result[side]
            self.assertTrue(observation.valid)
            self.assertEqual(observation.source, "manus")
            self.assertEqual(observation.source_sequence, source_sequence)
            self.assertEqual(observation.source_timestamp_ns, timestamp)
            self.assertEqual(observation.receiver_frame_sequence, 7)
            np.testing.assert_allclose(observation.keypoints_m, expected - expected[0])
            np.testing.assert_allclose(observation.keypoints_m[0], [0, 0, 0])

    def test_flat_manus_assembler_callback_becomes_two_canonical_observations(self):
        right = np.arange(63, dtype=np.float32).reshape(21, 3)
        left = (100.0 + np.arange(63, dtype=np.float32)).reshape(21, 3)
        callback = ManusCallback(
            receiver_instance_id="manus-receiver",
            sequence=8,
            received_timestamp_ns=900,
            points=tuple(np.concatenate((right, left)).reshape(-1).tolist()),
            source_sequences={"right": 13, "left": 14},
            source_timestamps_ns={"right": 201, "left": 202},
        )

        result = manus_callback_observations(callback, sides=("right", "left"))

        np.testing.assert_allclose(result["right"].keypoints_m, right - right[0])
        np.testing.assert_allclose(result["left"].keypoints_m, left - left[0])
        self.assertEqual(result["right"].source_sequence, 13)
        self.assertEqual(result["left"].source_sequence, 14)

    def test_incomplete_callback_shape_is_rejected_before_publication(self):
        callback = ManusCallback(
            receiver_instance_id="manus-receiver", sequence=0,
            received_timestamp_ns=1, points=tuple(np.zeros((20, 3)).tolist()),
            source_sequences={}, source_timestamps_ns={},
        )
        with self.assertRaisesRegex(ValueError, "points"):
            manus_callback_observations(callback, sides=("right",))

    def test_xr_arm_observation_carries_wrist_and_forearm_tracker_poses(self):
        class Client:
            def init(self): return True
            def close(self): return None
            def get_time_stamp_ns(self): return 100
            def get_headset_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_right_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_trigger(self): return 0
            def get_right_trigger(self): return 0
            def get_left_grip(self): return 0
            def get_right_grip(self): return 0
            def get_left_axis(self): return [0, 0]
            def get_right_axis(self): return [0, 0]
            def num_motion_data_available(self): return 4
            def get_motion_tracker_pose(self):
                return [
                    [0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 1],
                    [0.1, 0.2, 0.3, 0, 0, 0, 1],
                    [0.4, 0.5, 0.6, 0, 0, 0, 1],
                ]
            def get_motion_tracker_serial_numbers(self):
                return ["190058", "190600", "190046", "190023"]

        binding = XrBindingConfig(
            "xr_tracker",
            {"left": "190058", "right": "190600"},
            {"left": "left", "right": "right"},
            {"left": "190046", "right": "190023"},
        )
        source = XrRoboToolkitSource(
            client=Client(), binding=binding, receiver_instance_id="xr-receiver", clock=lambda: 200,
        )
        source.initialize()
        result = xr_frame_arm_observations(
            source.read_frame(), binding=binding,
            tracked_frame="wrist_tracker", reference_frame="xr_tracking",
        )
        np.testing.assert_allclose(result["left"].pose, [0, 0, 0, 0, 0, 0, 1])
        np.testing.assert_allclose(result["left"].elbow_pose, [0.1, 0.2, 0.3, 0, 0, 0, 1])
        np.testing.assert_allclose(result["right"].elbow_pose, [0.4, 0.5, 0.6, 0, 0, 0, 1])
        self.assertEqual(result["left"].source_instance_id, "xr-receiver:1")
        self.assertIn(":1:0", result["left"].frame_association_id)

    def test_observation_runtime_publishes_xr_raw_and_canonical_arm_streams(self):
        class Client:
            def init(self): return True
            def close(self): return None
            def get_time_stamp_ns(self): return 100
            def get_headset_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_right_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_trigger(self): return 0
            def get_right_trigger(self): return 0
            def get_left_grip(self): return 0
            def get_right_grip(self): return 0
            def get_left_axis(self): return [0, 0]
            def get_right_axis(self): return [0, 0]
            def num_motion_data_available(self): return 2
            def get_motion_tracker_pose(self):
                return [[0, 0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0, 1]]
            def get_motion_tracker_serial_numbers(self): return ["190058", "190600"]

        source = XrRoboToolkitSource(
            client=Client(),
            binding=XrBindingConfig("xr_tracker", {"left": "190058", "right": "190600"}),
            receiver_instance_id="xr-receiver",
            clock=lambda: 200,
        )
        source.initialize()
        messages = []
        runtime = ObservationRuntime(
            publish=lambda key, value: messages.append((key, value)),
            publisher_instance_id="observation",
            router_zid="router",
        )
        runtime.ingest_xr(source.read_frame(), source.binding)
        self.assertEqual({key for key, _ in messages}, {
            topics.RAW_XR_INPUT, topics.arm_input_observation("left"),
            topics.arm_input_observation("right"),
        })

    def test_observation_runtime_writes_raw_xr_to_an_injected_session_writer(self):
        class Client:
            def init(self): return True
            def close(self): return None
            def get_time_stamp_ns(self): return 100
            def get_headset_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_right_controller_pose(self): return [0, 0, 0, 0, 0, 0, 1]
            def get_left_trigger(self): return 0
            def get_right_trigger(self): return 0
            def get_left_grip(self): return 0
            def get_right_grip(self): return 0
            def get_left_axis(self): return [0, 0]
            def get_right_axis(self): return [0, 0]
            def num_motion_data_available(self): return 2
            def get_motion_tracker_pose(self):
                return [[0, 0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0, 1]]
            def get_motion_tracker_serial_numbers(self): return ["190058", "190600"]

        class Writer:
            def __init__(self):
                self.raw = []
                self.arms = []

            def append_raw_xr(self, frame):
                self.raw.append(frame)

            def append_arm_input_observation(self, observation):
                self.arms.append(observation)

        source = XrRoboToolkitSource(
            client=Client(),
            binding=XrBindingConfig("xr_tracker", {"left": "190058", "right": "190600"}),
            receiver_instance_id="xr-receiver",
            clock=lambda: 200,
        )
        source.initialize()
        writer = Writer()
        runtime = ObservationRuntime(
            publish=lambda *_: None,
            publisher_instance_id="observation",
            router_zid="router",
            session_writer=writer,
        )
        frame = source.read_frame()

        runtime.ingest_xr(frame, source.binding)

        self.assertEqual(writer.raw, [frame])
        self.assertEqual(len(writer.arms), 2)


if __name__ == "__main__":
    unittest.main()
