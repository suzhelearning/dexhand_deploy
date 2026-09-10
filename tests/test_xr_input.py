import unittest
from unittest.mock import patch
import os
import sys

import numpy as np

from tianji_teleop.hand_tracking.xr_input import (
    XrBindingConfig,
    XrControllerState,
    XrFrame,
    XRoboToolkitClient,
    XrRoboToolkitSource,
    XrTrackerState,
    validate_xr_sdk_module,
)


class _FakeXrClient:
    def __init__(self):
        self.initialized = False

    def init(self):
        self.initialized = True
        return True

    def close(self):
        self.initialized = False

    def get_time_stamp_ns(self):
        return 123456789

    def get_headset_pose(self):
        return [0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0]

    def get_left_controller_pose(self):
        return [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

    def get_right_controller_pose(self):
        return [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0]

    def num_motion_data_available(self):
        return 2

    def get_motion_tracker_pose(self):
        return [
            [0.7, 0.8, 0.9, 0.0, 0.0, 0.0, 1.0],
            [1.0, 1.1, 1.2, 0.0, 0.0, 0.0, 1.0],
        ]

    def get_motion_tracker_serial_numbers(self):
        return ["PICO-MOTION-190058", "PICO-MOTION-190600"]

    def get_left_trigger(self):
        return 0.1

    def get_right_trigger(self):
        return 0.9

    def get_left_grip(self):
        return 0.2

    def get_right_grip(self):
        return 0.8

    def get_left_axis(self):
        return [0.0, 0.0]

    def get_right_axis(self):
        return [0.25, -0.5]


class XrInputTest(unittest.TestCase):
    def binding(self, **overrides):
        value = dict(
            arm_input="xr_tracker",
            tracker_serials={"left": "190058", "right": "190600"},
            controller_sides={"left": "left", "right": "right"},
        )
        value.update(overrides)
        return XrBindingConfig(**value)

    def test_source_reads_hmd_controllers_trackers_and_buttons(self):
        client = _FakeXrClient()
        source = XrRoboToolkitSource(
            client=client,
            binding=self.binding(),
            receiver_instance_id="xr-receiver",
            clock=lambda: 987654321,
        )
        self.assertTrue(source.initialize())
        frame = source.read_frame()

        self.assertEqual(frame.sequence, 0)
        self.assertEqual(frame.connection_generation, 1)
        self.assertEqual(frame.source_timestamp_ns, 123456789)
        self.assertEqual(frame.received_timestamp_ns, 987654321)
        np.testing.assert_allclose(frame.hmd_pose, [0.0, 0.0, 1.6, 0, 0, 0, 1])
        self.assertEqual(frame.controller("right").grip, 0.8)
        self.assertEqual(frame.controller("right").axis.tolist(), [0.25, -0.5])
        self.assertEqual(frame.tracker("left").serial_number, "PICO-MOTION-190058")
        self.assertEqual(frame.tracker("right").serial_number, "PICO-MOTION-190600")
        np.testing.assert_allclose(
            self.binding().pose_for_arm(frame, "left"),
            [0.7, 0.8, 0.9, 0, 0, 0, 1],
        )

    def test_source_accepts_numpy_tracker_and_serial_arrays(self):
        class NumpyArrayClient(_FakeXrClient):
            def get_motion_tracker_pose(self):
                return np.asarray(super().get_motion_tracker_pose(), dtype=np.float64)

            def get_motion_tracker_serial_numbers(self):
                return np.asarray(super().get_motion_tracker_serial_numbers(), dtype=object)

            def get_motion_tracker_velocity(self):
                return np.zeros((2, 6), dtype=np.float64)

            def get_motion_tracker_acceleration(self):
                return np.zeros((2, 6), dtype=np.float64)

        source = XrRoboToolkitSource(
            client=NumpyArrayClient(),
            binding=self.binding(),
            receiver_instance_id="xr-receiver",
            clock=lambda: 987654321,
        )
        self.assertTrue(source.initialize())
        frame = source.read_frame()
        self.assertEqual(len(frame.trackers), 2)
        self.assertEqual(frame.tracker("left").serial_number, "PICO-MOTION-190058")
        np.testing.assert_allclose(frame.tracker("right").velocity, np.zeros(6))

    def test_zero_sdk_timestamp_is_recorded_as_unavailable(self):
        class NoSourceTimestampClient(_FakeXrClient):
            def get_time_stamp_ns(self):
                return 0

        source = XrRoboToolkitSource(
            client=NoSourceTimestampClient(),
            binding=self.binding(),
            receiver_instance_id="xr-receiver",
            clock=lambda: 987654321,
        )
        self.assertTrue(source.initialize())
        self.assertIsNone(source.read_frame().source_timestamp_ns)

    def test_source_reconnect_starts_a_new_generation_and_sequence(self):
        clock_values = iter((10, 20))
        source = XrRoboToolkitSource(
            client=_FakeXrClient(),
            binding=self.binding(),
            receiver_instance_id="xr-receiver",
            clock=lambda: next(clock_values),
        )
        self.assertTrue(source.initialize())
        first = source.read_frame()
        source.close()
        self.assertTrue(source.initialize())
        second = source.read_frame()

        self.assertEqual(first.connection_generation, 1)
        self.assertEqual(first.sequence, 0)
        self.assertEqual(second.connection_generation, 2)
        self.assertEqual(second.sequence, 0)

    def test_controller_mode_selects_controller_pose_and_explicit_binding(self):
        source = XrRoboToolkitSource(
            client=_FakeXrClient(),
            binding=self.binding(arm_input="xr_controller"),
            receiver_instance_id="xr-receiver",
            clock=lambda: 1,
        )
        source.initialize()
        frame = source.read_frame()
        np.testing.assert_allclose(
            source.binding.pose_for_arm(frame, "right"),
            [0.4, 0.5, 0.6, 0, 0, 0, 1],
        )

    def test_duplicate_tracker_serials_and_invalid_controller_state_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.binding(tracker_serials={"left": "190058", "right": "190058"})
        with self.assertRaisesRegex(ValueError, "non-empty serials"):
            self.binding(arm_input="xr_controller", tracker_serials={"left": []})
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.binding(
                arm_input="xr_controller",
                tracker_serials={"left": "190058", "right": "190058"},
            )
        with self.assertRaises(ValueError):
            XrControllerState(
                side="left", pose=None, valid=True, available=True,
                trigger=0.0, grip=0.0, axis=[0.0, np.nan],
            )

    def test_forearm_tracker_binding_is_explicit_and_side_safe(self):
        frame = XrFrame(
            sequence=1,
            source_timestamp_ns=2,
            received_timestamp_ns=3,
            hmd_pose=None,
            controllers={
                "left": XrControllerState("left", None, False, False, 0, 0, [0, 0]),
                "right": XrControllerState("right", None, False, False, 0, 0, [0, 0]),
            },
            trackers=(
                XrTrackerState("device-190058", [0, 0, 0, 0, 0, 0, 1], True, "left"),
                XrTrackerState("device-190600", [1, 0, 0, 0, 0, 0, 1], True, "right"),
                XrTrackerState("device-190046", [0.1, 0.2, 0.3, 0, 0, 0, 1], True),
                XrTrackerState("device-190023", [0.4, 0.5, 0.6, 0, 0, 0, 1], True),
            ),
        )
        binding = self.binding(
            elbow_tracker_serials={"left": "190046", "right": "190023"},
        )
        np.testing.assert_allclose(
            binding.elbow_pose_for_arm(frame, "left"),
            [0.1, 0.2, 0.3, 0, 0, 0, 1],
        )
        np.testing.assert_allclose(
            binding.elbow_pose_for_arm(frame, "right"),
            [0.4, 0.5, 0.6, 0, 0, 0, 1],
        )
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.binding(
                elbow_tracker_serials={"left": "190058", "right": "190023"},
            )

    def test_uninitialized_source_does_not_touch_sdk_implicitly(self):
        client = _FakeXrClient()
        source = XrRoboToolkitSource(
            client=client,
            binding=self.binding(),
            receiver_instance_id="xr-receiver",
        )
        with self.assertRaisesRegex(RuntimeError, "initialized"):
            source.read_frame()
        self.assertFalse(client.initialized)

    def test_missing_xr_sdk_fails_fast_with_install_guidance(self):
        client = XRoboToolkitClient()
        with patch(
            "tianji_teleop.hand_tracking.xr_input.importlib.import_module",
            side_effect=ImportError("module is not installed"),
        ):
            with self.assertRaisesRegex(RuntimeError, "xrobotoolkit_sdk is unavailable"):
                client.init()
        self.assertFalse(client.is_connected())

    def test_sdk_preflight_checks_required_xrobotoolkit_api_without_initializing(self):
        required = {
            "init", "get_headset_pose", "get_left_controller_pose",
            "get_right_controller_pose", "num_motion_data_available",
            "get_motion_tracker_pose", "get_motion_tracker_serial_numbers",
            "get_left_trigger", "get_right_trigger", "get_left_grip",
            "get_right_grip", "get_left_axis", "get_right_axis",
            "get_motion_timestamp_ns",
        }

        class _Sdk:
            pass

        for name in required:
            setattr(_Sdk, name, staticmethod(lambda: None))
        self.assertEqual(validate_xr_sdk_module(_Sdk()), ())

        delattr(_Sdk, "get_right_axis")
        self.assertEqual(validate_xr_sdk_module(_Sdk()), ("get_right_axis",))

    def test_xr_sdk_pythonpath_is_injected_only_when_sdk_is_loaded(self):
        client = XRoboToolkitClient(sdk_module=None)
        injected = os.path.join(os.sep, "portable", "xr-sdk", "python")
        with patch.dict(os.environ, {"TIANJI_XR_SDK_PYTHONPATH": injected}), \
                patch(
                    "tianji_teleop.hand_tracking.xr_input.importlib.import_module",
                    return_value=type("Sdk", (), {"init": staticmethod(lambda: True)})(),
                ) as importer:
            self.assertTrue(client.init())
        self.assertIn(injected, sys.path)
        importer.assert_called_once_with("xrobotoolkit_sdk")

    def test_xr_service_init_failure_remains_reconnectable(self):
        class _UnavailableService:
            def init(self):
                raise OSError("PC-Service is not running")

        client = XRoboToolkitClient(sdk_module=_UnavailableService())
        self.assertFalse(client.init())
        self.assertFalse(client.is_connected())

    def test_client_close_releases_loaded_sdk_when_supported(self):
        class _Sdk:
            def __init__(self):
                self.closed = 0

            def init(self):
                return True

            def close(self):
                self.closed += 1

        sdk = _Sdk()
        client = XRoboToolkitClient(sdk_module=sdk)
        self.assertTrue(client.init())
        client.close()
        self.assertEqual(sdk.closed, 1)
        self.assertFalse(client.is_connected())

    def test_client_close_tolerates_sdk_without_close_hook(self):
        class _Sdk:
            def init(self):
                return True

        client = XRoboToolkitClient(sdk_module=_Sdk())
        self.assertTrue(client.init())
        client.close()
        self.assertFalse(client.is_connected())


if __name__ == "__main__":
    unittest.main()
