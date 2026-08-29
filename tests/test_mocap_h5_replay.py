from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from pico_body_tianji.protocol.messages import (
    ArmSolvedPose,
    ArmTargetCommand,
    HAND_JOINT_NAMES,
    ProtocolEnvelope,
)
from pico_body_tianji.sources.mocap.h5_replay_node import (
    DEFAULT_PARAMETERS,
    MocapH5ReplayNode,
    _WUJI2_MOUNT_TO_WRIST_POSE,
    _configure_logging,
    _configured_pose,
)
from pico_body_tianji.sources.mocap.h5 import compose_pose, invert_pose


class _Deadman:
    def __init__(self, pressed: bool = False) -> None:
        self.pressed = pressed

    def is_pressed(self) -> bool:
        return self.pressed


class _BrokenDeadman:
    def is_pressed(self) -> bool:
        raise RuntimeError("X11 disconnected")


class _HandPublisher:
    def __init__(self) -> None:
        self.calls: list[dict] = []

    def publish_hand_joint_command(self, **kwargs) -> None:
        self.calls.append(kwargs)


class H5GeometryRegressionTest(unittest.TestCase):
    def test_default_motive_mount_chain_keeps_unit_rotations(self) -> None:
        rigid_to_marker = _configured_pose(
            DEFAULT_PARAMETERS, "right_rigid_to_marker_mocap"
        )
        marker_to_mount = _configured_pose(
            DEFAULT_PARAMETERS, "right_marker_to_mount"
        )
        rigid_to_wrist = compose_pose(
            compose_pose(rigid_to_marker, marker_to_mount),
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )
        tcp_to_wrist = compose_pose(
            _configured_pose(DEFAULT_PARAMETERS, "right_tcp_to_mount"),
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )
        self.assertAlmostEqual(np.linalg.norm(rigid_to_wrist[3:]), 1.0)
        self.assertAlmostEqual(np.linalg.norm(tcp_to_wrist[3:]), 1.0)
        np.testing.assert_allclose(
            compose_pose(rigid_to_wrist, invert_pose(rigid_to_wrist)),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            atol=1e-8,
        )

    def test_h5_wrist_axis_transform_is_non_reflective(self) -> None:
        pose = _configured_pose(
            DEFAULT_PARAMETERS, "right_h5_wrist_to_wuji2_wrist"
        )
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)
        np.testing.assert_allclose(
            rotation @ rotation.T, np.eye(3), atol=1e-9
        )


class H5DeadmanRegressionTest(unittest.TestCase):
    def test_deadman_read_error_is_retained_as_fail_closed_state(self) -> None:
        node = MocapH5ReplayNode.__new__(MocapH5ReplayNode)
        node._deadman = _BrokenDeadman()
        node._deadman_error = None
        node._deadman_pressed = False
        self.assertFalse(node._read_deadman())
        self.assertIn("X11 disconnected", node._deadman_error)

    def test_deadman_released_does_not_advance_state(self) -> None:
        node = MocapH5ReplayNode.__new__(MocapH5ReplayNode)
        node._deadman = _Deadman(False)
        node._deadman_error = None
        node._deadman_pressed = False
        self.assertFalse(node._read_deadman())
        self.assertFalse(node._deadman_pressed)


class H5DirectJointRegressionTest(unittest.TestCase):
    def test_direct_joint_payload_preserves_canonical_20_joint_order(self) -> None:
        node = MocapH5ReplayNode.__new__(MocapH5ReplayNode)
        values = np.linspace(-0.2, 0.2, 40, dtype=np.float64).reshape(2, 20)
        payload = node._build_hand_joint_commands_payload(values)
        self.assertEqual(payload.dtype, np.dtype("<f4"))
        node._publisher = _HandPublisher()
        node._hand_joint_commands_payload = payload
        node._trajectory = SimpleNamespace(start_frame_index=0)
        node._current_source_frame = 1
        node._publisher.publish_hand_joint_command = node._publisher.publish_hand_joint_command
        node._publish_hand_joint_commands()
        call = node._publisher.calls[-1]
        self.assertEqual(call["names"], HAND_JOINT_NAMES["right"])
        np.testing.assert_allclose(call["position_rad"], values[1], atol=1e-6)
        self.assertEqual(call["producer"], "h5_direct")

    def test_direct_payload_keeps_invalid_frames_out_of_real_preflight(self) -> None:
        from pico_body_tianji.sources.mocap.h5_replay_node import (
            validate_h5_hand_real_preflight,
        )

        values = np.zeros((2, 20), dtype=np.float64)
        values[1, 3] = np.nan
        admitted, reason = validate_h5_hand_real_preflight(values)
        self.assertFalse(admitted)
        self.assertIn("finite", reason)


class H5TerminalRegressionTest(unittest.TestCase):
    def test_h5_logging_keeps_blank_line_message_separator(self) -> None:
        with patch(
            "pico_body_tianji.sources.mocap.h5_replay_node.logging.basicConfig"
        ) as configure:
            _configure_logging()
        options = configure.call_args.kwargs
        self.assertTrue(options["force"])
        self.assertEqual(options["handlers"][0].terminator, "\n\n")


class Frame0ViewerRegressionTest(unittest.TestCase):
    def test_canonical_diagnostic_validate_only_is_passive(self) -> None:
        from pico_body_tianji.diagnostics import mujoco_h5_wrist_replay as viewer

        recording = SimpleNamespace(output_hz=60.0, summary=lambda: {"frames": 1})
        with patch.object(viewer, "load_mocap_h5", return_value=recording):
            with patch.object(viewer, "_run_viewer") as run_viewer:
                self.assertEqual(viewer.main(["/tmp/input.h5", "--validate-only"]), 0)
        run_viewer.assert_not_called()

    def test_canonical_diagnostic_viewer_is_opt_in(self) -> None:
        from pico_body_tianji.diagnostics import mujoco_h5_wrist_replay as viewer

        recording = SimpleNamespace(output_hz=60.0, summary=lambda: {"frames": 1})
        with patch.object(viewer, "load_mocap_h5", return_value=recording):
            with patch.object(viewer, "_run_viewer") as run_viewer:
                self.assertEqual(viewer.main(["/tmp/input.h5", "--viewer"]), 0)
        run_viewer.assert_called_once_with(recording)

if __name__ == "__main__":
    unittest.main()
