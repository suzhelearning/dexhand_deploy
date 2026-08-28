from __future__ import annotations

import importlib.util
import logging
import tempfile
import unittest
from pathlib import Path
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
    @staticmethod
    def _viewer_module():
        path = (
            Path(__file__).resolve().parents[1]
            / "src/pico_body_tianji/scripts/mujoco_joint_viewer.py"
        )
        spec = importlib.util.spec_from_file_location("viewer_regression", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"无法加载 viewer：{path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_apply_latest_consumes_canonical_target_and_solved(self) -> None:
        viewer = self._viewer_module()
        skeleton = viewer.FrameZeroHandSkeleton.__new__(
            viewer.FrameZeroHandSkeleton
        )
        envelope = ProtocolEnvelope(1, "producer-instance", "router", 3, 4)
        target = ArmTargetCommand(
            envelope=envelope,
            source_timestamp_ns=None,
            source="mocap_h5_replay",
            side="right",
            frame_id="Base_R",
            position_m=[0.1, 0.2, 0.3],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            elbow_reference_direction=[0.0, 1.0, 0.0],
        )
        solved = ArmSolvedPose(
            envelope=envelope,
            producer="ik",
            side="right",
            frame_id="Base_R",
            target_sequence=3,
            position_m=[0.11, 0.2, 0.3],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
        )
        skeleton._pending_target = target.to_dict()
        skeleton._pending_solved = solved.to_dict()
        self.assertTrue(skeleton._apply_pending_producer_messages())
        np.testing.assert_allclose(
            skeleton._target_tcp_pose_chest, [0.1, 0.2, 0.3, 0, 0, 0, 1]
        )
        np.testing.assert_allclose(
            skeleton._solved_tcp_pose_chest, [0.11, 0.2, 0.3, 0, 0, 0, 1]
        )

    def test_viewer_default_frame0_topic_is_canonical(self) -> None:
        viewer = self._viewer_module()
        with patch.object(viewer, "parse_cli_args", return_value=SimpleNamespace()):
            # Static source contract is checked by the parser declaration below;
            # direct topic value is part of the public viewer launcher contract.
            self.assertIn("tianji/diagnostics/h5/frame0_hand_skeleton", viewer.topics.FRAME0_HAND_SKELETON)


if __name__ == "__main__":
    unittest.main()
