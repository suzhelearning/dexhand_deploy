from __future__ import annotations

from pathlib import Path
import runpy
import threading
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig
from tianji_teleop.sources.mocap.h5 import compose_pose, invert_pose
from tianji_teleop.executors.wuji_hand2.config import WujiHandConfig
from tianji_teleop.joint_state_model import urdf_joint_names
from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
from tianji_teleop.protocol.messages import (
    HAND_JOINT_NAMES,
    ComponentStatus,
    HandJointState,
)
from tianji_teleop.regrind_policy import action_to_targets, quat_wxyz_to_rot6d
from tianji_teleop.sources.regrind_policy_node import RegrindPolicyNode, _pose_error


class RegrindRealPreflightTest(unittest.TestCase):
    def test_valid_authorized_hand_state_recovers_from_one_bad_sample(self) -> None:
        node = RegrindPolicyNode.__new__(RegrindPolicyNode)
        node._lock = threading.RLock()
        node._router_zid = "router"
        node._hand_executor_instance_id = "wuji-instance"
        node._hand_config = WujiHandConfig.load()
        node._hand_state = None
        node._hand_received_at = 0.0
        node._hand_sequence = -1
        node._input_error = None
        valid = HandJointState(
            1,
            2,
            2,
            "wuji_hand2",
            "right",
            list(HAND_JOINT_NAMES["right"]),
            list(node._hand_config.zero_position_rad),
            None,
            "wuji-instance",
            "router",
        ).to_dict()
        invalid = dict(valid, sequence=1, position_rad=[])

        node._on_hand_state(invalid)
        self.assertEqual(node._input_error, "position_rad must have shape [20]")
        node._on_hand_state(valid)

        self.assertIsNone(node._input_error)
        self.assertEqual(len(node._hand_state.position_rad), 20)

    def test_connection_ready_does_not_require_live_policy_observation(self) -> None:
        published = []
        node = RegrindPolicyNode.__new__(RegrindPolicyNode)
        node._real_admitted = lambda: (True, None)
        node._fresh_inputs = lambda _now: (None, None, "waiting for live inputs")
        node._last_error = None
        node._phase = "armed"
        node._reference = SimpleNamespace(frame_count=342)
        node._frame_index = 0
        node._frame0_errors = None
        node._deadman_pressed = False
        node._session_client = SimpleNamespace(startup_ready=True)
        node._publisher = SimpleNamespace(
            sequence=1,
            publisher_instance_id="source-instance",
            router_zid="router",
            publish_source_status=lambda **value: published.append(value),
        )
        node._hand_status_pub = SimpleNamespace(put_json=lambda _value: None)

        node._publish_status(1.0)

        self.assertTrue(published[0]["ready"])
        self.assertEqual(published[0]["diagnostics"]["input_error"], "waiting for live inputs")

    def test_alignment_viewer_preserves_zenoh_world_axes(self) -> None:
        scope = runpy.run_path(
            Path(__file__).resolve().parents[1] / "scripts/regrind_live_infer.py",
            run_name="regrind_alignment_axis_check",
        )
        quaternion = Rotation.from_euler("x", 90, degrees=True).as_quat()
        scene_wrist = np.concatenate(([0.5, 0.1, 1.2], quaternion))
        live_wrist = np.concatenate((
            [0.4, -0.2, 0.3],
            quaternion,
        ))
        live_hammer = np.concatenate(([0.4, -0.2, 0.4], live_wrist[3:]))
        expected_wrist, _, current_wrist, current_hammer = scope["_alignment_scene_poses"](
            scene_wrist, live_wrist, live_wrist, live_wrist, live_wrist, live_hammer
        )
        np.testing.assert_allclose(expected_wrist, scene_wrist, atol=1e-9)
        np.testing.assert_allclose(current_wrist, scene_wrist, atol=1e-9)
        np.testing.assert_allclose(current_hammer[:3] - current_wrist[:3], [0.0, 0.0, 0.1], atol=1e-9)

    def test_alignment_viewer_uses_one_world_transform_for_live_and_reference(self) -> None:
        scope = runpy.run_path(
            Path(__file__).resolve().parents[1] / "scripts/regrind_live_infer.py",
            run_name="regrind_alignment_shared_world_check",
        )
        identity = np.asarray([0.0, 0.0, 0.0, 1.0])
        scene_wrist = np.concatenate(([0.5, 0.0, 0.0], identity))
        live_home_wrist = np.concatenate(([0.1, 0.0, 0.0], identity))
        reference_wrist = np.concatenate(([0.2, 0.0, 0.0], identity))
        reference_hammer = np.concatenate(([0.3, 0.0, 0.0], identity))
        live_wrist = np.concatenate(([0.15, 0.0, 0.0], identity))
        live_hammer = np.concatenate(([0.25, 0.0, 0.0], identity))

        _, expected_hammer, current_wrist, current_hammer = scope["_alignment_scene_poses"](
            scene_wrist,
            live_home_wrist,
            reference_wrist,
            reference_hammer,
            live_wrist,
            live_hammer,
        )

        np.testing.assert_allclose(expected_hammer[:3], [0.7, 0.0, 0.0])
        np.testing.assert_allclose(current_wrist[:3], [0.55, 0.0, 0.0])
        np.testing.assert_allclose(current_hammer[:3], [0.65, 0.0, 0.0])

    def test_expected_hand_overlay_follows_expected_wrist_pose(self) -> None:
        scope = runpy.run_path(
            Path(__file__).resolve().parents[1] / "scripts/regrind_live_infer.py",
            run_name="regrind_expected_hand_check",
        )
        root = Path(__file__).resolve().parents[1]
        xml, assets = portable_mujoco_urdf(
            root / "src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf"
        )
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        robot = ArmRobotConfig.load()
        for name, value in zip(urdf_joint_names(), robot.home_all):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint]] = value
        mujoco.mj_forward(model, data)
        position, rotation = scope["_frame_from_wrist_axis_geoms"](model, data)
        home_wrist = np.concatenate((position, Rotation.from_matrix(rotation).as_quat()))
        overlay = scope["_ExpectedHandOverlay"](
            model,
            data.qpos,
            home_wrist,
            np.asarray([[0.0] * 20, [0.1] * 20]),
            mujoco,
        )
        home_scene = mujoco.MjvScene(model, 100)
        shifted_scene = mujoco.MjvScene(model, 100)
        next_frame_scene = mujoco.MjvScene(model, 100)
        overlay.draw(home_scene, mujoco, home_wrist, 0)
        shifted_wrist = home_wrist.copy()
        shifted_wrist[0] += 0.1
        overlay.draw(shifted_scene, mujoco, shifted_wrist, 0)
        overlay.draw(next_frame_scene, mujoco, home_wrist, 1)

        self.assertEqual(home_scene.ngeom, 22)
        self.assertEqual(shifted_scene.ngeom, 22)
        home_positions = np.stack([home_scene.geoms[i].pos for i in range(22)])
        shifted_positions = np.stack([shifted_scene.geoms[i].pos for i in range(22)])
        np.testing.assert_allclose(
            shifted_positions - home_positions,
            np.tile([0.1, 0.0, 0.0], (22, 1)),
            atol=1e-7,
        )
        next_positions = np.stack(
            [next_frame_scene.geoms[i].pos for i in range(22)]
        )
        self.assertGreater(
            float(np.max(np.linalg.norm(next_positions - home_positions, axis=1))),
            0.001,
        )

    def test_policy_frame_tracker_holds_frame0_until_running(self) -> None:
        scope = runpy.run_path(
            Path(__file__).resolve().parents[1] / "scripts/regrind_live_infer.py",
            run_name="regrind_policy_frame_tracker_check",
        )
        tracker = scope["_PolicyFrameTracker"](
            router_zid="router",
            source_instance="source-instance",
            frame_count=342,
        )

        def status(phase: str, frame_index: int, sequence: int = 1):
            return ComponentStatus(
                1,
                sequence,
                sequence,
                "source",
                "regrind_policy",
                phase,
                True,
                True,
                ["real"],
                None,
                {"frame_index": frame_index},
                "source-instance",
                "router",
            ).to_dict()

        self.assertTrue(tracker.on_status(status("ready", 17)))
        self.assertEqual(tracker.current(), ("ready", 0))
        self.assertTrue(tracker.on_status(status("running", 17, 2)))
        self.assertEqual(tracker.current(), ("running", 16))
        foreign = status("running", 30, 3)
        foreign["publisher_instance_id"] = "foreign"
        self.assertFalse(tracker.on_status(foreign))
        self.assertEqual(tracker.current(), ("running", 16))

    def test_home_calibration_preserves_reference_frame0_delta(self) -> None:
        node = RegrindPolicyNode.__new__(RegrindPolicyNode)
        identity = np.asarray([0.0, 0.0, 0.0, 1.0])
        node._home_wrist = np.concatenate(([0.5, 0.0, 0.0], identity))
        live_home = np.concatenate(([0.1, 0.0, 0.0], identity))
        reference_frame0 = np.concatenate(([0.2, 0.0, 0.0], identity))

        node._calibrate_world(live_home)

        np.testing.assert_allclose(node._training_from_motive, [0, 0, 0, 0, 0, 0, 1])
        np.testing.assert_allclose(
            compose_pose(node._robot_from_training, live_home), node._home_wrist
        )
        frame0_robot = compose_pose(node._robot_from_training, reference_frame0)
        np.testing.assert_allclose(frame0_robot[:3], [0.6, 0.0, 0.0])

    def test_rotation_and_action_contract_matches_training_layout(self) -> None:
        quaternion_wxyz = np.roll(Rotation.from_euler("xyz", [20, -30, 40], degrees=True).as_quat(), 1)
        matrix = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
        np.testing.assert_allclose(quat_wxyz_to_rot6d(quaternion_wxyz), matrix[:, :2].reshape(-1))

        action = np.zeros(26)
        action[:6] = [0.5, -0.5, 1.0, 0.0, 0.0, 1.0]
        position, quaternion, joints = action_to_targets(
            action,
            np.asarray([0.2, 0.1, 0.3]),
            np.asarray([1.0, 0.0, 0.0, 0.0]),
            np.zeros(20),
        )
        np.testing.assert_allclose(position, [0.21, 0.09, 0.32])
        np.testing.assert_allclose(
            Rotation.from_quat(np.roll(quaternion, -1)).as_rotvec(), [0.0, 0.0, 0.064]
        )
        np.testing.assert_allclose(joints, 0.0)

    def test_one_wrist_alignment_preserves_hammer_relative_pose(self) -> None:
        live_wrist = np.asarray([0.5, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        live_hammer = np.asarray([0.6, -0.1, 0.2, 0.0, 0.0, 0.0, 1.0])
        reference_wrist = np.asarray([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0])
        training_from_motive = compose_pose(reference_wrist, invert_pose(live_wrist))

        np.testing.assert_allclose(
            compose_pose(training_from_motive, live_wrist), reference_wrist, atol=1e-9
        )
        aligned_hammer = compose_pose(training_from_motive, live_hammer)
        position_error, orientation_error = _pose_error(
            aligned_hammer,
            np.asarray([0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0]),
        )
        self.assertAlmostEqual(position_error, 0.0)
        self.assertAlmostEqual(orientation_error, 0.0)

    def test_frame0_approach_then_i_checks_hammer_before_inference(self) -> None:
        node = RegrindPolicyNode.__new__(RegrindPolicyNode)
        node._reference = SimpleNamespace(
            wrist_pos=np.asarray([[0.0, 0.0, 0.0]]),
            wrist_quat_wxyz=np.asarray([[1.0, 0.0, 0.0, 0.0]]),
            object_pos=np.asarray([[0.0, 0.0, 0.0]]),
            object_quat_wxyz=np.asarray([[1.0, 0.0, 0.0, 0.0]]),
            joints=np.asarray([[0.02] * 20]),
        )
        node._robot_from_training = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        node._wrist_to_tcp = node._robot_from_training.copy()
        node._conditioner = SimpleNamespace(condition=lambda pos, quat: (pos, quat, None))
        node._hand_config = SimpleNamespace(
            lower_limits_rad=np.asarray([-1.0] * 20),
            upper_limits_rad=np.asarray([1.0] * 20),
        )
        node._params = {
            "hand_maximum_step_rad": 0.01,
            "wrist_frame0_position_tolerance_m": 0.01,
            "wrist_frame0_orientation_tolerance_deg": 5.0,
            "hammer_start_position_tolerance_m": 0.01,
            "hammer_start_orientation_tolerance_deg": 5.0,
        }
        node._last_hand_target = None
        sample = SimpleNamespace(
            wrist_xyzw=np.asarray([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            hammer_xyzw=np.asarray([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        )
        node._training_from_motive = node._robot_from_training.copy()

        self.assertFalse(node._approach_frame_zero(sample, np.zeros(20)))
        np.testing.assert_allclose(node._cached_joints, 0.01)
        self.assertFalse(node._approach_frame_zero(sample, np.asarray([0.02] * 20)))
        sample.wrist_xyzw = node._robot_from_training.copy()
        self.assertTrue(node._approach_frame_zero(sample, np.asarray([0.02] * 20)))

        joints = np.asarray([0.02] * 20)
        node._phase = "ready"
        node._read_deadman = lambda: False
        node._real_admitted = lambda: (True, None)
        node._fresh_inputs = lambda _now: (sample, joints, None)
        node._session_client = SimpleNamespace(start_authorized=True)
        node._last_action = np.ones(26)
        node._frame_index = 12

        node._request_inference()
        self.assertEqual(node._phase, "ready")
        sample.hammer_xyzw = node._robot_from_training.copy()
        node._request_inference()
        self.assertEqual(node._phase, "running")
        self.assertEqual(node._frame_index, 0)
        np.testing.assert_allclose(node._last_action, 0.0)


if __name__ == "__main__":
    unittest.main()
