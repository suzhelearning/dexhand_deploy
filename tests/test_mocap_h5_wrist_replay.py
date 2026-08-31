from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from tianji_teleop.protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    HAND_JOINT_NAMES,
    ArmJointState,
    Frame0HandSkeleton,
    HandJointState,
)
from tianji_teleop.joint_state_model import urdf_joint_names
from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
from tianji_world_output.config_loader import get_config


ROOT = Path(__file__).resolve().parents[1]
def _load_replay_module():
    from tianji_teleop.diagnostics import mujoco_h5_wrist_replay

    return mujoco_h5_wrist_replay
class MocapH5WristReplayCoordinateTest(unittest.TestCase):
    def test_real_overlay_forces_x11_before_importing_viewer(self) -> None:
        replay = _load_replay_module()
        events = []
        real_import = __import__

        class ViewerImportReached(Exception):
            pass

        def tracked_import(name, *args, **kwargs):
            if name == "mujoco.viewer":
                events.append("viewer")
                raise ViewerImportReached
            return real_import(name, *args, **kwargs)

        with patch.object(
            replay,
            "_configure_viewer_platform",
            side_effect=lambda: events.append("x11"),
            create=True,
        ), patch("builtins.__import__", side_effect=tracked_import):
            with self.assertRaises(ViewerImportReached):
                replay._run_viewer(SimpleNamespace())
        self.assertEqual(events, ["x11", "viewer"])

    def test_motive_world_axes_follow_config_not_marker_axes(self) -> None:
        replay = _load_replay_module()
        config = get_config()
        urdf = (
            ROOT
            / "src/tianji_teleop/assets/marvin_m6_ccs/urdf"
            / "marvin_m6_s_ccs_696_v4_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        home = np.concatenate(
            (
                np.asarray(config.init_joints["left"]),
                np.asarray(config.init_joints["right"]),
            )
        )
        for name, angle_deg in zip(urdf_joint_names(), home):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.assertGreaterEqual(joint_id, 0)
            data.qpos[model.jnt_qposadr[joint_id]] = np.deg2rad(
                float(angle_deg)
            )
        mujoco.mj_forward(model, data)

        axis_x = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "TCP_Link_R_axis_0"
        )
        axis_z = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "TCP_Link_R_axis_2"
        )
        _position, rotation_tcp_mj = replay._frame_from_axis_geoms(
            data, axis_x, axis_z, 0.025
        )
        actual = replay._sim_from_motive_rotation(
            rotation_tcp_mj, config
        )
        expected = np.asarray(config.mocap_to_robot, dtype=np.float64)

        # 当前组合 URDF 世界轴与 Robot world 对齐；允许 URDF/FK 数值舍入。
        np.testing.assert_allclose(actual, expected, atol=1.0e-4)
        self.assertAlmostEqual(float(np.linalg.det(actual)), 1.0, places=6)
        np.testing.assert_allclose(actual[:, 0], [1.0, 0.0, 0.0], atol=1e-4)
        np.testing.assert_allclose(actual[:, 1], [0.0, 1.0, 0.0], atol=1e-4)
        np.testing.assert_allclose(actual[:, 2], [0.0, 0.0, 1.0], atol=1e-4)

    def test_real_state_mirror_starts_at_home_and_applies_authorized_feedback(self) -> None:
        replay = _load_replay_module()
        urdf = (
            ROOT
            / "src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        mirror = replay.RealStateMirror(
            model,
            data,
            router_zid="router",
            arm_instance="marvin-instance",
            hand_instance="wuji-instance",
        )
        mirror.apply()
        robot = mirror.robot
        for name, expected in zip(ALL_ARM_JOINT_NAMES, robot.home_all):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertAlmostEqual(data.qpos[model.jnt_qposadr[joint_id]], expected)

        arm_values = [value * 0.5 for value in robot.home_all]
        hand_values = [0.1] * 20
        self.assertTrue(mirror.on_arm_state(ArmJointState(
            1, 1, 10, "marvin", list(ALL_ARM_JOINT_NAMES), arm_values, None,
            "marvin-instance", "router",
        ).to_dict()))
        self.assertTrue(mirror.on_hand_state(HandJointState(
            1, 1, 10, "wuji_hand2", "right", list(HAND_JOINT_NAMES["right"]),
            hand_values, None, "wuji-instance", "router",
        ).to_dict()))
        self.assertFalse(mirror.on_arm_state(ArmJointState(
            1, 2, 11, "marvin", list(ALL_ARM_JOINT_NAMES), arm_values, None,
            "other-instance", "router",
        ).to_dict()))
        mirror.apply()
        for name, expected in zip(ALL_ARM_JOINT_NAMES, arm_values):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertAlmostEqual(data.qpos[model.jnt_qposadr[joint_id]], expected)
        for name, expected in zip(HAND_JOINT_NAMES["right"], hand_values):
            model_name = name.replace("_mcp_", "_finger_mcp_").replace(
                "_pip", "_finger_pip"
            ).replace("_dip", "_finger_dip") if "thumb_" not in name else name
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, model_name)
            self.assertAlmostEqual(data.qpos[model.jnt_qposadr[joint_id]], expected)

    def test_expected_h5_overlay_aligns_path_and_keypoints_to_home(self) -> None:
        replay = _load_replay_module()
        urdf = ROOT / "src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf"
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        mirror = replay.RealStateMirror(
            model, data, router_zid="router",
            arm_instance="marvin-instance", hand_instance="wuji-instance",
        )
        mirror.apply()
        mujoco.mj_forward(model, data)
        home_position, home_rotation = replay._frame_from_wrist_axis_geoms(
            model, data
        )
        wrist = np.asarray([
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
            [1.1, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        ])
        overlay = replay.ExpectedH5Overlay(
            model, data,
            SimpleNamespace(hands={"right": SimpleNamespace(
                wrist=wrist, valid=np.asarray([True, True]),
            )}),
            router_zid="router", source_instance="source-instance",
        )
        keypoints = np.repeat(wrist[:1, :3], 21, axis=0)
        skeleton = Frame0HandSkeleton(
            schema_version=1,
            timestamp_ns=1,
            side="right",
            frame_id="motive_world",
            keypoints_world_m=keypoints.tolist(),
            edges=[[index, index + 1] for index in range(20)],
            manus_wrist_pose=wrist[0].tolist(),
            robot_wrist_home_pose=wrist[0].tolist(),
            target_wrist_pose=wrist[1].tolist(),
            tcp_to_wrist_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            sequence=2,
            publisher_instance_id="source-instance",
            router_zid="router",
        )
        self.assertTrue(overlay.on_skeleton(skeleton.to_dict()))
        path, points, expected_wrist, edges = overlay.snapshot()
        np.testing.assert_allclose(path[0], home_position, atol=1e-9)
        np.testing.assert_allclose(points[0], home_position, atol=1e-9)
        np.testing.assert_allclose(
            expected_wrist,
            home_position + home_rotation @ np.asarray([0.1, 0.0, 0.0]),
            atol=1e-9,
        )
        np.testing.assert_array_equal(
            edges, [[index, index + 1] for index in range(20)]
        )
        rejected = skeleton.to_dict()
        rejected["publisher_instance_id"] = "other-source"
        self.assertFalse(overlay.on_skeleton(rejected))


if __name__ == "__main__":
    unittest.main()
