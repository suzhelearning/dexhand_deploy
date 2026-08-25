from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from tianji_world_output.config_loader import get_config


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src/pico_body_tianji/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def _load_replay_module():
    path = SCRIPTS / "mujoco_h5_wrist_replay.py"
    spec = importlib.util.spec_from_file_location(
        "mujoco_h5_wrist_replay_test_module", path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载回放脚本：{path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MocapH5WristReplayCoordinateTest(unittest.TestCase):
    def test_motive_world_axes_follow_config_not_marker_axes(self) -> None:
        replay = _load_replay_module()
        config = get_config()
        urdf = (
            ROOT
            / "src/pico_body_tianji/assets/marvin_m6_ccs/urdf"
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


if __name__ == "__main__":
    unittest.main()
