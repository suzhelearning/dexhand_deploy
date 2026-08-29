"""H5 v4 wrist diagnostic overlay preparation。

该工具只读外部 acquisition v4 文件；可选 MuJoCo passive viewer 用于现场目检
robot home、frame0 skeleton 数据和 wrist/TCP 标定摘要。它不会声明 Zenoh
publisher，不发布 SessionState、JointState 或 final command。
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..sources.mocap.h5 import load_mocap_h5
from tianji_world_output.config_loader import get_config
from tianji_world_output.transform_utils import get_chest_to_world_rotation


def _frame_from_axis_geoms(
    data: object,
    axis_x_geom_id: int,
    axis_z_geom_id: int,
    axis_half_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Recover a right-handed frame from MuJoCo X/Z axis geoms."""
    geom_xmat = np.asarray(data.geom_xmat)
    geom_xpos = np.asarray(data.geom_xpos)
    axis_x = geom_xmat[axis_x_geom_id].reshape(3, 3)[:, 2].copy()
    axis_z = geom_xmat[axis_z_geom_id].reshape(3, 3)[:, 2].copy()
    axis_x /= np.linalg.norm(axis_x)
    axis_z /= np.linalg.norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)
    axis_y /= np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    rotation = np.column_stack((axis_x, axis_y, axis_z))
    origin_x = geom_xpos[axis_x_geom_id] - axis_half_length * axis_x
    origin_z = geom_xpos[axis_z_geom_id] - axis_half_length * axis_z
    return 0.5 * (origin_x + origin_z), rotation


def _sim_from_motive_rotation(
    home_tcp_rotation_mj: np.ndarray,
    tianji_config: object | None = None,
) -> np.ndarray:
    """Return the configured fixed Motive-world to MuJoCo-world rotation."""
    config = tianji_config or get_config()
    tcp_rotation_chest = Rotation.from_quat(config.init_quat["right"]).as_matrix()
    tcp_rotation_world = get_chest_to_world_rotation("right") @ tcp_rotation_chest
    rotation_sim_from_world = home_tcp_rotation_mj @ tcp_rotation_world.T
    result = rotation_sim_from_world @ np.asarray(config.mocap_to_robot, dtype=np.float64)
    if not np.allclose(result @ result.T, np.eye(3), atol=1.0e-4) or not np.isclose(
        np.linalg.det(result), 1.0, atol=1.0e-4
    ):
        raise ValueError("Motive→MuJoCo 世界轴映射不是 det=+1 正交矩阵")
    return result
def _run_viewer(recording) -> None:
    import mujoco
    import mujoco.viewer
    from ..mujoco_urdf import portable_mujoco_urdf

    root = Path(__file__).resolve().parents[4]
    urdf = root / "src" / "pico_body_tianji" / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    xml, assets = portable_mujoco_urdf(urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the model at configured Home while exposing frame0 diagnostics in
        # the terminal. Viewer remains passive: no command/state authority.
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / max(float(recording.output_hz), 1.0))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="read-only H5 wrist MuJoCo diagnostic")
    parser.add_argument("h5", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="open passive MuJoCo overlay")
    args = parser.parse_args(argv)
    recording = load_mocap_h5(args.h5)
    summary = recording.summary()
    summary.update({"overlay": "frame0_hand_skeleton", "executor_authority": "MujocoExecutor", "viewer": bool(args.viewer)})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.viewer:
        _run_viewer(recording)
    return 0


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
