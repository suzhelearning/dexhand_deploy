#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import re
import time

import mujoco
import mujoco.viewer
import numpy as np
import zenoh
from scipy.spatial.transform import Rotation

from pico_body_tianji.controller_only.mocap_h5 import HAND_KEYPOINT_EDGES
from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_joint_state import apply_joint_positions
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from pico_body_tianji.zenoh_util import ZenohJsonSub, key, open_session, parse_cli_args
from tianji_world_output.config_loader import get_config
from tianji_world_output.transform_utils import (
    get_chest_to_world_rotation,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "assets"
    / "marvin_m6_ccs"
    / "urdf"
    / "marvin_m6_s_ccs_696_v4_mujoco.urdf"
)

_LOG = logging.getLogger("pico_body_mujoco_viewer")


class MujocoJointMirror:
    """把隔离预览关节状态镜像到 MuJoCo qpos，不执行动力学。"""

    def __init__(self, session, model, topic: str):
        self._qpos_addresses = _qpos_addresses(model)
        self._pending: tuple[list[str], list[float]] | None = None
        self._received_once = False
        self._sub = ZenohJsonSub(
            session,
            key(topic),
            self._on_joint_state,
        )
        _LOG.info("等待只读关节状态：%s", topic)

    @property
    def received_once(self) -> bool:
        return self._received_once

    def _on_joint_state(self, msg: dict) -> None:
        self._pending = (list(msg["name"]), list(msg["position"]))

    def apply_latest(self, data) -> int:
        pending = self._pending
        self._pending = None
        if pending is None:
            return 0
        names, positions = pending
        count = apply_joint_positions(
            data.qpos,
            self._qpos_addresses,
            names,
            positions,
        )
        if count and not self._received_once:
            self._received_once = True
            _LOG.info("已接收 %d 个关节；MuJoCo 开始镜像预览", count)
        return count

    def close(self) -> None:
        try:
            self._sub.close()
        except Exception:
            pass


def _qpos_addresses(model) -> dict[str, int]:
    addresses = {}
    for name in urdf_joint_names():
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少关节：{name}")
        addresses[name] = int(model.jnt_qposadr[joint_id])
    return addresses


_POINT_COLORS = np.asarray(
    [
        [1.0, 1.0, 1.0, 0.95],
        *([[1.0, 0.78, 0.16, 0.95]] * 4),
        *([[0.18, 0.82, 0.45, 0.95]] * 4),
        *([[0.18, 0.62, 1.0, 0.95]] * 4),
        *([[0.72, 0.42, 1.0, 0.95]] * 4),
        *([[1.0, 0.38, 0.18, 0.95]] * 4),
    ],
    dtype=np.float32,
)


def _add_frame_zero_skeleton(xml: str) -> str:
    """向 viewer URDF 追加可动态更新的 21 点/20 骨段目标骨架。"""
    root_match = re.search(r"<link name=\"([^\"]+)\"", xml)
    if root_match is None:
        raise ValueError("URDF 缺少根 link，无法追加 frame0 骨架")
    if not xml.rstrip().endswith("</robot>"):
        raise ValueError("URDF 缺少 </robot> 结尾，无法追加 frame0 骨架")
    root_link = root_match.group(1)
    visuals = []
    for index in range(21):
        color = _POINT_COLORS[index]
        visuals.append(
            f"""
    <visual name="frame0_kp_{index:02d}">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry><sphere radius="0.007" /></geometry>
      <material name="frame0_kp_mat_{index:02d}">
        <color rgba="{color[0]} {color[1]} {color[2]} 0.001" />
      </material>
    </visual>"""
        )
    for index, (parent, child) in enumerate(HAND_KEYPOINT_EDGES):
        color = _POINT_COLORS[child]
        visuals.append(
            f"""
    <visual name="frame0_bone_{index:02d}">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry><cylinder radius="0.0035" length="0.001" /></geometry>
      <material name="frame0_bone_mat_{index:02d}">
        <color rgba="{color[0]} {color[1]} {color[2]} 0.001" />
      </material>
    </visual>"""
        )
    # 彩色坐标系轴（长为全长的圆柱，中心在中点，由世界坐标定位）。
    #  两套轴统一用 RGB：+X=红、+Y=绿、+Z=蓝。
    #  Manus wrist：+X=指尖、+Y=手背、+Z=小拇指侧。
    #  wuji2 r_wrist：+X/+Y/+Z 为厂商 URDF link 本地轴。
    for axis_name in ("manus", "r_wrist"):
        for axis_index in range(3):
            rgba = (
                ("1 0 0 0.95", "0 1 0 0.95", "0 0 1 0.95")[axis_index]
            )
            visuals.append(
                f"""
    <visual name="ax_{axis_name}_{axis_index}">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry><cylinder radius="0.006" length="0.001" /></geometry>
      <material name="ax_{axis_name}_mat_{axis_index}">
        <color rgba="{rgba}" />
      </material>
    </visual>"""
            )
    fragment = f"""
  <link name="frame0_hand_skeleton">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.001" />
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
{''.join(visuals)}
  </link>
  <joint name="frame0_hand_skeleton_joint" type="floating">
    <parent link="{root_link}" />
    <child link="frame0_hand_skeleton" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
</robot>"""
    return xml.rstrip()[: -len("</robot>")] + fragment


def _quat_wxyz_from_z_axis(direction: np.ndarray) -> np.ndarray:
    """返回把局部 +Z 旋到 direction 的 MuJoCo wxyz 四元数。"""
    vector = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("骨段长度必须为正有限值")
    vector /= norm
    dot = float(np.clip(vector[2], -1.0, 1.0))
    if dot < -1.0 + 1.0e-9:
        return np.array([0.0, 1.0, 0.0, 0.0])
    w = np.sqrt(0.5 * (1.0 + dot))
    return np.array(
        [w, -vector[1] / (2.0 * w), vector[0] / (2.0 * w), 0.0]
    )

def _draw_axis(
    model, geom_id: int, origin: np.ndarray,
    direction: np.ndarray, length: float,
    color: np.ndarray,
) -> None:
    """把局部 +Z 圆柱轴放在 origin，方向为 direction，长度 length。"""
    direction = np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-9:
        model.geom_rgba[geom_id, 3] = 0.0
        return
    unit = direction / norm
    model.geom_pos[geom_id] = origin + 0.5 * length * unit
    model.geom_quat[geom_id] = _quat_wxyz_from_z_axis(unit)
    model.geom_size[geom_id, 1] = 0.5 * length
    model.geom_rgba[geom_id] = color


def _frame_from_axis_geoms(
    data,
    axis_x_geom_id: int,
    axis_z_geom_id: int,
    axis_half_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    axis_x = data.geom_xmat[axis_x_geom_id].reshape(3, 3)[:, 2].copy()
    axis_z = data.geom_xmat[axis_z_geom_id].reshape(3, 3)[:, 2].copy()
    axis_x /= np.linalg.norm(axis_x)
    axis_z /= np.linalg.norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)
    axis_y /= np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    rotation = np.column_stack((axis_x, axis_y, axis_z))
    origin_x = (
        data.geom_xpos[axis_x_geom_id]
        - axis_half_length * axis_x
    )
    origin_z = (
        data.geom_xpos[axis_z_geom_id]
        - axis_half_length * axis_z
    )
    return 0.5 * (origin_x + origin_z), rotation


def _sim_from_motive_rotation(
    home_tcp_rotation_mj: np.ndarray,
    tianji_config=None,
) -> np.ndarray:
    """固定世界轴：Motive→Robot world→MuJoCo。"""
    config = tianji_config or get_config()
    tcp_rotation_chest = Rotation.from_quat(
        config.init_quat["right"]
    ).as_matrix()
    tcp_rotation_world = (
        get_chest_to_world_rotation("right") @ tcp_rotation_chest
    )
    rotation_sim_from_world = (
        home_tcp_rotation_mj @ tcp_rotation_world.T
    )
    result = (
        rotation_sim_from_world
        @ np.asarray(config.mocap_to_robot, dtype=np.float64)
    )
    if not np.allclose(
        result @ result.T, np.eye(3), atol=1.0e-4
    ) or not np.isclose(np.linalg.det(result), 1.0, atol=1.0e-4):
        raise ValueError("Motive→MuJoCo 世界轴映射不是 det=+1 正交矩阵")
    return result


class FrameZeroHandSkeleton:
    """固定世界轴映射 frame0；Motive r_mount 与 r_wrist Home 定位模型。"""

    def __init__(self, session, model, topic: str):
        self._pending: dict | None = None
        self._received_once = False
        self._sim_from_motive: tuple[np.ndarray, np.ndarray] | None = None
        self._tianji_config = get_config()
        self._wrist_axis_x_geom_id = self._required_geom(
            model, "r_wrist_axis_0"
        )
        self._wrist_axis_z_geom_id = self._required_geom(
            model, "r_wrist_axis_2"
        )
        self._tcp_axis_x_geom_id = self._required_geom(
            model, "TCP_Link_R_axis_0"
        )
        self._tcp_axis_z_geom_id = self._required_geom(
            model, "TCP_Link_R_axis_2"
        )
        self._point_geom_ids = [
            self._required_geom(model, f"frame0_kp_{index:02d}")
            for index in range(21)
        ]
        self._bone_geom_ids = [
            self._required_geom(model, f"frame0_bone_{index:02d}")
            for index in range(len(HAND_KEYPOINT_EDGES))
        ]
        self._manus_axis_ids = [
            self._required_geom(model, f"ax_manus_{index}")
            for index in range(3)
        ]
        self._wrist_axis_ids = [
            self._required_geom(model, f"ax_r_wrist_{index}")
            for index in range(3)
        ]
        self._wrist_axis_fk_ids = [
            self._required_geom(model, f"r_wrist_axis_{index}")
            for index in range(3)
        ]
        for geom_id in (
            self._point_geom_ids
            + self._bone_geom_ids
            + self._manus_axis_ids
            + self._wrist_axis_ids
        ):
            model.geom_sameframe[geom_id] = 0
            model.geom_rgba[geom_id, 3] = 0.0
        # 隐藏 URDF 自带的 TCP/marker/r_wrist 坐标轴（只保留上面两套
        # 动态轴）；geom 仍保留用于 FK 位姿读取。
        for prefix in (
            "TCP_Link_R_axis_",
            "marker_mocap_axis_",
            "r_wrist_axis_",
        ):
            for index in range(3):
                geom_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_GEOM,
                    f"{prefix}{index}",
                )
                if geom_id >= 0:
                    model.geom_rgba[geom_id, 3] = 0.0
        self._sub = ZenohJsonSub(
            session, key(topic), self._on_skeleton
        )
        _LOG.info("等待 frame0 手部关键点骨架：%s", topic)

    @staticmethod
    def _required_geom(model, name: str) -> int:
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, name
        )
        if geom_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少 frame0 骨架 geom：{name}")
        return geom_id


    @property
    def received_once(self) -> bool:
        return self._received_once

    def _on_skeleton(self, msg: dict) -> None:
        self._pending = msg

    def apply_latest(self, model, data) -> bool:
        payload = self._pending
        if payload is None:
            return False
        self._pending = None
        try:
            points_motive = np.asarray(
                payload["points_motive_world"], dtype=np.float64
            )
            home_pose_motive = np.asarray(
                payload["home_wuji2_wrist_pose_motive"],
                dtype=np.float64,
            )
            frame0_manus_quat = np.asarray(
                payload["frame0_manus_quat_xyzw"], dtype=np.float64
            )
            edges = tuple(
                tuple(int(value) for value in edge)
                for edge in payload["edges"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            _LOG.warning("忽略无效 frame0 骨架消息：%s", exc)
            return False
        if (
            points_motive.shape != (21, 3)
            or home_pose_motive.shape != (7,)
            or frame0_manus_quat.shape != (4,)
            or not np.isfinite(points_motive).all()
            or not np.isfinite(home_pose_motive).all()
            or not np.isfinite(frame0_manus_quat).all()
            or np.linalg.norm(frame0_manus_quat) < 1.0e-9
            or edges != HAND_KEYPOINT_EDGES
        ):
            _LOG.warning("忽略形状/拓扑不匹配的 frame0 骨架消息")
            return False

        frozen = bool(payload.get("frozen", False))
        if not frozen or self._sim_from_motive is None:
            home_position_mj, _home_wrist_rotation = (
                _frame_from_axis_geoms(
                    data,
                    self._wrist_axis_x_geom_id,
                    self._wrist_axis_z_geom_id,
                    0.045,
                )
            )
            _home_tcp_position, home_tcp_rotation = (
                _frame_from_axis_geoms(
                    data,
                    self._tcp_axis_x_geom_id,
                    self._tcp_axis_z_geom_id,
                    0.025,
                )
            )
            rotation_mj_from_motive = _sim_from_motive_rotation(
                home_tcp_rotation, self._tianji_config
            )
            translation_mj_from_motive = (
                home_position_mj
                - rotation_mj_from_motive @ home_pose_motive[:3]
            )
            self._sim_from_motive = (
                rotation_mj_from_motive,
                translation_mj_from_motive,
            )
        rotation_mj_from_motive, translation_mj_from_motive = (
            self._sim_from_motive
        )
        points_mj = (
            points_motive @ rotation_mj_from_motive.T
            + translation_mj_from_motive
        )

        for index, geom_id in enumerate(self._point_geom_ids):
            model.geom_pos[geom_id] = points_mj[index]
            model.geom_rgba[geom_id] = _POINT_COLORS[index]
        for index, ((parent, child), geom_id) in enumerate(
            zip(HAND_KEYPOINT_EDGES, self._bone_geom_ids)
        ):
            start = points_mj[parent]
            end = points_mj[child]
            delta = end - start
            length = float(np.linalg.norm(delta))
            if length < 1.0e-6:
                model.geom_rgba[geom_id, 3] = 0.0
                continue
            model.geom_pos[geom_id] = 0.5 * (start + end)
            model.geom_quat[geom_id] = _quat_wxyz_from_z_axis(delta)
            model.geom_size[geom_id, 1] = 0.5 * length
            color = _POINT_COLORS[child].copy()
            color[3] = 0.78
            model.geom_rgba[geom_id] = color
        self._update_axes(model, data, points_mj, frame0_manus_quat)
        if not self._received_once:
            self._received_once = True
            _LOG.info(
                "已显示 H5 frame%d 的 21 点/20 骨段目标骨架",
                int(payload.get("source_frame_index", 0)),
            )
        return True

    @staticmethod
    def _draw_axis(
        model, geom_id: int, origin: np.ndarray,
        direction: np.ndarray, length: float,
        color: np.ndarray,
    ) -> None:
        _draw_axis(model, geom_id, origin, direction, length, color)

    def _update_axes(
        self, model, data, points_mj: np.ndarray,
        frame0_manus_quat: np.ndarray,
    ) -> None:
        # Manus wrist 轴：原点在腕点，+X=指尖(红)、+Y=手背(绿)、+Z=小拇指侧(蓝)。
        wrist_origin = points_mj[0]
        rotation_mj_from_motive, _ = self._sim_from_motive
        manus_rotation_motive = (
            frame0_manus_quat / np.linalg.norm(frame0_manus_quat)
        )
        x, y, z, w = manus_rotation_motive
        R_manus_motive = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        R_manus_mj = rotation_mj_from_motive @ R_manus_motive
        tip_axis = R_manus_mj[:, 0]
        back_axis = R_manus_mj[:, 1]
        pinky_axis = R_manus_mj[:, 2]
        self._draw_axis(
            model, self._manus_axis_ids[0], wrist_origin,
            tip_axis, 0.16, np.array([1.0, 0.0, 0.0, 0.95]),
        )
        self._draw_axis(
            model, self._manus_axis_ids[1], wrist_origin,
            back_axis, 0.16, np.array([0.0, 1.0, 0.0, 0.95]),
        )
        self._draw_axis(
            model, self._manus_axis_ids[2], wrist_origin,
            pinky_axis, 0.16, np.array([0.0, 0.0, 1.0, 0.95]),
        )
        # wuji2 r_wrist 轴：从组合 URDF 自带 r_wrist_axis_* 的 FK 位姿
        # 实时读取——每个 geom 的 col2 是该轴世界方向，轴中点减去半长
        # 得到 r_wrist 原点，随机械臂 FK 运动。
        wrist_origin_fk = np.zeros(3)
        wrist_axes_fk = np.zeros((3, 3))
        for axis_index, fk_id in enumerate(self._wrist_axis_fk_ids):
            unit = data.geom_xmat[fk_id].reshape(3, 3)[:, 2].copy()
            unit /= np.linalg.norm(unit) + 1.0e-12
            origin = data.geom_xpos[fk_id] - (0.045 * unit)
            wrist_origin_fk += origin
            wrist_axes_fk[:, axis_index] = unit
        wrist_origin_fk /= 3.0
        # 正交化（URDF 轴 geom 可能未严格正交），以 X 为主方向。
        wrist_x = wrist_axes_fk[:, 0].copy()
        wrist_x /= np.linalg.norm(wrist_x) + 1.0e-12
        wrist_y_candidate = wrist_axes_fk[:, 1]
        wrist_y = (
            wrist_y_candidate
            - np.dot(wrist_y_candidate, wrist_x) * wrist_x
        )
        wrist_y /= np.linalg.norm(wrist_y) + 1.0e-12
        wrist_z = np.cross(wrist_x, wrist_y)
        wrist_z /= np.linalg.norm(wrist_z) + 1.0e-12
        self._draw_axis(
            model, self._wrist_axis_ids[0], wrist_origin_fk,
            wrist_x, 0.12, np.array([1.0, 0.0, 0.0, 0.95]),
        )
        self._draw_axis(
            model, self._wrist_axis_ids[1], wrist_origin_fk,
            wrist_y, 0.12, np.array([0.0, 1.0, 0.0, 0.95]),
        )
        self._draw_axis(
            model, self._wrist_axis_ids[2], wrist_origin_fk,
            wrist_z, 0.12, np.array([0.0, 0.0, 1.0, 0.95]),
        )

    def close(self) -> None:
        try:
            self._sub.close()
        except Exception:
            pass


def _parse_args():
    return parse_cli_args(
        extra={
            "--urdf": {
                "type": Path,
                "default": DEFAULT_URDF,
                "help": "MuJoCo 专用 Marvin URDF",
            },
            "--topic": {
                "default": "/pico_body_sim/model_joint_states",
                "help": "JointState JSON 输入话题",
            },
            "--frame0-skeleton-topic": {
                "default": "",
                "help": "frame0 手部 21 点/20 骨段目标话题；空值禁用",
            },
        }
    )


def main() -> None:
    args = _parse_args()
    xml, assets = portable_mujoco_urdf(args.urdf)
    if args.frame0_skeleton_topic:
        xml = _add_frame_zero_skeleton(xml)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    session = open_session()
    mirror = MujocoJointMirror(session, model, args.topic)
    skeleton = (
        FrameZeroHandSkeleton(
            session, model, args.frame0_skeleton_topic
        )
        if args.frame0_skeleton_topic
        else None
    )
    started = time.monotonic()
    warned = False
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.0, 0.0, 1.05]
            viewer.cam.distance = 2.8
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -18.0

            while viewer.is_running():
                with viewer.lock():
                    if mirror.apply_latest(data):
                        mujoco.mj_forward(model, data)
                    if (
                        skeleton is not None
                        and mirror.received_once
                        and skeleton.apply_latest(model, data)
                    ):
                        mujoco.mj_forward(model, data)
                viewer.sync()
                if (
                    not mirror.received_once
                    and not warned
                    and time.monotonic() - started > 3.0
                ):
                    _LOG.warning("尚未收到预览关节；请先运行 run_preview.sh")
                    warned = True
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        mirror.close()
        if skeleton is not None:
            skeleton.close()


if __name__ == "__main__":
    handler = logging.StreamHandler()
    handler.terminator = "\n\n"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[handler],
    )
    main()
