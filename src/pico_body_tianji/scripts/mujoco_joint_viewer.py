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

from pico_body_tianji.protocol import topics
from pico_body_tianji.protocol.messages import (
    ArmJointCommand,
    ArmSolvedPose,
    ArmTargetCommand,
    Frame0HandSkeleton,
    ProtocolError,
)
from pico_body_tianji.sources.mocap.h5 import (
    HAND_KEYPOINT_EDGES, compose_pose, invert_pose,
)
from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_joint_state import apply_joint_positions
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from pico_body_tianji.zenoh_util import ZenohJsonSub, key, open_session, parse_cli_args
from tianji_world_output.config_loader import get_config
from tianji_world_output.transform_utils import get_chest_to_world_rotation


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


# wuji-sdk firmware 序；新 tianji_wuji2 小指无 _finger_，旧 beta1 为别名。
_WUJI2_RIGHT_JOINT_NAMES = [
    "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
    "r_index_finger_mcp_flex", "r_index_finger_mcp_abd",
    "r_index_finger_pip", "r_index_finger_dip",
    "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd",
    "r_middle_finger_pip", "r_middle_finger_dip",
    "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd",
    "r_ring_finger_pip", "r_ring_finger_dip",
    "r_pinky_mcp_flex", "r_pinky_mcp_abd", "r_pinky_pip", "r_pinky_dip",
]
_WUJI2_RIGHT_JOINT_ALIASES = {
    "r_pinky_mcp_flex": "r_pinky_finger_mcp_flex",
    "r_pinky_mcp_abd": "r_pinky_finger_mcp_abd",
    "r_pinky_pip": "r_pinky_finger_pip",
    "r_pinky_dip": "r_pinky_finger_dip",
}


class WujiHandCommandMirror:
    """镜像 wuji_hand2_bridge 发布的 20×float32 firmware 序关节命令。"""

    def __init__(self, session, model, topic: str):
        self._qpos_addresses = []
        for name in _WUJI2_RIGHT_JOINT_NAMES:
            candidate = name
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, candidate
            )
            if joint_id < 0:
                candidate = _WUJI2_RIGHT_JOINT_ALIASES.get(name, "")
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, candidate
                )
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo 模型缺少 wuji2 手关节：{name}")
            self._qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        self._pending: np.ndarray | None = None
        self._received_once = False

        def on_sample(sample) -> None:
            payload = bytes(sample.payload)
            if len(payload) != 20 * 4:
                return
            self._pending = np.frombuffer(payload, dtype="<f4").copy()

        self._sub = session.declare_subscriber(key(topic), on_sample)
        _LOG.info("等待 wuji2 手部关节命令：%s", topic)

    @property
    def received_once(self) -> bool:
        return self._received_once

    def apply_latest(self, data) -> int:
        pending = self._pending
        self._pending = None
        if pending is None:
            return 0
        for qpos_address, value in zip(self._qpos_addresses, pending):
            data.qpos[qpos_address] = float(value)
        if not self._received_once:
            self._received_once = True
            _LOG.info("已接收 wuji2 20 关节命令；MuJoCo 开始镜像手指")
        return len(self._qpos_addresses)

    def close(self) -> None:
        try:
            self._sub.undeclare()
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
    # 两套动态坐标轴：
    # - manus_wrist：H5 frame0 原始 W frame，细/半透明/长；
    # - r_wrist：机器人当前 FK B frame，纯 RGB、粗/短，随机械臂移动。
    # W->B 目标仅用于数值误差诊断，不画第三套轴。
    axis_styles = {
        "manus_wrist": (
            0.003,
            ("1 0.35 0.35 0.62", "0.35 1 0.35 0.62", "0.35 0.55 1 0.62"),
        ),
        "r_wrist": (
            0.007,
            ("1 0 0 1", "0 1 0 1", "0 0 1 1"),
        ),
    }
    for axis_name, (radius, colors) in axis_styles.items():
        for axis_index in range(3):
            visuals.append(
                f"""
    <visual name="ax_{axis_name}_{axis_index}">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry><cylinder radius="{radius}" length="0.001" /></geometry>
      <material name="ax_{axis_name}_mat_{axis_index}">
        <color rgba="{colors[axis_index]}" />
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
        self._pending_target: dict | None = None
        self._pending_solved: dict | None = None
        self._received_once = False
        self._sim_from_motive: tuple[np.ndarray, np.ndarray] | None = None
        self._target_origin_mj: np.ndarray | None = None
        self._target_rotation_mj: np.ndarray | None = None
        self._target_tcp_pose_chest: np.ndarray | None = None
        self._solved_tcp_pose_chest: np.ndarray | None = None
        self._tcp_to_wrist_pose: np.ndarray | None = None
        self.last_position_error_mm: float | None = None
        self.last_rotation_error_deg: float | None = None
        self._last_error_log_at = -float("inf")
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
            self._required_geom(model, f"ax_manus_wrist_{index}")
            for index in range(3)
        ]
        self._wrist_axis_ids = [
            self._required_geom(model, f"ax_r_wrist_{index}")
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
            "TCP_Link_L_axis_",
            "TCP_Link_R_axis_",
            "marker_mocap_axis_",      # 旧组合 URDF
            "marker_mocap_l_axis_",    # 最新双手 URDF
            "marker_mocap_r_axis_",
            "l_mount_axis_",
            "r_mount_axis_",
            "l_wrist_axis_",
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
        self._target_sub = ZenohJsonSub(
            session,
            topics.arm_target("right"),
            self._on_target_pose,
        )
        self._solved_sub = ZenohJsonSub(
            session,
            topics.arm_solved_pose("right"),
            self._on_solved_pose,
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

    @staticmethod
    def _pose_from_message(msg: dict) -> np.ndarray | None:
        try:
            p = msg["position"]
            q = msg["orientation"]
            pose = np.array([
                p["x"], p["y"], p["z"],
                q["x"], q["y"], q["z"], q["w"],
            ], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            return None
        if not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1.0e-9:
            return None
        pose[3:] /= np.linalg.norm(pose[3:])
        return pose

    def _on_target_pose(self, msg: dict) -> None:
        self._pending_target = msg

    def _on_solved_pose(self, msg: dict) -> None:
        self._pending_solved = msg

    def _apply_pending_producer_messages(self) -> bool:
        changed = False
        if self._pending_target is not None:
            payload = self._pending_target
            self._pending_target = None
            try:
                target = ArmTargetCommand.from_dict(payload)
                if target.side != "right" or target.frame_id != "Base_R":
                    raise ProtocolError("frame0 target must be right Base_R")
                self._target_tcp_pose_chest = np.concatenate(
                    (target.position_m, target.orientation_xyzw)
                )
                changed = True
            except (ProtocolError, TypeError, ValueError) as exc:
                _LOG.warning("忽略无效 canonical arm target: %s", exc)
        if self._pending_solved is not None:
            payload = self._pending_solved
            self._pending_solved = None
            try:
                solved = ArmSolvedPose.from_dict(payload)
                if solved.side != "right" or solved.frame_id != "Base_R":
                    raise ProtocolError("frame0 solved pose must be right Base_R")
                self._solved_tcp_pose_chest = np.concatenate(
                    (solved.position_m, solved.orientation_xyzw)
                )
                changed = True
            except (ProtocolError, TypeError, ValueError) as exc:
                _LOG.warning("忽略无效 canonical solved pose: %s", exc)
        return changed

    def apply_latest(self, model, data) -> bool:
        producer_changed = self._apply_pending_producer_messages()

        payload = self._pending
        if payload is None:
            return producer_changed
        self._pending = None
        try:
            skeleton = Frame0HandSkeleton.from_dict(payload)
        except (ProtocolError, TypeError, ValueError) as exc:
            _LOG.warning("忽略无效 frame0 骨架消息：%s", exc)
            return False
        points_motive = np.asarray(skeleton.keypoints_world_m, dtype=np.float64)
        home_pose_motive = np.asarray(skeleton.robot_wrist_home_pose, dtype=np.float64)
        manus_wrist_quat_motive = np.asarray(skeleton.manus_wrist_pose[3:], dtype=np.float64)
        target_wrist_pose_motive = np.asarray(skeleton.target_wrist_pose, dtype=np.float64)
        tcp_to_wrist_pose = np.asarray(skeleton.tcp_to_wrist_pose, dtype=np.float64)
        edges = tuple(tuple(int(value) for value in edge) for edge in skeleton.edges)
        if (
            points_motive.shape != (21, 3)
            or home_pose_motive.shape != (7,)
            or manus_wrist_quat_motive.shape != (4,)
            or target_wrist_pose_motive.shape != (7,)
            or tcp_to_wrist_pose.shape != (7,)
            or not np.isfinite(points_motive).all()
            or not np.isfinite(home_pose_motive).all()
            or not np.isfinite(manus_wrist_quat_motive).all()
            or not np.isfinite(target_wrist_pose_motive).all()
            or not np.isfinite(tcp_to_wrist_pose).all()
            or edges != HAND_KEYPOINT_EDGES
        ):
            _LOG.warning("忽略形状/拓扑不匹配的 frame0 骨架消息")
            return False

        self._tcp_to_wrist_pose = tcp_to_wrist_pose.copy()
        self._tcp_to_wrist_pose[3:] /= np.linalg.norm(
            self._tcp_to_wrist_pose[3:]
        )
        frozen = True
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
        self._update_reference_axes(
            model,
            data,
            points_mj,
            manus_wrist_quat_motive,
            target_wrist_pose_motive[3:],
        )
        self.update_fk_axes(model, data, force_log=bool(frozen))
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

    def _update_reference_axes(
        self, model, data, points_mj: np.ndarray,
        manus_wrist_quat: np.ndarray,
        target_wrist_quat: np.ndarray,
    ) -> None:
        """绘制原始 Manus W 与转换后的目标 Wuji B 两套固定参考轴。"""
        wrist_origin = points_mj[0]
        rotation_mj_from_motive, _ = self._sim_from_motive
        R_manus_motive = Rotation.from_quat(
            manus_wrist_quat / np.linalg.norm(manus_wrist_quat)
        ).as_matrix()
        R_target_motive = Rotation.from_quat(
            target_wrist_quat / np.linalg.norm(target_wrist_quat)
        ).as_matrix()
        R_manus_mj = rotation_mj_from_motive @ R_manus_motive
        R_target_mj = rotation_mj_from_motive @ R_target_motive
        self._target_origin_mj = wrist_origin.copy()
        self._target_rotation_mj = R_target_mj.copy()

        # Manus W：长 180mm、细、半透明浅 RGB。
        manus_colors = (
            np.array([1.0, 0.35, 0.35, 0.62]),
            np.array([0.35, 1.0, 0.35, 0.62]),
            np.array([0.35, 0.55, 1.0, 0.62]),
        )
        for axis_index, geom_id in enumerate(self._manus_axis_ids):
            self._draw_axis(
                model, geom_id, wrist_origin,
                R_manus_mj[:, axis_index], 0.18,
                manus_colors[axis_index],
            )

    def update_fk_axes(self, model, data, *, force_log: bool = False) -> bool:
        """每个关节帧更新当前 FK r_wrist，并记录目标/FK 位姿误差。"""
        if self._target_origin_mj is None or self._target_rotation_mj is None:
            return False
        # 只使用 X/Z 圆柱恢复共同原点和右手系。Y 圆柱局部 +Z 指向 -Y，
        # 不能按统一 center-half*localZ 公式处理。
        wrist_origin_fk, wrist_rotation_fk = _frame_from_axis_geoms(
            data,
            self._wrist_axis_x_geom_id,
            self._wrist_axis_z_geom_id,
            0.045,
        )
        target_source = "Motive preview"
        if (
            self._target_tcp_pose_chest is not None
            and self._solved_tcp_pose_chest is not None
            and self._tcp_to_wrist_pose is not None
        ):
            target_wrist_chest = compose_pose(
                self._target_tcp_pose_chest, self._tcp_to_wrist_pose
            )
            solved_wrist_chest = compose_pose(
                self._solved_tcp_pose_chest, self._tcp_to_wrist_pose
            )
            current_wrist_mj = np.concatenate((
                wrist_origin_fk,
                Rotation.from_matrix(wrist_rotation_fk).as_quat(),
            ))
            mj_from_chest = compose_pose(
                current_wrist_mj, invert_pose(solved_wrist_chest)
            )
            target_wrist_mj = compose_pose(
                mj_from_chest, target_wrist_chest
            )
            self._target_origin_mj = target_wrist_mj[:3].copy()
            self._target_rotation_mj = Rotation.from_quat(
                target_wrist_mj[3:]
            ).as_matrix()
            target_source = "IK target/solved"

        actual_colors = (
            np.array([1.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0, 1.0]),
        )
        for axis_index, geom_id in enumerate(self._wrist_axis_ids):
            self._draw_axis(
                model, geom_id, wrist_origin_fk,
                wrist_rotation_fk[:, axis_index], 0.115,
                actual_colors[axis_index],
            )

        position_error_mm = 1000.0 * float(
            np.linalg.norm(wrist_origin_fk - self._target_origin_mj)
        )
        rotation_error_deg = float(np.rad2deg(
            Rotation.from_matrix(
                self._target_rotation_mj.T @ wrist_rotation_fk
            ).magnitude()
        ))
        self.last_position_error_mm = position_error_mm
        self.last_rotation_error_deg = rotation_error_deg
        now = time.monotonic()
        if force_log or now - self._last_error_log_at >= 0.5:
            _LOG.info(
                "frame0 r_wrist 目标↔FK[%s]：位置误差=%.2fmm 姿态误差=%.2f°",
                target_source,
                position_error_mm,
                rotation_error_deg,
            )
            self._last_error_log_at = now
        return True

    def close(self) -> None:
        for sub in (self._sub, self._target_sub, self._solved_sub):
            try:
                sub.close()
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
                "default": topics.FRAME0_HAND_SKELETON,
                "help": "canonical frame0 手部 21 点/20 骨段目标话题；空值禁用",
            },
            "--hand-commands-topic": {
                "default": "",
                "help": "wuji2 20×float32 关节命令话题；空值禁用",
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
    hand = (
        WujiHandCommandMirror(session, model, args.hand_commands_topic)
        if args.hand_commands_topic
        else None
    )
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
                    changed = bool(mirror.apply_latest(data))
                    if hand is not None and hand.apply_latest(data):
                        changed = True
                    if changed:
                        # 先刷新手臂/手指 FK，随后才能读取当前 r_wrist frame。
                        mujoco.mj_forward(model, data)
                    skeleton_changed = False
                    if skeleton is not None and mirror.received_once:
                        skeleton_changed = skeleton.apply_latest(model, data)
                        if not skeleton_changed and changed:
                            skeleton_changed = skeleton.update_fk_axes(
                                model, data
                            )
                    if skeleton_changed:
                        # 动态轴/骨架写入 model.geom_* 后再刷新显示坐标。
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
        if hand is not None:
            hand.close()
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
