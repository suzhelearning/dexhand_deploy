#!/usr/bin/env python3
"""机械臂+wuji2 场景的 H5 数据回放（不启动 IK、不控制机械臂）。

加载组合 URDF 并摆到 sim_mocap_h5 的 Home 关节角；订阅 Motive
``tianji_wrist`` 只用于经 marker 安装链定位 ``r_mount/r_wrist``，
世界轴固定使用 ``Motive→Robot world→MuJoCo``。H5 右手 21 点世界
坐标据此映射到 MuJoCo，并按时间轴播放；机械臂关节保持 Home 不变。

与 ``sim_mocap_h5`` 的区别：本脚本不驱动 IK、不发布目标话题，只读取
一次实时 tianji_wrist 原点标定并可视化 H5 数据。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from pico_body_tianji.controller_only.mocap_h5 import (
    HAND_KEYPOINT_EDGES,
    compose_pose,
    load_mocap_h5,
)
from pico_body_tianji.controller_only.mocap_h5_replay_node import (
    DEFAULT_PARAMETERS,
    _WUJI2_MOUNT_TO_WRIST_POSE,
)
from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from pico_body_tianji.zenoh_util import LiveToken
from tianji_world_output.config_loader import get_config

from mujoco_joint_viewer import (
    _POINT_COLORS,
    _add_frame_zero_skeleton,
    _draw_axis,
    _frame_from_axis_geoms,
    _quat_wxyz_from_z_axis,
    _sim_from_motive_rotation,
)


_LOG = logging.getLogger("mocap_h5_wrist_replay")

_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_CONFIG = (
    _ROOT
    / "src/pico_body_tianji/config/mode/controller_only"
    / "controller_only_ik.yaml"
)

# Home 关节角（度）从 controller_only_ik.yaml 读取，与 sim_mocap_h5
# 使用同一配置源，保证机械臂 Home 位姿完全一致。
# 顺序与 urdf_joint_names() 一致：左臂 Joint1-7，右臂 Joint1-7。
_LEFT_HOME_KEY = "left_home_deg"
_RIGHT_HOME_KEY = "right_home_deg"

# firmware 序 20 关节名（tianji_wuji2.urdf 的 revolute 声明顺序）。
# 旧 hand2_beta1 资产的小指带 "_finger_" 后缀，作为别名兼容。
WUJI2_HAND_JOINT_NAMES = [
    "r_thumb_cmc_flex", "r_thumb_cmc_abd", "r_thumb_mcp", "r_thumb_ip",
    "r_index_finger_mcp_flex", "r_index_finger_mcp_abd",
    "r_index_finger_pip", "r_index_finger_dip",
    "r_middle_finger_mcp_flex", "r_middle_finger_mcp_abd",
    "r_middle_finger_pip", "r_middle_finger_dip",
    "r_ring_finger_mcp_flex", "r_ring_finger_mcp_abd",
    "r_ring_finger_pip", "r_ring_finger_dip",
    "r_pinky_mcp_flex", "r_pinky_mcp_abd",
    "r_pinky_pip", "r_pinky_dip",
]
WUJI2_HAND_JOINT_ALIASES = {
    "r_pinky_mcp_flex": "r_pinky_finger_mcp_flex",
    "r_pinky_mcp_abd": "r_pinky_finger_mcp_abd",
    "r_pinky_pip": "r_pinky_finger_pip",
    "r_pinky_dip": "r_pinky_finger_dip",
}

HAND_COMMANDS_KEY = "pico_body_sim/right_hand/joint_commands"
HAND_KEYPOINTS_KEY = "pico_body_sim/right_hand/keypoints"
TELEOP_STATE_KEY = "pico_body_sim/right_hand/teleop_state"


class _HandCommandCache:
    """订阅桥发布的 20 关节命令，保留最新帧。"""

    def __init__(self) -> None:
        self._latest: np.ndarray | None = None

    def on_sample(self, sample) -> None:
        data = bytes(sample.payload)
        if len(data) == 20 * 4:
            self._latest = np.frombuffer(data, dtype=np.float32).copy()

    def get(self) -> np.ndarray | None:
        return self._latest


def _hand_joint_qpos_ids(model) -> list[int]:
    """按 firmware 序解析 URDF 关节在 MuJoCo 模型中的 qpos 偏移。"""
    ids = []
    for name in WUJI2_HAND_JOINT_NAMES:
        candidate = name
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            candidate = WUJI2_HAND_JOINT_ALIASES.get(name, "")
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, candidate)
        if joint_id < 0:
            raise ValueError(f"组合 URDF 缺少手部关节: {name}")
        ids.append(model.jnt_qposadr[joint_id])
    return ids


def _apply_hand_commands(model, data, qpos_ids, commands) -> None:
    if commands is None:
        return
    for qpos_index, value in zip(qpos_ids, commands):
        data.qpos[qpos_index] = float(value)


def _load_home_joints(config_path: Path | None) -> np.ndarray:
    """读取 controller_only_ik.yaml 的 left/right_home_deg 为 14 关节角（度）。"""
    path = config_path or _DEFAULT_CONFIG
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    node = data
    for segment in ("tianji_kinematic_sim", "ros__parameters"):
        node = node.get(segment, node)
    left = np.asarray(node[_LEFT_HOME_KEY], dtype=np.float64)
    right = np.asarray(node[_RIGHT_HOME_KEY], dtype=np.float64)
    if left.shape != (7,) or right.shape != (7,):
        raise ValueError(
            f"Home 关节角必须为 7 值，实际 left={left.shape} right={right.shape}"
        )
    return np.concatenate((left, right))


def _configured_pose(prefix: str) -> np.ndarray:
    position = np.asarray(
        DEFAULT_PARAMETERS[f"{prefix}_translation_m"], dtype=np.float64
    )
    quaternion = np.asarray(
        DEFAULT_PARAMETERS[f"{prefix}_quaternion_xyzw"], dtype=np.float64
    )
    values = np.concatenate((position, quaternion))
    return np.concatenate(
        (
            values[:3],
            values[3:] / np.linalg.norm(values[3:]),
        )
    )


def _read_right_arm_pose(spec: str | None) -> np.ndarray:
    """获取 tianji_wrist 刚体的 Motive 位姿（x,y,z,qx,qy,qz,qw）。

    给定 spec 时直接解析；否则订阅 mocap/hands/frame 取当前 id=3。
    """
    if spec:
        values = np.asarray(
            [float(part) for part in spec.split(",")], dtype=np.float64
        )
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(
                f"--right-arm-pose 需要 7 个数，实际 {values.shape}"
            )
        values[3:7] /= np.linalg.norm(values[3:7])
        return values
    import json
    import zenoh
    session = zenoh.open(zenoh.Config())
    holder: dict[str, np.ndarray] = {}

    def on_frame(sample) -> None:
        try:
            msg = json.loads(bytes(sample.payload))
        except (ValueError, TypeError):
            return
        for body in msg.get("rigid_bodies", []):
            if body.get("id") != 3 or not body.get("tracking_valid"):
                continue
            position = body.get("position")
            quaternion = body.get("quaternion_xyzw")
            if not position or not quaternion:
                continue
            values = np.asarray(
                list(position) + list(quaternion), dtype=np.float64
            )
            if np.isfinite(values).all():
                holder["pose"] = values
                return

    subscriber = session.declare_subscriber(
        "mocap/hands/frame", on_frame
    )
    _LOG.info("等待 Motive tianji_wrist 位姿（订阅 mocap/hands/frame）...")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if "pose" in holder:
            subscriber.undeclare()
            session.close()
            return holder["pose"]
        time.sleep(0.05)
    subscriber.undeclare()
    session.close()
    raise TimeoutError(
        "未在 5s 内收到 tianji_wrist 位姿；请用 --right-arm-pose 指定，"
        "或确认 Motive windows_pub.sh 在运行"
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "机械臂+wuji2 场景 H5 手部数据回放；机械臂保持 Home，"
            "只移动手部骨架；不启动 IK，只读 tianji_wrist 定位原点"
        )
    )
    parser.add_argument("h5", type=Path)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument(
        "--config",
        type=Path,
        default=_DEFAULT_CONFIG,
        help="controller_only_ik.yaml（含 left/right_home_deg）",
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--hold-s", type=float, default=2.0,
                        help="首帧停留秒数")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=0.0,
        help="绕 Motive 竖直轴(+Z)旋转整条手部轨迹",
    )
    parser.add_argument(
        "--hand-commands",
        action="store_true",
        help=(
            "订阅 wuji_hand2_bridge 的 retarget 输出"
            "（pico_body_sim/right_hand/joint_commands）并驱动手部关节；"
            "自动启用 --wuji2 合并 URDF"
        ),
    )
    parser.add_argument(
        "--right-arm-pose",
        type=str,
        default=None,
        help=(
            "tianji_wrist 刚体在 Motive 系的位姿 "
            "'x,y,z,qx,qy,qz,qw'；用于定位动捕原点。缺省时自动订阅 "
            "mocap/hands/frame 读取当前 id=3。"
        ),
    )
    return parser




def _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                     points_mj: np.ndarray) -> None:
    for index, geom_id in enumerate(point_geom_ids):
        model.geom_pos[geom_id] = points_mj[index]
        model.geom_rgba[geom_id] = _POINT_COLORS[index]
    for index, ((parent, child), geom_id) in enumerate(
        zip(HAND_KEYPOINT_EDGES, bone_geom_ids)
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


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.speed > 0.0:
        raise ValueError("--speed 必须为正数")
    if args.hold_s < 0.0:
        raise ValueError("--hold-s 必须非负")

    recording = load_mocap_h5(args.h5)
    hand = recording.hands["right"]
    valid_indices = np.flatnonzero(hand.valid)
    if valid_indices.size == 0:
        raise ValueError("H5 右侧手腕无有效帧")
    start_index = int(valid_indices[0])
    end_index = int(valid_indices[-1])

    yaw_rotation = Rotation.from_rotvec(
        np.array([0.0, np.deg2rad(args.yaw_deg), 0.0])
    ).as_matrix()

    root = Path(__file__).resolve().parents[3]
    default_urdf = (
        root / "src/pico_body_tianji/assets/tianji_wuji2" / "tianji_wuji2.urdf"
    )
    urdf_path = args.urdf or default_urdf
    xml, assets = portable_mujoco_urdf(urdf_path)
    xml = _add_frame_zero_skeleton(xml)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    # 从 controller_only_ik.yaml 读取 Home 关节角（度），与 sim_mocap_h5
    # 使用同一配置源，保证机械臂 Home 位姿完全一致。
    home_joints = _load_home_joints(args.config)
    for name, angle_deg in zip(urdf_joint_names(), home_joints):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少关节：{name}")
        data.qpos[model.jnt_qposadr[joint_id]] = np.deg2rad(
            float(angle_deg)
        )
    mujoco.mj_forward(model, data)

    # 可选：订阅 wuji_hand2_bridge 的 retarget 命令并驱动手指关节；
    # 同时把当前帧手腕相对键点发布给桥（仿真验收链路无需主机节点）。
    hand_cache = None
    hand_qpos_ids = []
    hand_session = None
    hand_keypoints_pub = None
    hand_state_pub = None
    hand_live = None
    last_hand_state = None
    if args.hand_commands:
        import zenoh

        hand_cache = _HandCommandCache()
        hand_qpos_ids = _hand_joint_qpos_ids(model)
        hand_session = zenoh.open(zenoh.Config())
        hand_session.declare_subscriber(
            HAND_COMMANDS_KEY, hand_cache.on_sample
        )
        hand_keypoints_pub = hand_session.declare_publisher(
            HAND_KEYPOINTS_KEY
        )
        hand_state_pub = hand_session.declare_publisher(TELEOP_STATE_KEY)
        hand_live = LiveToken(hand_session, "wuji_hand_replay")
        _LOG.info(
            "已订阅手部命令: %s（等 wuji_hand2_bridge 发布 retarget 输出）",
            HAND_COMMANDS_KEY,
        )

    def publish_frame_keypoints(frame_index: int) -> None:
        if hand_keypoints_pub is None:
            return
        frame_points = points_motive[frame_index]
        relative = frame_points - frame_points[0]
        hand_keypoints_pub.put(relative.astype("<f4").tobytes())

    def publish_hand_state(state: str) -> None:
        nonlocal last_hand_state
        if hand_state_pub is None or state == last_hand_state:
            return
        hand_state_pub.put(state.encode("utf-8"))
        last_hand_state = state
        _LOG.info("右手回放 teleop_state=%s", state)

    point_geom_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"frame0_kp_{index:02d}")
        for index in range(21)
    ]
    bone_geom_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"frame0_bone_{index:02d}")
        for index in range(len(HAND_KEYPOINT_EDGES))
    ]
    manus_axis_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"ax_manus_wrist_{index}")
        for index in range(3)
    ]
    wrist_axis_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"ax_r_wrist_{index}")
        for index in range(3)
    ]
    wrist_axis_fk_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"r_wrist_axis_{index}")
        for index in range(3)
    ]
    for geom_id in (
        point_geom_ids + bone_geom_ids + manus_axis_ids + wrist_axis_ids
    ):
        if geom_id < 0:
            raise RuntimeError(
                f"MuJoCo 模型缺少 frame0 骨架 geom：{geom_id}"
            )
        model.geom_sameframe[geom_id] = 0
        model.geom_rgba[geom_id, 3] = 0.0
    # 隐藏全部 URDF 静态调试轴，只保留三套动态轴。
    for prefix in (
        "TCP_Link_L_axis_", "TCP_Link_R_axis_",
        "marker_mocap_axis_", "marker_mocap_l_axis_", "marker_mocap_r_axis_",
        "l_mount_axis_", "r_mount_axis_",
        "l_wrist_axis_", "r_wrist_axis_",
    ):
        for index in range(3):
            geom_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, f"{prefix}{index}"
            )
            if geom_id >= 0:
                model.geom_rgba[geom_id, 3] = 0.0

    wrist_axis_x = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "r_wrist_axis_0"
    )
    wrist_axis_z = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "r_wrist_axis_2"
    )
    tcp_axis_x = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "TCP_Link_R_axis_0"
    )
    tcp_axis_z = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "TCP_Link_R_axis_2"
    )
    if min(wrist_axis_x, wrist_axis_z, tcp_axis_x, tcp_axis_z) < 0:
        raise RuntimeError(
            "纯数据回放需要 --wuji2 组合 URDF（缺少 TCP/r_wrist 坐标轴）"
        )
    home_position_mj, _home_wrist_rotation_mj = (
        _frame_from_axis_geoms(
            data, wrist_axis_x, wrist_axis_z, 0.045
        )
    )
    _home_tcp_position_mj, home_tcp_rotation_mj = (
        _frame_from_axis_geoms(
            data, tcp_axis_x, tcp_axis_z, 0.025
        )
    )

    # Home 关节角、TCP Home 位姿与 mocap_to_robot 必须来自同一机器人配置。
    tianji_config = get_config()
    configured_home = np.concatenate(
        (
            np.asarray(tianji_config.init_joints["left"]),
            np.asarray(tianji_config.init_joints["right"]),
        )
    )
    if not np.allclose(home_joints, configured_home, atol=1.0e-6):
        raise ValueError(
            "controller_only_ik.yaml Home 关节角与 tianji_robot.yaml "
            "init_joints 不一致，无法建立确定的世界轴映射"
        )

    # 固定世界轴：Motive→Robot world→MuJoCo。
    # tianji_wrist marker 的局部姿态不参与该旋转。
    rotation_sim_from_motive = _sim_from_motive_rotation(
        home_tcp_rotation_mj, tianji_config
    )

    # tianji_wrist 仅定位动捕原点：其姿态把 GL/GO、marker→r_mount
    # 和厂商 r_mount→r_wrist 平移旋转到 Motive 世界。
    rigid_pose = _read_right_arm_pose(args.right_arm_pose)
    marker_home_motive = compose_pose(
        rigid_pose,
        _configured_pose("right_rigid_to_marker_mocap"),
    )
    mount_home_motive = compose_pose(
        marker_home_motive,
        _configured_pose("right_marker_to_mount"),
    )
    wrist_home_motive = compose_pose(
        mount_home_motive,
        _WUJI2_MOUNT_TO_WRIST_POSE,
    )
    translation_sim_from_motive = (
        home_position_mj
        - rotation_sim_from_motive @ wrist_home_motive[:3]
    )
    _LOG.info(
        "坐标标定：Motive +X→Sim %s，+Y→Sim %s，+Z→Sim %s；"
        "原点平移=%s；tianji_wrist 姿态仅用于局部偏置",
        np.round(rotation_sim_from_motive[:, 0], 4).tolist(),
        np.round(rotation_sim_from_motive[:, 1], 4).tolist(),
        np.round(rotation_sim_from_motive[:, 2], 4).tolist(),
        np.round(translation_sim_from_motive, 4).tolist(),
    )

    def motive_to_sim(points: np.ndarray) -> np.ndarray:
        return (
            points @ rotation_sim_from_motive.T
            + translation_sim_from_motive
        )

    # H5 关键点（Motive 系）经 yaw + 动捕原点变换直接映射到 MuJoCo。
    keypoints_h5 = hand.keypoints_world  # (N,21,3) Motive 系
    # 丢弃 NaN 帧：用最近的有效帧前向填充，避免零长度骨段。
    finite_mask = np.isfinite(keypoints_h5).all(axis=(1, 2))
    keypoints_h5 = keypoints_h5.copy()
    last_finite = None
    for frame_index in range(recording.frame_count):
        if finite_mask[frame_index]:
            last_finite = keypoints_h5[frame_index].copy()
        elif last_finite is not None:
            keypoints_h5[frame_index] = last_finite
    if last_finite is None:
        raise ValueError("H5 右侧手部关键点全部无效")
    points_motive = (
        keypoints_h5 @ yaw_rotation.T
    )
    points_mj = motive_to_sim(points_motive)

    def pose_at(frame_index: int) -> np.ndarray:
        return points_mj[frame_index]

    # 相机对准骨架轨迹中心（映射后腕点均值），而非机械臂手腕，
    # 保证骨架移动全程在视野内。
    trajectory_center = np.mean(
        points_mj[start_index:end_index + 1, 0], axis=0
    )

    time_ns = recording.time_ns
    start_ns = int(time_ns[start_index])
    duration_s = float(time_ns[end_index] - start_ns) / 1.0e9

    _LOG.info(
        "H5 数据回放：%s；右手有效=%d/%d；首帧=%d 末帧=%d；时长=%.3fs；"
        "speed=%g；机械臂保持 Home；只读 tianji_wrist 原点，不启动 IK",
        recording.path,
        int(hand.valid.sum()),
        recording.frame_count,
        start_index,
        end_index,
        duration_s,
        args.speed,
    )

    if args.validate_only:
        points_start = pose_at(start_index)
        points_end = pose_at(end_index)
        _LOG.info("首帧腕点=%s 末帧腕点=%s",
                  points_start[0].round(4).tolist(),
                  points_end[0].round(4).tolist())
        _LOG.info("H5 与组合 URDF 校验通过")
        return 0

    def apply_frame_axes(frame_index: int, points_mj: np.ndarray) -> None:
        wrist_origin = points_mj[0]
        quat = hand.wrist[frame_index, 3:7].copy()
        if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1.0e-9:
            return
        quat /= np.linalg.norm(quat)
        R_manus_sim = (
            rotation_sim_from_motive @ Rotation.from_quat(quat).as_matrix()
        )
        manus_colors = (
            np.array([1.0, 0.35, 0.35, 0.62]),
            np.array([0.35, 1.0, 0.35, 0.62]),
            np.array([0.35, 0.55, 1.0, 0.62]),
        )
        actual_colors = (
            np.array([1.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0, 1.0]),
        )
        for i in range(3):
            _draw_axis(
                model, manus_axis_ids[i], wrist_origin,
                R_manus_sim[:, i], 0.18, manus_colors[i],
            )
        wrist_origin_fk, wrist_rotation_fk = _frame_from_axis_geoms(
            data, wrist_axis_x, wrist_axis_z, 0.045
        )
        for i in range(3):
            _draw_axis(
                model, wrist_axis_ids[i], wrist_origin_fk,
                wrist_rotation_fk[:, i], 0.115, actual_colors[i],
            )

    if args.headless:
        publish_hand_state("teleop")
        for frame_index in range(start_index, end_index + 1):
            frame_points = pose_at(frame_index)
            _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                             frame_points)
            apply_frame_axes(frame_index, frame_points)
            publish_frame_keypoints(frame_index)
            if hand_cache is not None:
                _apply_hand_commands(
                    model, data, hand_qpos_ids, hand_cache.get()
                )
            mujoco.mj_forward(model, data)
        publish_hand_state("returning")
        time.sleep(2.0)
        publish_hand_state("idle")
        if hand_live is not None:
            hand_live.close()
        if hand_session is not None:
            hand_session.close()
        _LOG.info("无窗口回放完成：%d 帧", end_index - start_index + 1)
        return 0

    controls = {"paused": bool(args.paused), "restart": False}
    finished = False

    def on_key(keycode: int) -> None:
        nonlocal finished
        if keycode == 32:
            if finished:
                controls["restart"] = True
                controls["paused"] = False
                finished = False
                _LOG.info("已到末帧：重新从首帧播放")
            else:
                controls["paused"] = not controls["paused"]
                _LOG.info("%s", "暂停" if controls["paused"] else "继续")
        elif keycode == 82:
            controls["restart"] = True
            controls["paused"] = False
            finished = False
            _LOG.info("从首帧重新开始")

    _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                     pose_at(start_index))
    elapsed_s = 0.0
    last_wall = time.monotonic()
    last_frame = -1
    last_log_at = time.monotonic()
    publish_hand_state("idle" if controls["paused"] else "teleop")
    _LOG.info(
        "开始数据回放：Space 暂停/继续（已到末帧时重新播放），R 从首帧重播，"
        "关闭窗口退出；loop=%s，speed=%g",
        args.loop,
        args.speed,
    )
    try:
        with mujoco.viewer.launch_passive(
            model, data, key_callback=on_key
        ) as viewer:
            viewer.cam.lookat[:] = trajectory_center
            viewer.cam.distance = 1.5
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -25.0

            while viewer.is_running():
                now = time.monotonic()
                wall_step = max(0.0, now - last_wall)
                last_wall = now
                if controls["restart"]:
                    controls["restart"] = False
                    elapsed_s = 0.0
                if not controls["paused"]:
                    elapsed_s += wall_step * args.speed

                if args.loop:
                    elapsed_s %= max(duration_s, 1.0e-9)
                else:
                    elapsed_s = min(elapsed_s, duration_s)
                if (
                    not args.loop
                    and not controls["paused"]
                    and elapsed_s >= duration_s
                    and not finished
                ):
                    finished = True
                    controls["paused"] = True
                    _LOG.info("已播放到末帧，暂停；按 Space 重新从头播放")
                desired_hand_state = (
                    "returning" if finished else
                    ("idle" if controls["paused"] else "teleop")
                )
                publish_hand_state(desired_hand_state)
                progress = elapsed_s / max(duration_s, 1.0e-9)
                frame_index = start_index + int(
                    progress * (end_index - start_index)
                )
                frame_index = int(
                    np.clip(frame_index, start_index, end_index)
                )
                frame_changed = frame_index != last_frame
                with viewer.lock():
                    if frame_changed:
                        frame_points = pose_at(frame_index)
                        _update_skeleton(
                            model, data, point_geom_ids, bone_geom_ids,
                            frame_points,
                        )
                        apply_frame_axes(frame_index, frame_points)
                        publish_frame_keypoints(frame_index)
                        last_frame = frame_index
                    if hand_cache is not None:
                        _apply_hand_commands(
                            model, data, hand_qpos_ids,
                            hand_cache.get(),
                        )
                    if frame_changed or hand_cache is not None:
                        mujoco.mj_forward(model, data)
                if time.monotonic() - last_log_at >= 2.0:
                    last_log_at = time.monotonic()
                    _LOG.info(
                        "进度 %.1f%%  frame=%d/%d  腕点=%s",
                        progress * 100.0,
                        frame_index,
                        end_index,
                        np.round(pose_at(frame_index)[0], 3).tolist(),
                    )
                viewer.sync()
                time.sleep(1.0 / 120.0)
    except KeyboardInterrupt:
        pass
    finally:
        publish_hand_state("returning")
        # 给手桥斜坡回零留出时间；真机退出仍需观察 zero_hold。
        time.sleep(1.5 if hand_state_pub is not None else 0.0)
        publish_hand_state("idle")
        if hand_live is not None:
            hand_live.close()
        if hand_session is not None:
            hand_session.close()
    return 0


if __name__ == "__main__":
    handler = logging.StreamHandler()
    handler.terminator = "\n\n"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[handler],
    )
    raise SystemExit(main())
