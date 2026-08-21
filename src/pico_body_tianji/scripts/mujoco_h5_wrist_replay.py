#!/usr/bin/env python3
"""机械臂+wuji2 场景的纯 H5 数据回放（不启动 IK/Motive/Zenoh）。

加载机械臂+wuji2 组合 URDF 并摆到 sim_mocap_h5 的 Home 关节角（IK
求解 init_pos/init_quat 的确定性配置，非零位）；只把 H5 右手 21 点
关键点经固定 Manus→wuji2 外参转到 wuji2 局部坐标，再叠在 Home
r_wrist 上，按时间轴播放手部位姿移动。机械臂关节保持 Home 不变。

与 sim_mocap_h5 的区别：本脚本不驱动 IK，不读 Motive marker，不发布
目标话题，只做确定性数据可视化，用于离线核对 H5 手姿态在机器人场景
中的运动。
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
    load_mocap_h5,
)
from pico_body_tianji.controller_only.mocap_h5_replay_node import (
    DEFAULT_PARAMETERS,
)
from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf

from mujoco_joint_viewer import (
    _POINT_COLORS,
    _add_frame_zero_skeleton,
    _quat_wxyz_from_z_axis,
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
    """获取 right_arm 刚体在 Motive 系位姿（x,y,z,qx,qy,qz,qw）。

    给定 spec 时直接解析；否则订阅 mocap/hands/frame 取 id=3（right_arm）。
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
    _LOG.info("等待 Motive right_arm 位姿（订阅 mocap/hands/frame）...")
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
        "未在 5s 内收到 right_arm 位姿；请用 --right-arm-pose 指定，"
        "或确认 Motive windows_pub.sh 在运行"
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "机械臂+wuji2 场景纯 H5 手部数据回放；机械臂保持 Home，"
            "只移动手部骨架；不启动 IK/Motive/Zenoh"
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
        help="绕 Motive +Y 旋转整条手部轨迹",
    )
    parser.add_argument(
        "--right-arm-pose",
        type=str,
        default=None,
        help=(
            "right_arm 刚体在 Motive 系的位姿 'x,y,z,qx,qy,qz,qw'；"
            "用于定位动捕原点。缺省时自动订阅 mocap/hands/frame 读 right_arm。"
        ),
    )
    return parser


def _wrist_frame_mj(data, model, axis_x_geom_id: int,
                    axis_z_geom_id: int) -> tuple[np.ndarray, np.ndarray]:
    axis_x_matrix = data.geom_xmat[axis_x_geom_id].reshape(3, 3)
    axis_z_matrix = data.geom_xmat[axis_z_geom_id].reshape(3, 3)
    axis_x = axis_x_matrix[:, 2].copy()
    axis_z = axis_z_matrix[:, 2].copy()
    axis_x /= np.linalg.norm(axis_x)
    axis_z /= np.linalg.norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)
    axis_y /= np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    rotation = np.column_stack((axis_x, axis_y, axis_z))
    origin_x = data.geom_xpos[axis_x_geom_id] - 0.045 * axis_x
    origin_z = data.geom_xpos[axis_z_geom_id] - 0.045 * axis_z
    return 0.5 * (origin_x + origin_z), rotation


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
        root
        / "src/pico_body_tianji/assets/marvin_m6_ccs/urdf"
        / "marvin_m6_s_ccs_696_v4_wuji2.urdf"
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
    for geom_id in point_geom_ids + bone_geom_ids:
        if geom_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少 frame0 骨架 geom：{geom_id}")
        model.geom_sameframe[geom_id] = 0
        model.geom_rgba[geom_id, 3] = 0.0

    axis_x_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "r_wrist_axis_0"
    )
    axis_z_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "r_wrist_axis_2"
    )
    if axis_x_geom_id < 0 or axis_z_geom_id < 0:
        raise RuntimeError(
            "纯数据回放需要 --wuji2 组合 URDF（缺少 r_wrist 坐标轴 geom）"
        )
    home_position_mj, home_rotation_mj = _wrist_frame_mj(
        data, model, axis_x_geom_id, axis_z_geom_id
    )

    # 定位动捕原点：right_arm 刚体（Motive 系）经外参推导 r_wrist Home
    # 在 Motive 系，再与 MuJoCo Home r_wrist 对齐，建立 Motive->MuJoCo 变换。
    rigid_to_marker = _configured_pose("right_rigid_to_marker_mocap")
    marker_to_wrist = _configured_pose("right_marker_to_wrist")
    rigid_pose = _read_right_arm_pose(args.right_arm_pose)
    # compose_pose(a, b) 语义：p = p_a + R_a @ p_b，R = R_a @ R_b。
    marker_position = (
        rigid_pose[:3]
        + Rotation.from_quat(rigid_pose[3:7]).as_matrix()
        @ rigid_to_marker[:3]
    )
    marker_rotation = (
        Rotation.from_quat(rigid_pose[3:7])
        * Rotation.from_quat(rigid_to_marker[3:7])
    )
    marker_pose_full = np.concatenate(
        (marker_position, marker_rotation.as_quat())
    )
    wrist_position = (
        marker_pose_full[:3]
        + marker_rotation.as_matrix() @ marker_to_wrist[:3]
    )
    wrist_rotation = (
        marker_rotation * Rotation.from_quat(marker_to_wrist[3:7])
    )
    wrist_home_motive = np.concatenate(
        (wrist_position, wrist_rotation.as_quat())
    )
    rotation_home_motive = Rotation.from_quat(
        wrist_home_motive[3:7]
    ).as_matrix()
    rotation_sim_from_motive = (
        home_rotation_mj @ rotation_home_motive.T
    )
    translation_sim_from_motive = (
        home_position_mj
        - rotation_sim_from_motive @ wrist_home_motive[:3]
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
        "纯数据回放：%s；右手有效=%d/%d；首帧=%d 末帧=%d；时长=%.3fs；"
        "speed=%g；机械臂保持 Home，仅移动手部骨架；不启动 IK/Motive/Zenoh",
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

    if args.headless:
        for frame_index in range(start_index, end_index + 1):
            _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                             pose_at(frame_index))
            mujoco.mj_forward(model, data)
        _LOG.info("无窗口回放完成：%d 帧", end_index - start_index + 1)
        return 0

    controls = {"paused": bool(args.paused), "restart": False}
    finished = False

    def on_key(keycode: int) -> None:
        nonlocal finished
        if keycode == 32:
            if finished:
                controls["restart"] = True
                finished = False
                _LOG.info("已到末帧：重新从首帧播放")
            else:
                controls["paused"] = not controls["paused"]
                _LOG.info("%s", "暂停" if controls["paused"] else "继续")
        elif keycode == 82:
            controls["restart"] = True
            finished = False
            _LOG.info("从首帧重新开始")

    _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                     pose_at(start_index))
    elapsed_s = 0.0
    last_wall = time.monotonic()
    last_frame = -1
    last_log_at = time.monotonic()
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
                progress = elapsed_s / max(duration_s, 1.0e-9)
                frame_index = start_index + int(
                    progress * (end_index - start_index)
                )
                frame_index = int(
                    np.clip(frame_index, start_index, end_index)
                )
                if frame_index != last_frame:
                    with viewer.lock():
                        _update_skeleton(
                            model, data, point_geom_ids, bone_geom_ids,
                            pose_at(frame_index),
                        )
                        mujoco.mj_forward(model, data)
                    last_frame = frame_index
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
