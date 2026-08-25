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
    manus_axis_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                          f"ax_manus_{index}")
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
    # 隐藏 URDF 自带的 TCP/marker/r_wrist 轴，只保留两套动态轴。
    for prefix in (
        "TCP_Link_R_axis_",
        "marker_mocap_axis_",
        "r_wrist_axis_",
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
        if not np.isfinite(quat).all():
            return
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm < 1.0e-9:
            return
        quat /= quat_norm
        x, y, z, w = quat
        R_manus_motive = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        R_manus_sim = rotation_sim_from_motive @ R_manus_motive
        _draw_axis(
            model, manus_axis_ids[0], wrist_origin,
            R_manus_sim[:, 0], 0.16, np.array([1.0, 0.0, 0.0, 0.95]),
        )
        _draw_axis(
            model, manus_axis_ids[1], wrist_origin,
            R_manus_sim[:, 1], 0.16, np.array([0.0, 1.0, 0.0, 0.95]),
        )
        _draw_axis(
            model, manus_axis_ids[2], wrist_origin,
            R_manus_sim[:, 2], 0.16, np.array([0.0, 0.0, 1.0, 0.95]),
        )
        # r_wrist 轴：组合 URDF 自带 r_wrist_axis_* 的 FK geom。
        wrist_origin_fk = np.zeros(3)
        wrist_axes_fk = np.zeros((3, 3))
        for axis_index, fk_id in enumerate(wrist_axis_fk_ids):
            unit = data.geom_xmat[fk_id].reshape(3, 3)[:, 2].copy()
            unit /= np.linalg.norm(unit) + 1.0e-12
            wrist_origin_fk += (
                data.geom_xpos[fk_id] - (0.045 * unit)
            )
            wrist_axes_fk[:, axis_index] = unit
        wrist_origin_fk /= 3.0
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
        _draw_axis(
            model, wrist_axis_ids[0], wrist_origin_fk,
            wrist_x, 0.12, np.array([1.0, 0.0, 0.0, 0.95]),
        )
        _draw_axis(
            model, wrist_axis_ids[1], wrist_origin_fk,
            wrist_y, 0.12, np.array([0.0, 1.0, 0.0, 0.95]),
        )
        _draw_axis(
            model, wrist_axis_ids[2], wrist_origin_fk,
            wrist_z, 0.12, np.array([0.0, 0.0, 1.0, 0.95]),
        )

    if args.headless:
        for frame_index in range(start_index, end_index + 1):
            frame_points = pose_at(frame_index)
            _update_skeleton(model, data, point_geom_ids, bone_geom_ids,
                             frame_points)
            apply_frame_axes(frame_index, frame_points)
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
                        frame_points = pose_at(frame_index)
                        _update_skeleton(
                            model, data, point_geom_ids, bone_geom_ids,
                            frame_points,
                        )
                        apply_frame_axes(frame_index, frame_points)
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
